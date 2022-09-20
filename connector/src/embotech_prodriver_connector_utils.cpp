// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "embotech_prodriver_connector/embotech_prodriver_connector.hpp"
#include "embotech_prodriver_connector/embotech_prodriver_connector_utils.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <unistd.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

namespace embotech_prodriver_connector
{

PTCL_ObjectType to_PTCL_object_type(const ObjectClassification & classification)
{
  const auto label = classification.label;

  if (label == ObjectClassification::CAR) {
    return PTCL_ObjectType::PTCL_OBJECT_TYPE_CAR;
  }
  if (label == ObjectClassification::TRUCK) {
    return PTCL_ObjectType::PTCL_OBJECT_TYPE_TRUCK;
  }
  if (label == ObjectClassification::BUS) {
    return PTCL_ObjectType::PTCL_OBJECT_TYPE_TRUCK;  // TODO: think later
  }
  if (label == ObjectClassification::TRAILER) {
    return PTCL_ObjectType::PTCL_OBJECT_TYPE_TRUCK;  // TODO: think later
  }
  if (label == ObjectClassification::MOTORCYCLE) {
    return PTCL_ObjectType::PTCL_OBJECT_TYPE_MOTORCYCLE;
  }
  if (label == ObjectClassification::BICYCLE) {
    return PTCL_ObjectType::PTCL_OBJECT_TYPE_MOTORCYCLE;  // TODO: think later
  }
  if (label == ObjectClassification::PEDESTRIAN) {
    return PTCL_ObjectType::PTCL_OBJECT_TYPE_PEDESTRIAN;
  }

  return PTCL_ObjectType::PTCL_OBJECT_TYPE_UNKNOWN;
}

UTMPoint convert_LatLon_to_UTM_coordinate(const lanelet::GPSPoint & p_target)
{
  // TODO(K.Sugahara): this is from the Turbo87/UTM repository. Take right way to manage this code.
  // https://github.com/Turbo87/utm/blob/master/utm/conversion.py#L190-L286

  // parameters
  constexpr auto K0 = 0.9996;

  constexpr auto E = 0.00669438;
  constexpr auto E2 = E * E;
  constexpr auto E3 = E2 * E;
  constexpr auto E_P2 = E / (1 - E);

  constexpr auto M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256);
  constexpr auto M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024);
  constexpr auto M3 = (15 * E2 / 256 + 45 * E3 / 1024);
  constexpr auto M4 = (35 * E3 / 3072);

  constexpr auto R = 6378137;

  constexpr double LATLON_TO_RAD = M_PI / 180.0;

  const auto zone_number_to_central_longitude = [](const auto zone_number) {
    return (zone_number - 1) * 6 - 180 + 3;
  };

  const auto mod_angle = [](const auto value) { return std::fmod(value + M_PI, 2 * M_PI) - M_PI; };

  const auto latlon_to_zone_number = [](const auto latitude, const auto longitude) {
    if ((56 <= latitude && latitude < 64) && (3 <= longitude && longitude < 12)) return 32;
    if ((72 <= latitude && latitude <= 84) && (longitude >= 0)) {
      if (longitude < 9) {
        return 31;
      } else if (longitude < 21) {
        return 33;
      } else if (longitude < 33) {
        return 35;
      } else if (longitude < 42) {
        return 37;
      }
    }
    return static_cast<int>((longitude + 180) / 6) + 1;
  };

  const auto lat_rad = p_target.lat * LATLON_TO_RAD;
  const auto lat_sin = std::sin(lat_rad);
  const auto lat_cos = std::cos(lat_rad);

  const auto lat_tan = lat_sin / lat_cos;
  const auto lat_tan2 = lat_tan * lat_tan;
  const auto lat_tan4 = lat_tan2 * lat_tan2;

  const auto zone_number = latlon_to_zone_number(p_target.lat, p_target.lon);

  if (zone_number != 54) {
    throw std::runtime_error("TEMP: zone_number must be 54 in this test");
  }

  // const auto zone_letter = 'S'; // unused

  const auto lon_rad = p_target.lon * LATLON_TO_RAD;
  const auto central_lon = zone_number_to_central_longitude(zone_number);

  const auto central_lon_rad = central_lon * LATLON_TO_RAD;

  const auto n = R / std::sqrt(1 - E * lat_sin * lat_sin);
  const auto c = E_P2 * lat_cos * lat_cos;

  const auto a = lat_cos * mod_angle(lon_rad - central_lon_rad);
  const auto a2 = a * a;
  const auto a3 = a2 * a;
  const auto a4 = a3 * a;
  const auto a5 = a4 * a;
  const auto a6 = a5 * a;

  const auto m = R * (M1 * lat_rad - M2 * std::sin(2 * lat_rad) + M3 * std::sin(4 * lat_rad) -
                      M4 * std::sin(6 * lat_rad));

  const auto easting = K0 * n *
                         (a + a3 / 6 * (1 - lat_tan2 + c) +
                          a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) +
                       500000;

  auto northing = K0 * (m + n * lat_tan *
                              (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) +
                               a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)));

  if (p_target.lat < 0) {
    northing += 10000000;
  }

  return {easting, northing, 0.0};
}

PredictedPath get_highest_prob_path(const PredictedObjectKinematics object)
{
  const auto & paths = object.predicted_paths;
  size_t highest_idx = 0;
  float highest_conf = 0.0;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths.at(i).confidence > highest_conf) {
      highest_conf = paths.at(i).confidence;
      highest_idx = i;
    }
  }
  return paths.at(highest_idx);
}

Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & length, const double & width)
{
  Polygon2d object_polygon;

  tf2::Transform tf_map2obj;
  tf2::fromMsg(current_pose, tf_map2obj);

  // set vertices at map coordinate
  tf2::Vector3 p1_map;
  p1_map.setX(length / 2);
  p1_map.setY(width / 2);
  p1_map.setZ(0.0);
  p1_map.setW(1.0);

  tf2::Vector3 p2_map;
  p2_map.setX(-length / 2);
  p2_map.setY(width / 2);
  p2_map.setZ(0.0);
  p2_map.setW(1.0);

  tf2::Vector3 p3_map;
  p3_map.setX(-length / 2);
  p3_map.setY(-width / 2);
  p3_map.setZ(0.0);
  p3_map.setW(1.0);

  tf2::Vector3 p4_map;
  p4_map.setX(length / 2);
  p4_map.setY(-width / 2);
  p4_map.setZ(0.0);
  p4_map.setW(1.0);

  // transform vertices from map coordinate to object coordinate
  tf2::Vector3 p1_obj = tf_map2obj * p1_map;
  tf2::Vector3 p2_obj = tf_map2obj * p2_map;
  tf2::Vector3 p3_obj = tf_map2obj * p3_map;
  tf2::Vector3 p4_obj = tf_map2obj * p4_map;

  object_polygon.outer().emplace_back(p1_obj.x(), p1_obj.y());
  object_polygon.outer().emplace_back(p2_obj.x(), p2_obj.y());
  object_polygon.outer().emplace_back(p3_obj.x(), p3_obj.y());
  object_polygon.outer().emplace_back(p4_obj.x(), p4_obj.y());

  object_polygon.outer().push_back(object_polygon.outer().front());

  return object_polygon;
}

Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const Shape & obj_shape)
{
  Polygon2d object_polygon;

  const double obj_x = current_pose.position.x;
  const double obj_y = current_pose.position.y;

  constexpr int N = 20;
  const double r = obj_shape.dimensions.x / 2;
  for (int i = 0; i < N; ++i) {
    object_polygon.outer().emplace_back(
      obj_x + r * std::cos(2.0 * M_PI / N * i), obj_y + r * std::sin(2.0 * M_PI / N * i));
  }

  object_polygon.outer().push_back(object_polygon.outer().front());

  return object_polygon;
}

Polygon2d convertPolygonObjectToGeometryPolygon(const Pose & current_pose, const Shape & obj_shape)
{
  Polygon2d object_polygon;
  tf2::Transform tf_map2obj;
  fromMsg(current_pose, tf_map2obj);
  const auto obj_points = obj_shape.footprint.points;
  for (const auto & obj_point : obj_points) {
    tf2::Vector3 obj(obj_point.x, obj_point.y, obj_point.z);
    tf2::Vector3 tf_obj = tf_map2obj * obj;
    object_polygon.outer().emplace_back(tf_obj.x(), tf_obj.y());
  }
  object_polygon.outer().push_back(object_polygon.outer().front());

  return object_polygon;
}

void calcObjectPolygon(const Shape & shape, const Pose & pose, Polygon2d * object_polygon)
{
  if (shape.type == Shape::BOUNDING_BOX) {
    *object_polygon =
      convertBoundingBoxObjectToGeometryPolygon(pose, shape.dimensions.x, shape.dimensions.y);
  } else if (shape.type == Shape::CYLINDER) {
    *object_polygon = convertCylindricalObjectToGeometryPolygon(pose, shape);
  } else if (shape.type == Shape::POLYGON) {
    *object_polygon = convertPolygonObjectToGeometryPolygon(pose, shape);
  } else {
    throw std::runtime_error("invalid perception type");
  }
}

ObjectClassification getHighestProbClass(const std::vector<ObjectClassification> & classifications){
  float highest_prob = -1.0;
  ObjectClassification highest_class;
  for (const auto & c : classifications) {
    if (c.probability > highest_prob) {
      highest_class = c;
      highest_prob = c.probability;
    }
  }
  return highest_class;
}


}  // namespace embotech_prodriver_connector
