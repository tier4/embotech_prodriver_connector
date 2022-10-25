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

#include "embotech_prodriver_connector/embotech_prodriver_connector_utils.hpp"

#include "embotech_prodriver_connector/embotech_prodriver_connector.hpp"

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

  const int N = 20;
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

ObjectClassification getHighestProbClass(const std::vector<ObjectClassification> & classifications)
{
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
