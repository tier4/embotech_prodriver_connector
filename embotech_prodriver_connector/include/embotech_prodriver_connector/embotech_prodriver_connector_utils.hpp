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
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace embotech_prodriver_connector
{
using tier4_autoware_utils::Polygon2d;

PTCL_ObjectType to_PTCL_object_type(const ObjectClassification & classification);

UTMPoint convert_LatLon_to_UTM_coordinate(const lanelet::GPSPoint & p_target);
lanelet::GPSPoint convert_UTM_to_LatLon_coordinate(const UTMPoint & p_target);
MGRSPoint convert_to_MGRS_Point(const PTCL_Position & ptcl_point);

PredictedPath get_highest_prob_path(const PredictedObjectKinematics object);

Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & length, const double & width);

Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const Shape & obj_shape);

Polygon2d convertPolygonObjectToGeometryPolygon(const Pose & current_pose, const Shape & obj_shape);

void calcObjectPolygon(const Shape & shape, const Pose & pose, Polygon2d * object_polygon);

ObjectClassification getHighestProbClass(const std::vector<ObjectClassification> & classifications);

}  // namespace embotech_prodriver_connector
