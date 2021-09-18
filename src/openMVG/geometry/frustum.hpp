// This file is part of OpenMVG, an Open Multiple View Geometry C++ library.

// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_GEOMETRY_FRUSTUM_HPP
#define OPENMVG_GEOMETRY_FRUSTUM_HPP

#include <string>
#include <vector>

#include "openMVG/geometry/half_space_intersection.hpp"

namespace openMVG
{

/**
* @brief Namespace holding various classes and methods for geometry manipulation
*/
namespace geometry
{

using namespace openMVG::geometry::halfPlane;

/**
* @brief Define a camera Frustum:
*  - infinite Frustum (4 Half Spaces) (a pyramid)
*  - truncated Frustum (6 Half Spaces) (a truncated pyramid)
*  - This structure is used for testing frustum intersection (see if two cam can share visual content)
*/
struct Frustum : public HalfPlaneObject
{
  /// Camera centre and the 4 points that define the image plane
  Vec3 cones[5];

  /// Near clip plane distance
  double z_near;

  /// Far clip plane distance
  double z_far;

  /// Support points
  std::vector<Vec3> points;

  /**
  * @brief Default constructor
  */
  Frustum();

  /**
  * @brief Build a frustum from the image size, camera intrinsic and pose
  * @param w Width of image plane
  * @param h Height of image plane
  * @param K Intrinsic matrix
  * @param R Extrinsic rotation matrix
  * @param C Center of the camera (optical center)
  */
  Frustum
  (
    const int w,
    const int h,
    const Mat3 & K,
    const Mat3 & R,
    const Vec3 & C
  );

  /**
  * @brief Build a frustum from the image size, camera intrinsic and pose
  * @param w Width of image plane
  * @param h Height of image plane
  * @param K Intrinsic matrix
  * @param R Extrinsic rotation matrix
  * @param C Center of the camera (optical center)
  * @param Specify a far plane
  */
  Frustum
  (
    const int w,
    const int h,
    const Mat3 & K,
    const Mat3 & R,
    const Vec3 & C ,
    const double zFar
  );

  /**
  * @brief Build a frustum from image size, camera intrinsics, pose and clip planes
  * @param w Width of image plane
  * @param h Height of image plane
  * @param K Intrinsic matrix
  * @param R Extrinsic rotation matrix
  * @param C Center of the camera (optical center)
  * @param zNear Near clip plane distance
  * @param zFar Far clip plane distance
  */
  Frustum
  (
    const int w,
    const int h,
    const Mat3 & K,
    const Mat3 & R,
    const Vec3 & C,
    const double zNear,
    const double zFar
  );

  /**
  * @brief Tells if the Frustum is an infinite one
  * @retval true If frustum is infinite
  * @retval false If frustum is not infinite
  */
  bool isInfinite() const;

  /**
  * @brief Tells if the Frustum is truncated
  * @retval true If frustum is truncated
  * @retval false If frustum is not truncated
  */
  bool isTruncated() const;

  // /**
  // * @brief Test if two frustums intersect
  // * A prior radial check for speed up
  // * @retval true If an non-empty intersection exists
  // * @retval false If there is no intersection
  // */
  // bool intersect(const Frustum & rhs) const
  // {
  //   // std::cout << "Intersecting with frustum" << std::endl;
  //   if (this->isTruncated() && rhs.isTruncated())
  //   {
  //     const Vec3& frustumA_center = this->cones[0];
  //     const Vec3& frustumB_center = rhs.cones[0];
  //     // edge from center to outer face
  //     const double frustumA_edge = (this->points.back() - frustumA_center).norm();
  //     const double frustumB_edge = (rhs.points.back() - frustumB_center).norm();
  //     const double frustums_distance = (frustumB_center - frustumA_center).norm();

  //     if (frustums_distance > frustumA_edge + frustumB_edge)
  //     {
  //       return false;
  //     }
  //   }

  //   return HalfPlaneObject::intersect(rhs);
  // }

  // bool intersect(const HalfPlaneObject & rhs) const
  // {
  //   return HalfPlaneObject::intersect(rhs);
  // }

  /**
  * @brief Return the supporting frustum points
  * @return Supporting frustum points
  * @note 5 points for infinite frustum
  * @note 8 points for truncated frustum
  */
  const std::vector<Vec3> & frustum_points() const;

  /**
  * @brief Export the Frustum as a PLY file (infinite frustum as exported as a normalized cone)
  * @return true if the file can be saved on disk
  */
  static bool export_Ply
  (
    const Frustum & frustum,
    const std::string & filename
  );

}; // struct Frustum

} // namespace geometry
} // namespace openMVG

#endif // OPENMVG_GEOMETRY_FRUSTUM_HPP
