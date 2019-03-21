// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "PointCloud.h"

#include <Eigen/Dense>
#include <Open3D/Utility/Console.h>
#include <Open3D/Geometry/KDTreeFlann.h>

#include <iostream>
#include <iomanip>
using namespace std;

#ifdef OPEN3D_USE_CUDA
#include "Open3D/Utility/CUDA.cuh"
#endif

namespace open3d {
namespace geometry {

void PointCloud::Clear() {
    points_.clear();
    normals_.clear();
    colors_.clear();
}

bool PointCloud::IsEmpty() const { return !HasPoints(); }

Vec3d PointCloud::GetMinBound() const {
    if (!HasPoints()) {
        return Vec3d{};
    }
    auto itr_x = std::min_element(
            points_.begin(), points_.end(),
            [](const Vec3d &a, const Vec3d &b) {
                return a[0] < b[0];
            });
    auto itr_y = std::min_element(
            points_.begin(), points_.end(),
            [](const Vec3d &a, const Vec3d &b) {
                return a[1] < b[1];
            });
    auto itr_z = std::min_element(
            points_.begin(), points_.end(),
            [](const Vec3d &a, const Vec3d &b) {
                return a[2] < b[2];
            });
    return Vec3d{(*itr_x)[0], (*itr_y)[1], (*itr_z)[2]};
}

Vec3d PointCloud::GetMaxBound() const {
    if (!HasPoints()) {
        return Vec3d{};
    }
    auto itr_x = std::max_element(
            points_.begin(), points_.end(),
            [](const Vec3d &a, const Vec3d &b) {
                return a[0] < b[0];
            });
    auto itr_y = std::max_element(
            points_.begin(), points_.end(),
            [](const Vec3d &a, const Vec3d &b) {
                return a[1] < b[1];
            });
    auto itr_z = std::max_element(
            points_.begin(), points_.end(),
            [](const Vec3d &a, const Vec3d &b) {
                return a[2] < b[2];
            });
    return Vec3d{(*itr_x)[0], (*itr_y)[1], (*itr_z)[2]};
}

void PointCloud::Transform(const Eigen::Matrix4d &transformation) {
    for (auto &point : points_) {
        Eigen::Vector4d new_point =
                transformation *
                Eigen::Vector4d(point[0], point[1], point[2], 1.0);
        point = new_point.block<3, 1>(0, 0);
    }
    for (auto &normal : normals_) {
        Eigen::Vector4d new_normal =
                transformation *
                Eigen::Vector4d(normal[0], normal[1], normal[2], 0.0);
        normal = new_normal.block<3, 1>(0, 0);
    }
}

PointCloud &PointCloud::operator+=(const PointCloud &cloud) {
    // We do not use std::vector::insert to combine std::vector because it will
    // crash if the pointcloud is added to itself.
    if (cloud.IsEmpty()) return (*this);
    size_t old_vert_num = points_.size();
    size_t add_vert_num = cloud.points_.size();
    size_t new_vert_num = old_vert_num + add_vert_num;
    if ((!HasPoints() || HasNormals()) && cloud.HasNormals()) {
        normals_.resize(new_vert_num);
        for (size_t i = 0; i < add_vert_num; i++)
            normals_[old_vert_num + i] = cloud.normals_[i];
    } else {
        normals_.clear();
    }
    if ((!HasPoints() || HasColors()) && cloud.HasColors()) {
        colors_.resize(new_vert_num);
        for (size_t i = 0; i < add_vert_num; i++)
            colors_[old_vert_num + i] = cloud.colors_[i];
    } else {
        colors_.clear();
    }
    points_.resize(new_vert_num);
    for (size_t i = 0; i < add_vert_num; i++)
        points_[old_vert_num + i] = cloud.points_[i];
    return (*this);
}

PointCloud PointCloud::operator+(const PointCloud &cloud) const {
    return (PointCloud(*this) += cloud);
}

std::vector<double> ComputePointCloudToPointCloudDistance(
        const PointCloud &source, const PointCloud &target) {
    std::vector<double> distances(source.points_.size());
    KDTreeFlann kdtree;
    kdtree.SetGeometry(target);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < (int)source.points_.size(); i++) {
        std::vector<int> indices(1);
        std::vector<double> dists(1);
        if (kdtree.SearchKNN(source.points_[i], 1, indices, dists) == 0) {
            utility::PrintDebug(
                    "[ComputePointCloudToPointCloudDistance] Found a point "
                    "without neighbors.\n");
            distances[i] = 0.0;
        } else {
            distances[i] = std::sqrt(dists[0]);
        }
    }
    return distances;
}

std::tuple<Vec3d, Mat3d> ComputePointCloudMeanAndCovariance(
        PointCloud &input) {
    Vec3d mean{};
    Mat3d covariance{};
    covariance[0][0] = 1.0;
    covariance[1][1] = 1.0;
    covariance[2][2] = 1.0;

    if (input.IsEmpty()) return std::make_tuple(mean, covariance);

#ifdef OPEN3D_USE_CUDA
    if (0 <= input.cuda_device_id)
        return ComputePointCloudMeanAndCovarianceCUDA(input);
    else
        return ComputePointCloudMeanAndCovarianceCPU(input);
#else
    return ComputePointCloudMeanAndCovarianceCPU(input);
#endif
}

std::tuple<Vec3d, Mat3d>
ComputePointCloudMeanAndCovarianceCPU(const PointCloud &input) {
    Vec3d mean{};
    Mat3d covariance{};
    covariance[0][0] = 1.0;
    covariance[1][1] = 1.0;
    covariance[2][2] = 1.0;

    if (input.IsEmpty()) return std::make_tuple(mean, covariance);

    Mat3d cumulants{};
    for (const auto &point : input.points_) {
        cumulants[0][0] += point[0];
        cumulants[0][1] += point[1];
        cumulants[0][2] += point[2];
        cumulants[1][0] += point[0] * point[0];
        cumulants[1][1] += point[0] * point[1];
        cumulants[1][2] += point[0] * point[2];
        cumulants[2][0] += point[1] * point[1];
        cumulants[2][1] += point[1] * point[2];
        cumulants[2][2] += point[2] * point[2];
    }

    cumulants /= (double)input.points_.size();

    mean[0] = cumulants[0][0];
    mean[1] = cumulants[0][1];
    mean[2] = cumulants[0][2];

    covariance[0][0] = cumulants[1][0] - cumulants[0][0] * cumulants[0][0];
    covariance[1][1] = cumulants[2][0] - cumulants[0][1] * cumulants[0][1];
    covariance[2][2] = cumulants[2][2] - cumulants[0][2] * cumulants[0][2];
    covariance[0][1] = cumulants[1][1] - cumulants[0][0] * cumulants[0][1];
    covariance[1][0] = covariance[0][1];
    covariance[0][2] = cumulants[1][2] - cumulants[0][0] * cumulants[0][2];
    covariance[2][0] = covariance[0][2];
    covariance[1][2] = cumulants[2][1] - cumulants[0][1] * cumulants[0][2];
    covariance[2][1] = covariance[1][2];

    return std::make_tuple(mean, covariance);
}

std::vector<double> ComputePointCloudMahalanobisDistance(PointCloud &input) {
    std::vector<double> mahalanobis(input.points_.size());
    Vec3d mean;
    Mat3d covariance;
    std::tie(mean, covariance) = ComputePointCloudMeanAndCovariance(input);
    Mat3d cov_inv = covariance.inverse();
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < (int)input.points_.size(); i++) {
        Vec3d p = input.points_[i] - mean;
        mahalanobis[i] = std::sqrt(p.transpose() * cov_inv * p);
    }
    return mahalanobis;
}

std::vector<double> ComputePointCloudNearestNeighborDistance(
        const PointCloud &input) {
    std::vector<double> nn_dis(input.points_.size());
    KDTreeFlann kdtree(input);
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int i = 0; i < (int)input.points_.size(); i++) {
        std::vector<int> indices(2);
        std::vector<double> dists(2);
        if (kdtree.SearchKNN(input.points_[i], 2, indices, dists) <= 1) {
            utility::PrintDebug(
                    "[ComputePointCloudNearestNeighborDistance] Found a point "
                    "without neighbors.\n");
            nn_dis[i] = 0.0;
        } else {
            nn_dis[i] = std::sqrt(dists[1]);
        }
    }
    return nn_dis;
}

#ifdef OPEN3D_USE_CUDA

std::tuple<Vec3d, Mat3d>
ComputePointCloudMeanAndCovarianceCUDA(PointCloud &input) {
    input.UpdateDevicePoints();
    auto output = meanAndCovarianceCUDA(input.cuda_device_id, input.d_points_,
                                        input.points_.size());
    return output;
}

// update the memory assigned to d_points_
bool PointCloud::UpdateDevicePoints() {
    size_t size = points_.size() * open3d::Vec3d::Size;
    return UpdateDeviceMemory(&d_points_, (const double *const)points_.data(),
                              size, cuda_device_id);
}

// update the memory assigned to d_normals_
bool PointCloud::UpdateDeviceNormals() {
    size_t size = normals_.size() * open3d::Vec3d::Size;
    return UpdateDeviceMemory(&d_normals_, (const double *const)normals_.data(),
                              size, cuda_device_id);
}

// update the memory assigned to d_colors_
bool PointCloud::UpdateDeviceColors() {
    size_t size = colors_.size() * open3d::Vec3d::Size;
    return UpdateDeviceMemory(&d_colors_, (const double *const)colors_.data(),
                              size, cuda_device_id);
}

// perform cleanup
bool PointCloud::ReleaseDeviceMemory(double **d_data) {
    if (*d_data == NULL) return true;

    if (cudaSuccess != cudaFree(*d_data)) return false;

    *d_data = NULL;
}

// release the memory asigned to d_points_
bool PointCloud::ReleaseDevicePoints() {
    return ReleaseDeviceMemory(&d_points_);
}

// release the memory asigned to d_normals_
bool PointCloud::ReleaseDeviceNormals() {
    return ReleaseDeviceMemory(&d_normals_);
}

// release the memory asigned to d_colors_
bool PointCloud::ReleaseDeviceColors() {
    return ReleaseDeviceMemory(&d_colors_);
}

#endif  // OPEN3D_USE_CUDA

}  // namespace geometry
}  // namespace open3d
