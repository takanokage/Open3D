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

#include "VoxelGrid.h"

namespace open3d {
namespace geometry {

void VoxelGrid::Clear() {
    voxel_size_ = 0.0;
    origin_ = Vec3d::Zero();
    voxels_.h_data.clear();
    colors_.h_data.clear();
}

bool VoxelGrid::IsEmpty() const { return !HasVoxels(); }

Vec3d VoxelGrid::GetMinBound() const {
    if (!HasVoxels()) {
        return Vec3d{};
    }
    auto itr_x = std::min_element(
            voxels_.h_data.begin(), voxels_.h_data.end(),
            [](const Vec3i &a, const Vec3i &b) { return a[0] < b[0]; });
    auto itr_y = std::min_element(
            voxels_.h_data.begin(), voxels_.h_data.end(),
            [](const Vec3i &a, const Vec3i &b) { return a[1] < b[1]; });
    auto itr_z = std::min_element(
            voxels_.h_data.begin(), voxels_.h_data.end(),
            [](const Vec3i &a, const Vec3i &b) { return a[2] < b[2]; });
    return Vec3d{(*itr_x)[0] * voxel_size_ + origin_[0],
                 (*itr_y)[1] * voxel_size_ + origin_[1],
                 (*itr_z)[2] * voxel_size_ + origin_[2]};
}

Vec3d VoxelGrid::GetMaxBound() const {
    if (!HasVoxels()) {
        return Vec3d{};
    }
    auto itr_x = std::max_element(
            voxels_.h_data.begin(), voxels_.h_data.end(),
            [](const Vec3i &a, const Vec3i &b) { return a[0] < b[0]; });
    auto itr_y = std::max_element(
            voxels_.h_data.begin(), voxels_.h_data.end(),
            [](const Vec3i &a, const Vec3i &b) { return a[1] < b[1]; });
    auto itr_z = std::max_element(
            voxels_.h_data.begin(), voxels_.h_data.end(),
            [](const Vec3i &a, const Vec3i &b) { return a[2] < b[2]; });
    return Vec3d{(*itr_x)[0] * voxel_size_ + origin_[0],
                 (*itr_y)[1] * voxel_size_ + origin_[1],
                 (*itr_z)[2] * voxel_size_ + origin_[2]};
}

void VoxelGrid::Transform(const Mat4d &transformation) {
    // not implemented.
}

VoxelGrid &VoxelGrid::operator+=(const VoxelGrid &voxelgrid) {
    // not implemented.
    return (*this);
}

VoxelGrid VoxelGrid::operator+(const VoxelGrid &voxelgrid) const {
    return (VoxelGrid(*this) += voxelgrid);
}

}  // namespace geometry
}  // namespace open3d
