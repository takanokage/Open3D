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

#include "TransformationEstimation.h"

#include <Eigen/Geometry>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/Utility/Eigen.h>

namespace open3d {
namespace registration {

double TransformationEstimationPointToPoint::ComputeRMSE(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty()) return 0.0;
    double err = 0.0;
    for (const auto &c : corres) {
        err += (source.points_.h_data[c[0]] - target.points_.h_data[c[1]])
                       .squaredNorm();
    }
    return std::sqrt(err / (double)corres.size());
}

Mat4d TransformationEstimationPointToPoint::ComputeTransformation(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty()) return Mat4d::Identity();
    Eigen::MatrixXd source_mat(3, corres.size());
    Eigen::MatrixXd target_mat(3, corres.size());
    for (size_t i = 0; i < corres.size(); i++) {
        source_mat.block<3, 1>(0, i) = source.points_.h_data[corres[i][0]];
        target_mat.block<3, 1>(0, i) = target.points_.h_data[corres[i][1]];
    }
    return Eigen::umeyama(source_mat, target_mat, with_scaling_);
}

double TransformationEstimationPointToPlane::ComputeRMSE(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || target.HasNormals() == false) return 0.0;
    double err = 0.0, r;
    for (const auto &c : corres) {
        r = (source.points_.h_data[c[0]] - target.points_.h_data[c[1]])
                    .dot(target.normals_.h_data[c[1]]);
        err += r * r;
    }
    return std::sqrt(err / (double)corres.size());
}

Mat4d TransformationEstimationPointToPlane::ComputeTransformation(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || target.HasNormals() == false)
        return Mat4d::Identity();

    auto compute_jacobian_and_residual = [&](int i, Vec6d &J_r, double &r) {
        const Vec3d &vs = source.points_.h_data[corres[i][0]];
        const Vec3d &vt = target.points_.h_data[corres[i][1]];
        const Vec3d &nt = target.normals_.h_data[corres[i][1]];
        r = (vs - vt).dot(nt);
        J_r.block<3, 1>(0, 0) = vs.cross(nt);
        J_r.block<3, 1>(3, 0) = nt;
    };

    Mat6d JTJ;
    Vec6d JTr;
    double r2;
    std::tie(JTJ, JTr, r2) = utility::ComputeJTJandJTr<Mat6d, Vec6d>(
            compute_jacobian_and_residual, (int)corres.size());

    bool is_success;
    Mat4d extrinsic;
    std::tie(is_success, extrinsic) =
            utility::SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);

    return is_success ? extrinsic : Mat4d::Identity();
}

}  // namespace registration
}  // namespace open3d
