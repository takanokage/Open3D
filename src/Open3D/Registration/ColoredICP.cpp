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

#include "ColoredICP.h"

#include <Eigen/Dense>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/Geometry/KDTreeFlann.h>
#include <Open3D/Geometry/KDTreeSearchParam.h>
#include <Open3D/Utility/Eigen.h>

#include <iostream>
#include <Open3D/Utility/Console.h>

namespace open3d {

namespace {
using namespace registration;

class PointCloudForColoredICP : public geometry::PointCloud {
public:
    std::vector<Vec3d> color_gradient_;
};

class TransformationEstimationForColoredICP : public TransformationEstimation {
public:
    TransformationEstimationType GetTransformationEstimationType()
            const override {
        return type_;
    };
    TransformationEstimationForColoredICP(double lambda_geometric = 0.968)
        : lambda_geometric_(lambda_geometric) {
        if (lambda_geometric_ < 0 || lambda_geometric_ > 1.0)
            lambda_geometric_ = 0.968;
    }
    ~TransformationEstimationForColoredICP() override {}

public:
    double ComputeRMSE(const geometry::PointCloud &source,
                       const geometry::PointCloud &target,
                       const CorrespondenceSet &corres) const override;
    Mat4d ComputeTransformation(const geometry::PointCloud &source,
                                const geometry::PointCloud &target,
                                const CorrespondenceSet &corres) const override;

public:
    double lambda_geometric_;

private:
    const TransformationEstimationType type_ =
            TransformationEstimationType::ColoredICP;
};

std::shared_ptr<PointCloudForColoredICP> InitializePointCloudForColoredICP(
        const geometry::PointCloud &target,
        const geometry::KDTreeSearchParamHybrid &search_param) {
    utility::PrintDebug("InitializePointCloudForColoredICP\n");

    geometry::KDTreeFlann tree;
    tree.SetGeometry(target);

    auto output = std::make_shared<PointCloudForColoredICP>();
    output->colors_.h_data = target.colors_.h_data;
    output->normals_.h_data = target.normals_.h_data;
    output->points_.h_data = target.points_.h_data;

    size_t n_points = output->points_.size();
    output->color_gradient_.resize(n_points, Vec3d::Zero());

    for (auto k = 0; k < n_points; k++) {
        const Vec3d &vt = output->points_.h_data[k];
        const Vec3d &nt = output->normals_.h_data[k];
        double it =
                (output->colors_.h_data[k][0] + output->colors_.h_data[k][1] +
                 output->colors_.h_data[k][2]) /
                3.0;

        std::vector<int> point_idx;
        std::vector<double> point_squared_distance;

        if (tree.SearchHybrid(vt, search_param.radius_, search_param.max_nn_,
                              point_idx, point_squared_distance) >= 3) {
            // approximate image gradient of vt's tangential plane
            size_t nn = point_idx.size();
            Eigen::MatrixXd A(nn, 3);
            Eigen::MatrixXd b(nn, 1);
            A.setZero();
            b.setZero();
            for (auto i = 1; i < nn; i++) {
                int P_adj_idx = point_idx[i];
                Vec3d vt_adj = output->points_.h_data[P_adj_idx];
                Vec3d vt_proj = vt_adj - (vt_adj - vt).dot(nt) * nt;
                double it_adj = (output->colors_.h_data[P_adj_idx][0] +
                                 output->colors_.h_data[P_adj_idx][1] +
                                 output->colors_.h_data[P_adj_idx][2]) /
                                3.0;
                A(i - 1, 0) = (vt_proj[0] - vt[0]);
                A(i - 1, 1) = (vt_proj[1] - vt[1]);
                A(i - 1, 2) = (vt_proj[2] - vt[2]);
                b(i - 1, 0) = (it_adj - it);
            }
            // adds orthogonal constraint
            A(nn - 1, 0) = (nn - 1) * nt[0];
            A(nn - 1, 1) = (nn - 1) * nt[1];
            A(nn - 1, 2) = (nn - 1) * nt[2];
            b(nn - 1, 0) = 0;
            // solving linear equation
            bool is_success;
            Eigen::MatrixXd x;
            std::tie(is_success, x) = utility::SolveLinearSystemPSD(
                    A.transpose() * A, A.transpose() * b);
            if (is_success) {
                output->color_gradient_[k] = x;
            }
        }
    }
    return output;
}

Mat4d TransformationEstimationForColoredICP::ComputeTransformation(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    if (corres.empty() || target.HasNormals() == false ||
        target.HasColors() == false || source.HasColors() == false)
        return Mat4d::Identity();

    double sqrt_lambda_geometric = sqrt(lambda_geometric_);
    double lambda_photometric = 1.0 - lambda_geometric_;
    double sqrt_lambda_photometric = sqrt(lambda_photometric);

    const auto &target_c = (const PointCloudForColoredICP &)target;

    auto compute_jacobian_and_residual = [&](int i, std::vector<Vec6d> &J_r,
                                             std::vector<double> &r) {
        size_t cs = corres[i][0];
        size_t ct = corres[i][1];
        const Vec3d &vs = source.points_.h_data[cs];
        const Vec3d &vt = target.points_.h_data[ct];
        const Vec3d &nt = target.normals_.h_data[ct];

        J_r.resize(2);
        r.resize(2);

        J_r[0].block<3, 1>(0, 0) = sqrt_lambda_geometric * vs.cross(nt);
        J_r[0].block<3, 1>(3, 0) = sqrt_lambda_geometric * nt;
        r[0] = sqrt_lambda_geometric * (vs - vt).dot(nt);

        // project vs into vt's tangential plane
        Vec3d vs_proj = vs - (vs - vt).dot(nt) * nt;
        double is =
                (source.colors_.h_data[cs][0] + source.colors_.h_data[cs][1] +
                 source.colors_.h_data[cs][2]) /
                3.0;
        double it =
                (target.colors_.h_data[ct][0] + target.colors_.h_data[ct][1] +
                 target.colors_.h_data[ct][2]) /
                3.0;
        const Vec3d &dit = target_c.color_gradient_[ct];
        double is0_proj = (dit.dot(vs_proj - vt)) + it;

        const Mat3d M =
                (Mat3d() << 1.0 - nt[0] * nt[0], -nt[0] * nt[1], -nt[0] * nt[2],
                 -nt[0] * nt[1], 1.0 - nt[1] * nt[1], -nt[1] * nt[2],
                 -nt[0] * nt[2], -nt[1] * nt[2], 1.0 - nt[2] * nt[2])
                        .finished();

        const Vec3d &ditM = -dit.transpose() * M;
        J_r[1].block<3, 1>(0, 0) = sqrt_lambda_photometric * vs.cross(ditM);
        J_r[1].block<3, 1>(3, 0) = sqrt_lambda_photometric * ditM;
        r[1] = sqrt_lambda_photometric * (is - is0_proj);
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

double TransformationEstimationForColoredICP::ComputeRMSE(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &corres) const {
    double sqrt_lambda_geometric = sqrt(lambda_geometric_);
    double lambda_photometric = 1.0 - lambda_geometric_;
    double sqrt_lambda_photometric = sqrt(lambda_photometric);
    const auto &target_c = (const PointCloudForColoredICP &)target;

    double residual = 0.0;
    for (size_t i = 0; i < corres.size(); i++) {
        size_t cs = corres[i][0];
        size_t ct = corres[i][1];
        const Vec3d &vs = source.points_.h_data[cs];
        const Vec3d &vt = target.points_.h_data[ct];
        const Vec3d &nt = target.normals_.h_data[ct];
        Vec3d vs_proj = vs - (vs - vt).dot(nt) * nt;
        double is =
                (source.colors_.h_data[cs][0] + source.colors_.h_data[cs][1] +
                 source.colors_.h_data[cs][2]) /
                3.0;
        double it =
                (target.colors_.h_data[ct][0] + target.colors_.h_data[ct][1] +
                 target.colors_.h_data[ct][2]) /
                3.0;
        const Vec3d &dit = target_c.color_gradient_[ct];
        double is0_proj = (dit.dot(vs_proj - vt)) + it;
        double residual_geometric = sqrt_lambda_geometric * (vs - vt).dot(nt);
        double residual_photometric = sqrt_lambda_photometric * (is - is0_proj);
        residual += residual_geometric * residual_geometric +
                    residual_photometric * residual_photometric;
    }
    return residual;
};

}  // unnamed namespace

namespace registration {

RegistrationResult RegistrationColoredICP(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        double max_distance,
        const Mat4d &init /* = Mat4d::Identity()*/,
        const ICPConvergenceCriteria &criteria /* = ICPConvergenceCriteria()*/,
        double lambda_geometric /* = 0.968*/) {
    auto target_c = InitializePointCloudForColoredICP(
            target, geometry::KDTreeSearchParamHybrid(max_distance * 2.0, 30));
    return RegistrationICP(
            source, *target_c, max_distance, init,
            TransformationEstimationForColoredICP(lambda_geometric), criteria);
}

}  // namespace registration
}  // namespace open3d
