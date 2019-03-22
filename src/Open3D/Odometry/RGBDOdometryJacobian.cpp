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

#include "Odometry.h"

#include <Open3D/Geometry/Image.h>
#include <Open3D/Geometry/RGBDImage.h>
#include <Open3D/Odometry/RGBDOdometryJacobian.h>

namespace open3d {

namespace {

const double SOBEL_SCALE = 0.125;
const double LAMBDA_HYBRID_DEPTH = 0.968;

}  // unnamed namespace

namespace odometry {
void RGBDOdometryJacobianFromColorTerm::ComputeJacobianAndResidual(
        int row,
        std::vector<Vec6d> &J_r,
        std::vector<double> &r,
        const geometry::RGBDImage &source,
        const geometry::RGBDImage &target,
        const geometry::Image &source_xyz,
        const geometry::RGBDImage &target_dx,
        const geometry::RGBDImage &target_dy,
        const Mat3d &intrinsic,
        const Mat4d &extrinsic,
        const CorrespondenceSetPixelWise &corresps) const {
    Mat3d R = extrinsic.block<3, 3>(0, 0);
    Vec3d t = extrinsic.block<3, 1>(0, 3);

    int u_s = corresps[row][0];
    int v_s = corresps[row][1];
    int u_t = corresps[row][2];
    int v_t = corresps[row][3];
    double diff = *geometry::PointerAt<float>(target.color_, u_t, v_t) -
                  *geometry::PointerAt<float>(source.color_, u_s, v_s);
    double dIdx = SOBEL_SCALE *
                  (*geometry::PointerAt<float>(target_dx.color_, u_t, v_t));
    double dIdy = SOBEL_SCALE *
                  (*geometry::PointerAt<float>(target_dy.color_, u_t, v_t));
    Vec3d p3d_mat{*geometry::PointerAt<float>(source_xyz, u_s, v_s, 0),
                  *geometry::PointerAt<float>(source_xyz, u_s, v_s, 1),
                  *geometry::PointerAt<float>(source_xyz, u_s, v_s, 2)};
    Vec3d p3d_trans = R * p3d_mat + t;
    double invz = 1. / p3d_trans[2];
    double c0 = dIdx * intrinsic[0][0] * invz;
    double c1 = dIdy * intrinsic[1][1] * invz;
    double c2 = -(c0 * p3d_trans[0] + c1 * p3d_trans[1]) * invz;

    J_r.resize(1);
    J_r[0][0] = -p3d_trans[2] * c1 + p3d_trans[1] * c2;
    J_r[0][1] = p3d_trans[2] * c0 - p3d_trans[0] * c2;
    J_r[0][2] = -p3d_trans[1] * c0 + p3d_trans[0] * c1;
    J_r[0][3] = c0;
    J_r[0][4] = c1;
    J_r[0][5] = c2;
    r.resize(1);
    r[0] = diff;
}

void RGBDOdometryJacobianFromHybridTerm::ComputeJacobianAndResidual(
        int row,
        std::vector<Vec6d> &J_r,
        std::vector<double> &r,
        const geometry::RGBDImage &source,
        const geometry::RGBDImage &target,
        const geometry::Image &source_xyz,
        const geometry::RGBDImage &target_dx,
        const geometry::RGBDImage &target_dy,
        const Mat3d &intrinsic,
        const Mat4d &extrinsic,
        const CorrespondenceSetPixelWise &corresps) const {
    double sqrt_lamba_dep, sqrt_lambda_img;
    sqrt_lamba_dep = sqrt(LAMBDA_HYBRID_DEPTH);
    sqrt_lambda_img = sqrt(1.0 - LAMBDA_HYBRID_DEPTH);

    const double fx = intrinsic[0][0];
    const double fy = intrinsic[1][1];
    Mat3d R = extrinsic.block<3, 3>(0, 0);
    Vec3d t = extrinsic.block<3, 1>(0, 3);

    int u_s = corresps[row][0];
    int v_s = corresps[row][1];
    int u_t = corresps[row][2];
    int v_t = corresps[row][3];
    double diff_photo = (*geometry::PointerAt<float>(target.color_, u_t, v_t) -
                         *geometry::PointerAt<float>(source.color_, u_s, v_s));
    double dIdx = SOBEL_SCALE *
                  (*geometry::PointerAt<float>(target_dx.color_, u_t, v_t));
    double dIdy = SOBEL_SCALE *
                  (*geometry::PointerAt<float>(target_dy.color_, u_t, v_t));
    double dDdx = SOBEL_SCALE *
                  (*geometry::PointerAt<float>(target_dx.depth_, u_t, v_t));
    double dDdy = SOBEL_SCALE *
                  (*geometry::PointerAt<float>(target_dy.depth_, u_t, v_t));
    if (std::isnan(dDdx)) dDdx = 0;
    if (std::isnan(dDdy)) dDdy = 0;
    Vec3d p3d_mat{*geometry::PointerAt<float>(source_xyz, u_s, v_s, 0),
                  *geometry::PointerAt<float>(source_xyz, u_s, v_s, 1),
                  *geometry::PointerAt<float>(source_xyz, u_s, v_s, 2)};
    Vec3d p3d_trans = R * p3d_mat + t;

    double diff_geo =
            *geometry::PointerAt<float>(target.depth_, u_t, v_t) - p3d_trans[2];
    double invz = 1. / p3d_trans[2];
    double c0 = dIdx * fx * invz;
    double c1 = dIdy * fy * invz;
    double c2 = -(c0 * p3d_trans[0] + c1 * p3d_trans[1]) * invz;
    double d0 = dDdx * fx * invz;
    double d1 = dDdy * fy * invz;
    double d2 = -(d0 * p3d_trans[0] + d1 * p3d_trans[1]) * invz;

    J_r.resize(2);
    r.resize(2);
    J_r[0][0] = sqrt_lambda_img * (-p3d_trans[2] * c1 + p3d_trans[1] * c2);
    J_r[0][1] = sqrt_lambda_img * (p3d_trans[2] * c0 - p3d_trans[0] * c2);
    J_r[0][2] = sqrt_lambda_img * (-p3d_trans[1] * c0 + p3d_trans[0] * c1);
    J_r[0][3] = sqrt_lambda_img * (c0);
    J_r[0][4] = sqrt_lambda_img * (c1);
    J_r[0][5] = sqrt_lambda_img * (c2);
    double r_photo = sqrt_lambda_img * diff_photo;
    r[0] = r_photo;

    J_r[1][0] = sqrt_lamba_dep *
                ((-p3d_trans[2] * d1 + p3d_trans[1] * d2) - p3d_trans[1]);
    J_r[1][1] = sqrt_lamba_dep *
                ((p3d_trans[2] * d0 - p3d_trans[0] * d2) + p3d_trans[0]);
    J_r[1][2] = sqrt_lamba_dep * ((-p3d_trans[1] * d0 + p3d_trans[0] * d1));
    J_r[1][3] = sqrt_lamba_dep * (d0);
    J_r[1][4] = sqrt_lamba_dep * (d1);
    J_r[1][5] = sqrt_lamba_dep * (d2 - 1.0f);
    double r_geo = sqrt_lamba_dep * diff_geo;
    r[1] = r_geo;
}

}  // namespace odometry
}  // namespace open3d
