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

#include "UnitTest.h"

#include "Core/Integration/ScalableTSDFVolume.h"
#include <Core/Geometry/Image.h>
#include <Core/Geometry/RGBDImage.h>
#include "Core/Camera/PinholeCameraIntrinsic.h"

using namespace open3d;
using namespace std;
using namespace unit_test;

using VolumeUnit = ScalableTSDFVolume::VolumeUnit;

// ----------------------------------------------------------------------------
// Generate a color image based on TSDFVolumeColorType.
// ----------------------------------------------------------------------------
Image ColorImage(const int& width,
                 const int& height,
                 const int& seed,
                 const TSDFVolumeColorType& color_type)
{
    int num_of_channels   = 1;
    int bytes_per_channel = 1;

    switch (color_type)
    {
        case TSDFVolumeColorType::RGB8:
        {
            num_of_channels   = 3;
            bytes_per_channel = 1;

            break;
        }
        case TSDFVolumeColorType::Gray32:
        {
            num_of_channels   = 1;
            bytes_per_channel = 4;

            break;
        }
        default: break;
    }

    Image image;

    image.PrepareImage(width,
                       height,
                       num_of_channels,
                       bytes_per_channel);

    if (bytes_per_channel == 4)
    {
        float* const depth_data = reinterpret_cast<float*>(&image.data_[0]);
        Rand(depth_data, width * height, 1.0, 1.0, seed);
    }
    else
        Rand(image.data_, 0, 255, seed);

    return image;
}

// ----------------------------------------------------------------------------
// Generate a depth image.
// ----------------------------------------------------------------------------
Image DepthImage(const int& width, const int& height, const int& seed)
{
    int num_of_channels = 1;
    int bytes_per_channel = 4;

    Image image;

    image.PrepareImage(width,
                       height,
                       num_of_channels,
                       bytes_per_channel);

    float* const depth_data = reinterpret_cast<float*>(&image.data_[0]);
    Rand(depth_data, width * height, 0.0, 1.0, seed);

    return image;
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, Constructor)
{
    TSDFVolumeColorType color_type = TSDFVolumeColorType::Gray32;
    int depth_sampling_stride = 4;
    int volume_unit_length = 12;
    int volume_unit_resolution = 16;
    double voxel_length = 0.75;
    double sdf_trunc = 0.75;

    ScalableTSDFVolume tsdf_volume(voxel_length,
                                   sdf_trunc,
                                   color_type,
                                   volume_unit_resolution,
                                   depth_sampling_stride);

    EXPECT_EQ(color_type, tsdf_volume.color_type_);
    EXPECT_EQ(depth_sampling_stride, tsdf_volume.depth_sampling_stride_);
    EXPECT_NEAR(volume_unit_length, tsdf_volume.volume_unit_length_,
                THRESHOLD_1E_6);
    EXPECT_EQ(volume_unit_resolution, tsdf_volume.volume_unit_resolution_);
    EXPECT_NEAR(voxel_length, tsdf_volume.voxel_length_, THRESHOLD_1E_6);
    EXPECT_NEAR(sdf_trunc, tsdf_volume.sdf_trunc_, THRESHOLD_1E_6);

    EXPECT_EQ(0, tsdf_volume.volume_units_.size());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, DISABLED_MemberData)
{
    unit_test::NotImplemented();
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, Reset)
{
    TSDFVolumeColorType color_type = TSDFVolumeColorType::Gray32;
    int depth_sampling_stride = 4;
    int volume_unit_resolution = 16;
    double voxel_length = 0.75;
    double sdf_trunc = 0.75;

    ScalableTSDFVolume tsdf_volume(voxel_length,
                                   sdf_trunc,
                                   color_type,
                                   volume_unit_resolution,
                                   depth_sampling_stride);

    EXPECT_EQ(0, tsdf_volume.volume_units_.size());

    Eigen::Vector3i key = { 0, 0, 0 };
    VolumeUnit volume_unit;

    pair<Eigen::Vector3i, VolumeUnit> entry(key, volume_unit);

    tsdf_volume.volume_units_.insert(entry);

    EXPECT_EQ(1, tsdf_volume.volume_units_.size());

    tsdf_volume.Reset();

    EXPECT_EQ(0, tsdf_volume.volume_units_.size());
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, Integrate)
{
    TSDFVolumeColorType color_type = TSDFVolumeColorType::Gray32;
    int depth_sampling_stride = 4;
    int volume_unit_resolution = 16;
    double voxel_length = 0.75;
    double sdf_trunc = 0.75;

    ScalableTSDFVolume tsdf_volume(voxel_length,
                                   sdf_trunc,
                                   color_type,
                                   volume_unit_resolution,
                                   depth_sampling_stride);

    int width = 240;
    int height = 180;

    Image color = ColorImage(width, height, 0, color_type);
    Image depth = DepthImage(width, height, 1);
    RGBDImage rgbd_image(color, depth);

    double fx = 0.5;
    double fy = 0.65;
    double cx = 0.75;
    double cy = 0.35;
    open3d::PinholeCameraIntrinsic intrinsic(width, height, fx, fy, cx, cy);

    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Zero();
    extrinsic(0, 0) = 1.0;
    extrinsic(1, 1) = 1.0;
    extrinsic(2, 2) = 1.0;
    extrinsic(0, 3) = 1.0;

    tsdf_volume.Integrate(rgbd_image, intrinsic, extrinsic);

    EXPECT_EQ(color_type, tsdf_volume.color_type_);
    EXPECT_EQ(depth_sampling_stride, tsdf_volume.depth_sampling_stride_);
    EXPECT_EQ(volume_unit_resolution, tsdf_volume.volume_unit_resolution_);
    EXPECT_NEAR(voxel_length, tsdf_volume.voxel_length_, THRESHOLD_1E_6);
    EXPECT_NEAR(sdf_trunc, tsdf_volume.sdf_trunc_, THRESHOLD_1E_6);

    EXPECT_EQ(1, tsdf_volume.volume_units_.size());

    for (auto entry: tsdf_volume.volume_units_)
    {
        cout << entry.first << endl << endl;
        // cout << entry.second << endl << endl;
    }
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, DISABLED_ExtractPointCloud)
{
    unit_test::NotImplemented();
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, DISABLED_ExtractTriangleMesh)
{
    unit_test::NotImplemented();
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, DISABLED_ExtractVoxelPointCloud)
{
    unit_test::NotImplemented();
}
