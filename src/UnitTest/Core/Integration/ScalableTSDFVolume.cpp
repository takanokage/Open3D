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

using namespace open3d;
using namespace std;
using namespace unit_test;

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
    EXPECT_NEAR(volume_unit_length, tsdf_volume.volume_unit_length_, THRESHOLD_1E_6);
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
TEST(ScalableTSDFVolume, DISABLED_Reset)
{
    unit_test::NotImplemented();
}

// ----------------------------------------------------------------------------
//
// ----------------------------------------------------------------------------
TEST(ScalableTSDFVolume, DISABLED_Integrate)
{
    unit_test::NotImplemented();
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
