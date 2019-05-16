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

#include "TestUtility/UnitTest.h"

#include "Open3D/Types/Blob.h"

#include <Eigen/Geometry>

#include <iostream>
using namespace std;
using namespace unit_test;

// ----------------------------------------------------------------------------
// Not a POD.
// ----------------------------------------------------------------------------
TEST(BlobType, is_POD) {
    EXPECT_FALSE(is_pod<open3d::Blob2i>());
    EXPECT_FALSE(is_pod<open3d::Blob3i>());
    EXPECT_FALSE(is_pod<open3d::Blob3d>());
}

// ----------------------------------------------------------------------------
// Default constructor.
// ----------------------------------------------------------------------------
TEST(BlobType, Default_constructor) {
    open3d::Blob3d b3d;

    EXPECT_EQ(b3d.num_elements, 0);
    EXPECT_EQ(b3d.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b3d.h_data.size(), 0);
    EXPECT_TRUE(NULL == b3d.d_data);
    EXPECT_EQ(b3d.size(), 0);
}

// ----------------------------------------------------------------------------
// Copy constructor - CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Copy_constructor_CPU) {
    size_t num_elements = 100;
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    open3d::Blob3d b0(num_elements, device_id);
    Rand((double* const)b0.h_data.data(), b0.size() * 3, 0.0, 10.0, 0);

    open3d::Blob3d b1(b0);

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_EQ(b1.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b1.h_data.size(), num_elements);
    EXPECT_FALSE(b1.h_data.empty());
    EXPECT_TRUE(NULL == b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    ExpectEQ(b0.h_data, b1.h_data);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Copy constructor - GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Copy_constructor_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::GPU_00;

    open3d::Blob3d b0(num_elements, device_id);

    // random initialization of b0
    vector<Eigen::Vector3d> b0_d_data(num_elements);
    Rand((double* const)b0_d_data.data(), num_doubles, 0.0, 10.0, 0);
    open3d::cuda::CopyHst2DevMemory((const double* const)b0_d_data.data(),
                                    b0.d_data, num_doubles);

    open3d::Blob3d b1(b0);

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_EQ(b1.device_id, open3d::DeviceID::GPU_00);
    EXPECT_EQ(b1.h_data.size(), 0);
    EXPECT_TRUE(b1.h_data.empty());
    EXPECT_TRUE(NULL != b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    vector<Eigen::Vector3d> b1_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b1.d_data, (double* const)b1_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0_d_data, b1_d_data);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Copy constructor - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Copy_constructor_CPU_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    open3d::Blob3d b0(num_elements, device_id);
    Rand((double* const)b0.h_data.data(), num_doubles, 0.0, 10.0, 0);
    open3d::cuda::CopyHst2DevMemory((const double* const)b0.h_data.data(),
                                    b0.d_data, num_doubles);

    open3d::Blob3d b1(b0);

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_TRUE(open3d::DeviceID::CPU & b1.device_id);
    EXPECT_TRUE(open3d::DeviceID::GPU_00 & b1.device_id);
    EXPECT_EQ(b1.h_data.size(), num_elements);
    EXPECT_FALSE(b1.h_data.empty());
    EXPECT_TRUE(NULL != b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    ExpectEQ(b0.h_data, b1.h_data);

    vector<Eigen::Vector3d> b1_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b1.d_data, (double* const)b1_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0.h_data, b1_d_data);
}
#endif

// ----------------------------------------------------------------------------
// Move constructor - CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Move_constructor_CPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    vector<Eigen::Vector3d> points(num_elements);
    Rand((double* const)points.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(points, device_id);
    open3d::Blob3d b1(move(b0));

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_EQ(b1.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b1.h_data.size(), num_elements);
    EXPECT_FALSE(b1.h_data.empty());
    EXPECT_TRUE(NULL == b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);

    ExpectEQ(points, b1.h_data);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Move constructor - GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Move_constructor_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::GPU_00;

    vector<Eigen::Vector3d> points(num_elements);
    Rand((double* const)points.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(points, device_id);
    open3d::Blob3d b1(std::move(b0));

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_EQ(b1.device_id, open3d::DeviceID::GPU_00);
    EXPECT_EQ(b1.h_data.size(), 0);
    EXPECT_TRUE(b1.h_data.empty());
    EXPECT_TRUE(NULL != b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);

    vector<Eigen::Vector3d> b1_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b1.d_data, (double* const)b1_d_data.data(),
                                    num_doubles);

    ExpectEQ(points, b1_d_data);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Move constructor - CPU & GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Move_constructor_CPU_GPU) {
    size_t num_elements = 1;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    vector<Eigen::Vector3d> points(num_elements);
    Rand((double* const)points.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(points, device_id);
    open3d::Blob3d b1(std::move(b0));

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_EQ(b1.device_id, device_id);
    EXPECT_EQ(b1.h_data.size(), num_elements);
    EXPECT_FALSE(b1.h_data.empty());
    EXPECT_TRUE(NULL != b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);

    vector<Eigen::Vector3d> b1_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b1.d_data, (double* const)b1_d_data.data(),
                                    num_doubles);

    ExpectEQ(points, b1.h_data);
    ExpectEQ(points, b1_d_data);
}
#endif

// ----------------------------------------------------------------------------
// Initialization constructor - CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Initialization_constructor_CPU) {
    size_t num_elements = 100;
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    open3d::Blob3d b3d(num_elements, device_id);

    EXPECT_EQ(b3d.num_elements, num_elements);
    EXPECT_EQ(b3d.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b3d.h_data.size(), num_elements);
    EXPECT_FALSE(b3d.h_data.empty());
    EXPECT_TRUE(NULL == b3d.d_data);
    EXPECT_EQ(b3d.size(), num_elements);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Initialization constructor - GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Initialization_constructor_GPU) {
    size_t num_elements = 100;
    open3d::DeviceID::Type device_id = open3d::DeviceID::GPU_00;

    open3d::Blob3d b3d(num_elements, device_id);

    EXPECT_EQ(b3d.num_elements, num_elements);
    EXPECT_EQ(b3d.device_id, open3d::DeviceID::GPU_00);
    EXPECT_EQ(b3d.h_data.size(), 0);
    EXPECT_TRUE(b3d.h_data.empty());
    EXPECT_TRUE(NULL != b3d.d_data);
    EXPECT_EQ(b3d.size(), num_elements);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Initialization constructor - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Initialization_constructor_CPU_GPU) {
    size_t num_elements = 100;
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    open3d::Blob3d b3d(num_elements, device_id);

    EXPECT_EQ(b3d.num_elements, num_elements);
    EXPECT_TRUE(open3d::DeviceID::CPU & b3d.device_id);
    EXPECT_TRUE(open3d::DeviceID::GPU_00 & b3d.device_id);
    EXPECT_EQ(b3d.h_data.size(), num_elements);
    EXPECT_FALSE(b3d.h_data.empty());
    EXPECT_TRUE(NULL != b3d.d_data);
    EXPECT_EQ(b3d.size(), num_elements);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Initialize - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Initialize) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    open3d::Blob3d b0(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_TRUE(b0.device_id == device_id);
    EXPECT_EQ(b0.h_data.size(), num_elements);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);

    const Eigen::Vector3d* const b0_h_data = b0.h_data.data();
    const double* const b0_d_data = b0.d_data;

    // already initialized, shouldn't change anything
    b0.Initialize();

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_TRUE(b0.device_id == device_id);
    EXPECT_EQ(b0.h_data.size(), num_elements);
    EXPECT_EQ(b0.h_data.data(), b0_h_data);
    EXPECT_EQ(b0.d_data, b0_d_data);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Initialize - with args - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Initialize_args) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    open3d::Blob3d b0;

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_TRUE(open3d::DeviceID::CPU & b0.device_id);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);

    b0.Initialize(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_TRUE(b0.device_id == device_id);
    EXPECT_EQ(b0.h_data.size(), num_elements);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);
}
#endif

// ----------------------------------------------------------------------------
// Reset - CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Reset_CPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    open3d::Blob3d b0(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), num_elements);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);

    b0.Reset();

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Reset - GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Reset_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::GPU_00;

    open3d::Blob3d b0(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::GPU_00);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);

    b0.Reset();

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Reset - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Reset_CPU_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    open3d::Blob3d b0(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_TRUE(open3d::DeviceID::CPU & b0.device_id);
    EXPECT_TRUE(open3d::DeviceID::GPU_00 & b0.device_id);
    EXPECT_EQ(b0.h_data.size(), num_elements);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);

    b0.Reset();

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);
}
#endif

// ----------------------------------------------------------------------------
// Subscript operator - readonly - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Subscript_operator_ro_CPU) {
    size_t num_elements = 100;
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    open3d::Blob3d b0(num_elements, device_id);
    Rand((double* const)b0.h_data.data(), b0.size() * 3, 0.0, 10.0, 0);

    for (size_t i = 0; i < num_elements; i++) ExpectEQ(b0[i], b0.h_data[i]);
}

// ----------------------------------------------------------------------------
// Subscript operator - readwrite - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Subscript_operator_rw_CPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    vector<Eigen::Vector3d> points(num_elements);
    Rand((double* const)points.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(num_elements, device_id);

    for (size_t i = 0; i < num_elements; i++) b0[i] = b0.h_data[i];

    for (size_t i = 0; i < num_elements; i++) ExpectEQ(b0[i], b0.h_data[i]);
}

// ----------------------------------------------------------------------------
// Equality operator - CPU vs. CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Equality_operator_CPU_CPU) {
    size_t num_elements = 100;

    Eigen::Vector3d vmin(-1.0, -1.0, -1.0);
    Eigen::Vector3d vmax(+1.0, +1.0, +1.0);

    vector<Eigen::Vector3d> points(num_elements);
    Rand(points, vmin, vmax, 0);

    open3d::Blob3d b0(points, open3d::DeviceID::CPU);
    open3d::Blob3d b1(points, open3d::DeviceID::CPU);

    EXPECT_TRUE(b1 == b0);

    b1.push_back(Eigen::Vector3d::Zero());

    EXPECT_TRUE(b1 != b0);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Equality operator - CPU vs. GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Equality_operator_CPU_GPU) {
    size_t num_elements = 100;

    Eigen::Vector3d vmin(-1.0, -1.0, -1.0);
    Eigen::Vector3d vmax(+1.0, +1.0, +1.0);

    vector<Eigen::Vector3d> points(num_elements);
    Rand(points, vmin, vmax, 0);

    open3d::Blob3d b0(points, open3d::DeviceID::CPU);
    open3d::Blob3d b1(points, open3d::DeviceID::GPU_00);

    EXPECT_TRUE(b1 == b0);

    b1.push_back(Eigen::Vector3d::Zero());

    EXPECT_TRUE(b1 != b0);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Equality operator - GPU vs. CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Equality_operator_GPU_CPU) {
    size_t num_elements = 100;

    Eigen::Vector3d vmin(-1.0, -1.0, -1.0);
    Eigen::Vector3d vmax(+1.0, +1.0, +1.0);

    vector<Eigen::Vector3d> points(num_elements);
    Rand(points, vmin, vmax, 0);

    open3d::Blob3d b0(points, open3d::DeviceID::GPU_00);
    open3d::Blob3d b1(points, open3d::DeviceID::CPU);

    EXPECT_TRUE(b1 == b0);

    b1.push_back(Eigen::Vector3d::Zero());

    EXPECT_TRUE(b1 != b0);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Equality operator - GPU vs. GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Equality_operator_GPU_GPU) {
    size_t num_elements = 100;

    Eigen::Vector3d vmin(-1.0, -1.0, -1.0);
    Eigen::Vector3d vmax(+1.0, +1.0, +1.0);

    vector<Eigen::Vector3d> points(num_elements);
    Rand(points, vmin, vmax, 0);

    open3d::Blob3d b0(points, open3d::DeviceID::GPU_00);
    open3d::Blob3d b1(points, open3d::DeviceID::GPU_00);

    EXPECT_TRUE(b1 == b0);

    b1.push_back(Eigen::Vector3d::Zero());

    EXPECT_TRUE(b1 != b0);
}
#endif

// ----------------------------------------------------------------------------
// Assignment operator - CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_CPU) {
    size_t num_elements = 100;
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    open3d::Blob3d b0(num_elements, device_id);
    Rand((double* const)b0.h_data.data(), b0.size() * 3, 0.0, 10.0, 0);

    open3d::Blob3d b1 = b0;

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_EQ(b1.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b1.h_data.size(), num_elements);
    EXPECT_FALSE(b1.h_data.empty());
    EXPECT_TRUE(NULL == b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    ExpectEQ(b0.h_data, b1.h_data);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Assignment operator - GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::GPU_00;

    open3d::Blob3d b0(num_elements, device_id);

    // random initialization of b0
    vector<Eigen::Vector3d> b0_d_data(num_elements);
    Rand((double* const)b0_d_data.data(), num_doubles, 0.0, 10.0, 0);
    open3d::cuda::CopyHst2DevMemory((const double* const)b0_d_data.data(),
                                    b0.d_data, num_doubles);

    open3d::Blob3d b1 = b0;

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_EQ(b1.device_id, open3d::DeviceID::GPU_00);
    EXPECT_EQ(b1.h_data.size(), 0);
    EXPECT_TRUE(b1.h_data.empty());
    EXPECT_TRUE(NULL != b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    vector<Eigen::Vector3d> b1_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b1.d_data, (double* const)b1_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0_d_data, b1_d_data);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Assignment operator - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_CPU_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    open3d::Blob3d b0(num_elements, device_id);
    Rand((double* const)b0.h_data.data(), num_doubles, 0.0, 10.0, 0);
    open3d::cuda::CopyHst2DevMemory((const double* const)b0.h_data.data(),
                                    b0.d_data, num_doubles);

    open3d::Blob3d b1 = b0;

    EXPECT_EQ(b1.num_elements, num_elements);
    EXPECT_TRUE(open3d::DeviceID::CPU & b1.device_id);
    EXPECT_TRUE(open3d::DeviceID::GPU_00 & b1.device_id);
    EXPECT_EQ(b1.h_data.size(), num_elements);
    EXPECT_FALSE(b1.h_data.empty());
    EXPECT_TRUE(NULL != b1.d_data);
    EXPECT_EQ(b1.size(), num_elements);

    ExpectEQ(b0.h_data, b1.h_data);

    vector<Eigen::Vector3d> b1_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b1.d_data, (double* const)b1_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0.h_data, b1_d_data);
}
#endif

// ----------------------------------------------------------------------------
// Assignment operator - from vector - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_vector_CPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    vector<Eigen::Vector3d> v(num_elements);
    Rand((double* const)v.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(num_elements, device_id);
    b0 = v;

    ExpectEQ(b0.h_data, v);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Assignment operator - from vector - GPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_vector_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::GPU_00;

    vector<Eigen::Vector3d> v(num_elements);
    Rand((double* const)v.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(num_elements, device_id);
    b0 = v;

    vector<Eigen::Vector3d> b0_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0_d_data, v);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Assignment operator - from vector - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_vector_CPU_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    vector<Eigen::Vector3d> v(num_elements);
    Rand((double* const)v.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(num_elements, device_id);
    b0 = v;

    vector<Eigen::Vector3d> b0_d_data(num_elements);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0.h_data, v);
    ExpectEQ(b0_d_data, v);
}
#endif

// ----------------------------------------------------------------------------
// Assignment operator - from initializer list - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_initializer_list_CPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::Blob3d b0(0, open3d::DeviceID::CPU);

    b0 = {{-0.282003, -0.866394, -0.412111}, {-0.550791, -0.829572, 0.091869},
          {0.076085, -0.974168, 0.212620},   {-0.261265, -0.825182, -0.500814},
          {-0.035397, -0.428362, -0.902913}, {-0.711421, -0.595291, -0.373508},
          {-0.519141, -0.552592, -0.652024}, {0.490520, 0.573293, -0.656297},
          {-0.324029, -0.744177, -0.584128}, {0.120589, -0.989854, 0.075152},
          {-0.370700, -0.767066, -0.523632}, {0.874692, -0.158725, -0.457952},
          {-0.238700, -0.937064, 0.254819},  {-0.518237, -0.540189, -0.663043},
          {-0.238700, -0.937064, 0.254819},  {0.080943, -0.502095, -0.861016}};

    ExpectEQ(b0.h_data, ref);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Assignment operator - from initializer list - GPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_initializer_list_GPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::Blob3d b0(0, open3d::DeviceID::GPU_00);

    b0 = {{-0.282003, -0.866394, -0.412111}, {-0.550791, -0.829572, 0.091869},
          {0.076085, -0.974168, 0.212620},   {-0.261265, -0.825182, -0.500814},
          {-0.035397, -0.428362, -0.902913}, {-0.711421, -0.595291, -0.373508},
          {-0.519141, -0.552592, -0.652024}, {0.490520, 0.573293, -0.656297},
          {-0.324029, -0.744177, -0.584128}, {0.120589, -0.989854, 0.075152},
          {-0.370700, -0.767066, -0.523632}, {0.874692, -0.158725, -0.457952},
          {-0.238700, -0.937064, 0.254819},  {-0.518237, -0.540189, -0.663043},
          {-0.238700, -0.937064, 0.254819},  {0.080943, -0.502095, -0.861016}};

    size_t num_doubles = ref.size() * sizeof(Eigen::Vector3d) / sizeof(double);

    vector<Eigen::Vector3d> b0_d_data(ref.size());
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0_d_data, ref);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Assignment operator - from initializer list - CPU & GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Assignment_operator_initializer_list_CPU_GPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);
    open3d::Blob3d b0(0, device_id);

    b0 = {{-0.282003, -0.866394, -0.412111}, {-0.550791, -0.829572, 0.091869},
          {0.076085, -0.974168, 0.212620},   {-0.261265, -0.825182, -0.500814},
          {-0.035397, -0.428362, -0.902913}, {-0.711421, -0.595291, -0.373508},
          {-0.519141, -0.552592, -0.652024}, {0.490520, 0.573293, -0.656297},
          {-0.324029, -0.744177, -0.584128}, {0.120589, -0.989854, 0.075152},
          {-0.370700, -0.767066, -0.523632}, {0.874692, -0.158725, -0.457952},
          {-0.238700, -0.937064, 0.254819},  {-0.518237, -0.540189, -0.663043},
          {-0.238700, -0.937064, 0.254819},  {0.080943, -0.502095, -0.861016}};

    size_t num_doubles = ref.size() * sizeof(Eigen::Vector3d) / sizeof(double);

    vector<Eigen::Vector3d> b0_d_data(ref.size());
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    ExpectEQ(b0.h_data, ref);
    ExpectEQ(b0_d_data, ref);
}
#endif

// ----------------------------------------------------------------------------
// operator+ - both args on the CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Add_operator_CPU_CPU) {
    size_t num_elements = 100;

    vector<Eigen::Vector3d> b0_h_data(num_elements);
    Rand((double* const)b0_h_data.data(), num_elements * 3, 0.0, 10.0, 0);

    vector<Eigen::Vector3d> b1_h_data(num_elements);
    Rand((double* const)b1_h_data.data(), num_elements * 3, 0.0, 10.0, 1);

    open3d::Blob3d b0(b0_h_data, open3d::DeviceID::CPU);
    open3d::Blob3d b1(b1_h_data, open3d::DeviceID::CPU);
    open3d::Blob3d b2 = b0 + b1;

    EXPECT_EQ(b2.num_elements, b0.size() + b1.size());
    EXPECT_EQ(b2.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b2.h_data.size(), b0.size() + b1.size());
    EXPECT_FALSE(b2.h_data.empty());
    EXPECT_TRUE(NULL == b2.d_data);
    EXPECT_EQ(b2.size(), b0.size() + b1.size());

    for (size_t i = 0; i < b0.size(); i++) ExpectEQ(b0.h_data[i], b2.h_data[i]);

    for (size_t i = 0; i < b1.size(); i++)
        ExpectEQ(b1.h_data[i], b2.h_data[i + b0.size()]);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// operator+ - both args on the GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Add_operator_GPU_GPU) {
    size_t num_elements = 100;

    vector<Eigen::Vector3d> b0_h_data(num_elements);
    Rand((double* const)b0_h_data.data(), num_elements * 3, 0.0, 10.0, 0);

    vector<Eigen::Vector3d> b1_h_data(num_elements);
    Rand((double* const)b1_h_data.data(), num_elements * 3, 0.0, 10.0, 1);

    open3d::Blob3d b0(b0_h_data, open3d::DeviceID::GPU_00);
    open3d::Blob3d b1(b1_h_data, open3d::DeviceID::GPU_00);
    open3d::Blob3d b2 = b0 + b1;

    EXPECT_EQ(b2.num_elements, b0.size() + b1.size());
    EXPECT_EQ(b2.device_id, open3d::DeviceID::GPU_00);
    EXPECT_TRUE(b2.h_data.empty());
    EXPECT_TRUE(NULL != b2.d_data);
    EXPECT_EQ(b2.size(), b0.size() + b1.size());

    vector<Eigen::Vector3d> b0_d_data(b0.size());
    open3d::cuda::CopyDev2HstMemory(
            b0.d_data, (double* const)b0_d_data.data(),
            b0.size() * sizeof(Eigen::Vector3d) / sizeof(double));
    vector<Eigen::Vector3d> b1_d_data(b1.size());
    open3d::cuda::CopyDev2HstMemory(
            b1.d_data, (double* const)b1_d_data.data(),
            b1.size() * sizeof(Eigen::Vector3d) / sizeof(double));
    vector<Eigen::Vector3d> b2_d_data(b2.size());
    open3d::cuda::CopyDev2HstMemory(
            b2.d_data, (double* const)b2_d_data.data(),
            b2.size() * sizeof(Eigen::Vector3d) / sizeof(double));

    for (size_t i = 0; i < b0.size(); i++) ExpectEQ(b0_d_data[i], b2_d_data[i]);

    for (size_t i = 0; i < b1.size(); i++)
        ExpectEQ(b1_d_data[i], b2_d_data[i + b0.size()]);
}
#endif

// ----------------------------------------------------------------------------
// operator+= - both args on the CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Append_operator_CPU_CPU) {
    size_t num_elements = 100;

    vector<Eigen::Vector3d> b0_h_data(num_elements);
    Rand((double* const)b0_h_data.data(), num_elements * 3, 0.0, 10.0, 0);

    vector<Eigen::Vector3d> b1_h_data(num_elements);
    Rand((double* const)b1_h_data.data(), num_elements * 3, 0.0, 10.0, 1);

    size_t ref_size = b0_h_data.size() + b1_h_data.size();

    open3d::Blob3d b0(b0_h_data, open3d::DeviceID::CPU);
    open3d::Blob3d b1(b1_h_data, open3d::DeviceID::CPU);
    b0 += b1;

    EXPECT_EQ(b0.num_elements, ref_size);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), ref_size);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), ref_size);

    for (size_t i = 0; i < num_elements; i++)
        ExpectEQ(b0_h_data[i], b0.h_data[i]);

    for (size_t i = 0; i < b1.size(); i++)
        ExpectEQ(b1.h_data[i], b0.h_data[i + num_elements]);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// operator+= - both args on the GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Append_operator_GPU_GPU) {
    size_t num_elements = 100;

    vector<Eigen::Vector3d> b0_h_data(num_elements);
    Rand((double* const)b0_h_data.data(), num_elements * 3, 0.0, 10.0, 0);

    vector<Eigen::Vector3d> b1_h_data(num_elements);
    Rand((double* const)b1_h_data.data(), num_elements * 3, 0.0, 10.0, 1);

    size_t ref_size = b0_h_data.size() + b1_h_data.size();

    open3d::Blob3d b0(b0_h_data, open3d::DeviceID::GPU_00);
    open3d::Blob3d b1(b1_h_data, open3d::DeviceID::GPU_00);
    b0 += b1;

    EXPECT_EQ(b0.num_elements, ref_size);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::GPU_00);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), ref_size);

    vector<Eigen::Vector3d> b0_d_data(b0.size());
    open3d::cuda::CopyDev2HstMemory(
            b0.d_data, (double* const)b0_d_data.data(),
            b0.size() * sizeof(Eigen::Vector3d) / sizeof(double));
    vector<Eigen::Vector3d> b1_d_data(b1.size());
    open3d::cuda::CopyDev2HstMemory(
            b1.d_data, (double* const)b1_d_data.data(),
            b1.size() * sizeof(Eigen::Vector3d) / sizeof(double));

    for (size_t i = 0; i < num_elements; i++)
        ExpectEQ(b0_h_data[i], b0_d_data[i]);

    for (size_t i = 0; i < b1.size(); i++)
        ExpectEQ(b1_d_data[i], b0_d_data[i + num_elements]);
}
#endif

// ----------------------------------------------------------------------------
// Clear - CPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Clear_CPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    open3d::Blob3d b0(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), num_elements);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);

    b0.clear();

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::CPU);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Clear - GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Clear_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::GPU_00;

    open3d::Blob3d b0(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::GPU_00);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);

    b0.clear();

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_EQ(b0.device_id, open3d::DeviceID::GPU_00);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);
}
#endif

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Clear - CPU and GPU.
// ----------------------------------------------------------------------------
TEST(BlobType, Clear_CPU_GPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = (open3d::DeviceID::Type)(
            open3d::DeviceID::CPU | open3d::DeviceID::GPU_00);

    open3d::Blob3d b0(num_elements, device_id);

    EXPECT_EQ(b0.num_elements, num_elements);
    EXPECT_TRUE(open3d::DeviceID::CPU & b0.device_id);
    EXPECT_TRUE(open3d::DeviceID::GPU_00 & b0.device_id);
    EXPECT_EQ(b0.h_data.size(), num_elements);
    EXPECT_FALSE(b0.h_data.empty());
    EXPECT_TRUE(NULL != b0.d_data);
    EXPECT_EQ(b0.size(), num_elements);

    b0.clear();

    EXPECT_EQ(b0.num_elements, 0);
    EXPECT_TRUE(open3d::DeviceID::CPU & b0.device_id);
    EXPECT_TRUE(open3d::DeviceID::GPU_00 & b0.device_id);
    EXPECT_EQ(b0.h_data.size(), 0);
    EXPECT_TRUE(b0.h_data.empty());
    EXPECT_TRUE(NULL == b0.d_data);
    EXPECT_EQ(b0.size(), 0);
}
#endif

// ----------------------------------------------------------------------------
// begin() & end() iterators - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Begin_end_CPU) {
    size_t num_elements = 100;
    size_t num_doubles =
            num_elements * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::DeviceID::Type device_id = open3d::DeviceID::CPU;

    vector<Eigen::Vector3d> v(num_elements);
    Rand((double* const)v.data(), num_doubles, 0.0, 10.0, 0);

    open3d::Blob3d b0(num_elements, device_id);
    b0 = v;

    EXPECT_EQ(b0.size(), v.size());
    if (b0.size() != v.size()) return;

    std::vector<Eigen::Vector3d>::iterator it0;
    std::vector<Eigen::Vector3d>::iterator itv;
    for (it0 = b0.begin(), itv = v.begin(); it0 != b0.end() && itv != v.end();
         it0++, itv++) {
        EXPECT_NEAR((*it0)[0], (*itv)[0], unit_test::THRESHOLD);
        EXPECT_NEAR((*it0)[1], (*itv)[1], unit_test::THRESHOLD);
        EXPECT_NEAR((*it0)[2], (*itv)[2], unit_test::THRESHOLD);
    }
}

// ----------------------------------------------------------------------------
// Insert - value - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Insert_value_CPU) {
    Eigen::Vector3d ref = {-0.282003, -0.866394, -0.412111};

    open3d::Blob3d b0(0, open3d::DeviceID::CPU);

    b0.insert(b0.end(), ref);

    ExpectEQ(b0[0], ref);
}

// ----------------------------------------------------------------------------
// Insert - same value n times - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Insert_value_n_CPU) {
    Eigen::Vector3d ref = {-0.282003, -0.866394, -0.412111};

    open3d::Blob3d b0(0, open3d::DeviceID::CPU);

    size_t n = 100;
    b0.insert(b0.end(), n, ref);

    for (size_t i = 0; i < n; i++) ExpectEQ(b0[i], ref);
}

// ----------------------------------------------------------------------------
// Insert - vector - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Insert_vector_CPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::Blob3d b0(0, open3d::DeviceID::CPU);

    b0.insert(b0.end(), ref.begin(), ref.end());

    ExpectEQ(b0.h_data, ref);
}

// ----------------------------------------------------------------------------
// Insert - initializer list - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Insert_initializer_list_CPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::Blob3d b0(0, open3d::DeviceID::CPU);

    b0.insert(b0.end(), {{-0.282003, -0.866394, -0.412111},
                         {-0.550791, -0.829572, 0.091869},
                         {0.076085, -0.974168, 0.212620},
                         {-0.261265, -0.825182, -0.500814},
                         {-0.035397, -0.428362, -0.902913},
                         {-0.711421, -0.595291, -0.373508},
                         {-0.519141, -0.552592, -0.652024},
                         {0.490520, 0.573293, -0.656297},
                         {-0.324029, -0.744177, -0.584128},
                         {0.120589, -0.989854, 0.075152},
                         {-0.370700, -0.767066, -0.523632},
                         {0.874692, -0.158725, -0.457952},
                         {-0.238700, -0.937064, 0.254819},
                         {-0.518237, -0.540189, -0.663043},
                         {-0.238700, -0.937064, 0.254819},
                         {0.080943, -0.502095, -0.861016}});

    ExpectEQ(b0.h_data, ref);
}

// ----------------------------------------------------------------------------
// Push back - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Push_back_CPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::Blob3d b0(0, open3d::DeviceID::CPU);

    for (size_t i = 0; i < ref.size(); i++) b0.push_back(ref[i]);

    ExpectEQ(b0.h_data, ref);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Push back - GPU.
// Functional but has a high memory communication overhead.
// ----------------------------------------------------------------------------
TEST(BlobType, Push_back_GPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::Blob3d b0(0, open3d::DeviceID::GPU_00);

    for (size_t i = 0; i < ref.size(); i++) b0.push_back(ref[i]);

    vector<Eigen::Vector3d> b0_d_data = vector<Eigen::Vector3d>(b0.size());
    size_t num_doubles = b0.size() * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    EXPECT_EQ(b0.size(), ref.size());
    for (size_t i = 0; i < ref.size(); i++) ExpectEQ(b0_d_data[i], ref[i]);
}
#endif

// ----------------------------------------------------------------------------
// Resize - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Resize_CPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    open3d::Blob3d b0(ref, open3d::DeviceID::CPU);

    b0.resize(ref.size() << 1);

    EXPECT_EQ(b0.size(), ref.size() << 1);
    for (size_t i = 0; i < ref.size(); i++) ExpectEQ(b0[i], ref[i]);

    b0.resize(ref.size());

    EXPECT_EQ(b0.size(), ref.size());
    ExpectEQ(b0.h_data, ref);

    b0.resize(ref.size() >> 1);

    EXPECT_EQ(b0.size(), ref.size() >> 1);
    for (size_t i = 0; i < b0.size(); i++) ExpectEQ(b0[i], ref[i]);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Resize - GPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Resize_GPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    vector<Eigen::Vector3d> b0_d_data{};
    size_t num_doubles = 0;

    open3d::Blob3d b0(ref, open3d::DeviceID::GPU_00);

    b0.resize(ref.size() << 1);

    b0_d_data = vector<Eigen::Vector3d>(b0.size());
    num_doubles = b0.size() * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    EXPECT_EQ(b0.size(), ref.size() << 1);
    for (size_t i = 0; i < ref.size(); i++) ExpectEQ(b0_d_data[i], ref[i]);

    b0.resize(ref.size());

    b0_d_data = vector<Eigen::Vector3d>(b0.size());
    num_doubles = b0.size() * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    EXPECT_EQ(b0.size(), ref.size());
    ExpectEQ(b0_d_data, ref);

    b0.resize(ref.size() >> 1);

    b0_d_data = vector<Eigen::Vector3d>(b0.size());
    num_doubles = b0.size() * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    EXPECT_EQ(b0.size(), ref.size() >> 1);
    for (size_t i = 0; i < b0.size(); i++) ExpectEQ(b0_d_data[i], ref[i]);
}
#endif

// ----------------------------------------------------------------------------
// Resize - with value - CPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Resize_value_CPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    Eigen::Vector3d value = {1.1111, 2.22222, 3.333333};

    open3d::Blob3d b0(ref, open3d::DeviceID::CPU);

    b0.resize((ref.size() << 1), value);

    EXPECT_EQ(b0.size(), ref.size() << 1);
    for (size_t i = 0; i < ref.size(); i++) ExpectEQ(b0[i], ref[i]);
    for (size_t i = ref.size(); i < b0.size(); i++) ExpectEQ(b0[i], value);

    b0.resize(ref.size(), value);

    EXPECT_EQ(b0.size(), ref.size());
    ExpectEQ(b0.h_data, ref);

    b0.resize(ref.size() >> 1, value);

    EXPECT_EQ(b0.size(), ref.size() >> 1);
    for (size_t i = 0; i < b0.size(); i++) ExpectEQ(b0[i], ref[i]);
}

#ifdef OPEN3D_USE_CUDA
// ----------------------------------------------------------------------------
// Resize - with value - GPU only.
// ----------------------------------------------------------------------------
TEST(BlobType, Resize_value_GPU) {
    vector<Eigen::Vector3d> ref = {{-0.282003, -0.866394, -0.412111},
                                   {-0.550791, -0.829572, 0.091869},
                                   {0.076085, -0.974168, 0.212620},
                                   {-0.261265, -0.825182, -0.500814},
                                   {-0.035397, -0.428362, -0.902913},
                                   {-0.711421, -0.595291, -0.373508},
                                   {-0.519141, -0.552592, -0.652024},
                                   {0.490520, 0.573293, -0.656297},
                                   {-0.324029, -0.744177, -0.584128},
                                   {0.120589, -0.989854, 0.075152},
                                   {-0.370700, -0.767066, -0.523632},
                                   {0.874692, -0.158725, -0.457952},
                                   {-0.238700, -0.937064, 0.254819},
                                   {-0.518237, -0.540189, -0.663043},
                                   {-0.238700, -0.937064, 0.254819},
                                   {0.080943, -0.502095, -0.861016}};

    Eigen::Vector3d value = {1.1111, 2.22222, 3.333333};

    vector<Eigen::Vector3d> b0_d_data{};
    size_t num_doubles = 0;

    open3d::Blob3d b0(ref, open3d::DeviceID::GPU_00);

    b0.resize(ref.size() << 1, value);

    b0_d_data = vector<Eigen::Vector3d>(b0.size());
    num_doubles = b0.size() * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    EXPECT_EQ(b0.size(), ref.size() << 1);
    for (size_t i = 0; i < ref.size(); i++) ExpectEQ(b0_d_data[i], ref[i]);
    for (size_t i = ref.size(); i < b0.size(); i++)
        ExpectEQ(b0_d_data[i], value);

    b0.resize(ref.size(), value);

    b0_d_data = vector<Eigen::Vector3d>(b0.size());
    num_doubles = b0.size() * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    EXPECT_EQ(b0.size(), ref.size());
    ExpectEQ(b0_d_data, ref);

    b0.resize(ref.size() >> 1, value);

    b0_d_data = vector<Eigen::Vector3d>(b0.size());
    num_doubles = b0.size() * sizeof(Eigen::Vector3d) / sizeof(double);
    open3d::cuda::CopyDev2HstMemory(b0.d_data, (double* const)b0_d_data.data(),
                                    num_doubles);

    EXPECT_EQ(b0.size(), ref.size() >> 1);
    for (size_t i = 0; i < b0.size(); i++) ExpectEQ(b0_d_data[i], ref[i]);
}
#endif
