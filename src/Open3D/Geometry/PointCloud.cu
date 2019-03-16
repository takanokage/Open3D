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

#include <stdio.h>

#include "Open3D/Utility/CUDA.cuh"
#include "Open3D/Types/Matrix3d.h"
using namespace open3d;

#include <iostream>
#include <vector>
#include <tuple>
using namespace std;

// ---------------------------------------------------------------------------
// cumulant kernel
// ---------------------------------------------------------------------------
__global__ void cumulant(double* data, int nrPoints, double* output) {
    int gid = blockIdx.x * blockDim.x + threadIdx.x;

    Vector3d* points = (Vector3d*)data;
    Matrix3d* cumulants = (Matrix3d*)output;

    Matrix3d c = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    Vector3d p = points[gid];
    c[0][0] += p[0];
    c[0][1] += p[1];
    c[0][2] += p[2];
    c[1][0] += p[0] * p[0];
    c[1][1] += p[0] * p[1];
    c[1][2] += p[0] * p[2];
    c[2][0] += p[1] * p[1];
    c[2][1] += p[1] * p[2];
    c[2][2] += p[2] * p[2];

    cumulants[gid][0][0] = c[0][0] / nrPoints;
    cumulants[gid][0][1] = c[0][1] / nrPoints;
    cumulants[gid][0][2] = c[0][2] / nrPoints;

    cumulants[gid][1][0] = c[1][0] / nrPoints;
    cumulants[gid][1][1] = c[1][1] / nrPoints;
    cumulants[gid][1][2] = c[1][2] / nrPoints;

    cumulants[gid][2][0] = c[2][0] / nrPoints;
    cumulants[gid][2][1] = c[2][1] / nrPoints;
    cumulants[gid][2][2] = c[2][2] / nrPoints;
}

// ---------------------------------------------------------------------------
// helper function calls the cumulant CUDA kernel
// ---------------------------------------------------------------------------
bool cumulantGPU(double* const d_points, const int& nrPoints,
    double* const d_cumulants) {
    int threadsPerBlock = 256;
    int blocksPerGrid =(nrPoints + threadsPerBlock - 1) / threadsPerBlock;

    cumulant<<<blocksPerGrid, threadsPerBlock>>>(d_points, nrPoints, d_cumulants);
    cudaDeviceSynchronize();

    cudaError_t status = cudaGetLastError();
    if (cudaSuccess != status) {
        cout << "status: " << cudaGetErrorString(status) << endl;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Compute PointCloud mean and covariance using the GPU
// ---------------------------------------------------------------------------
std::tuple<Vector3d, Matrix3d>
meanAndCovarianceCUDA(double* const d_points, const int& nrPoints) {
    Vector3d mean{};
    Matrix3d covariance = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    cout << "Running CUDA2..." << endl;

    // host memory
    vector<Matrix3d> h_cumulants(nrPoints);

    int outputSize = h_cumulants.size() * Matrix3d::SIZE;

    // allocate temporary device memory
    double *d_cumulants = NULL;
    if (!AlocateDevMemory(&d_cumulants, outputSize, "d_cumulants"))
        return std::make_tuple(mean, covariance);

    // execute on GPU
    //*/// v0
    if (!cumulantGPU(d_points, nrPoints, d_cumulants))
        return std::make_tuple(mean, covariance);
    /*/// v1
    int threadsPerBlock = 256;
    int blocksPerGrid =(nrPoints + threadsPerBlock - 1) / threadsPerBlock;

    cumulant<<<blocksPerGrid, threadsPerBlock>>>(d_points, nrPoints, d_cumulants);
    cudaDeviceSynchronize();

    cudaError_t status = cudaGetLastError();
    if (cudaSuccess != status) {
        cout << "status: " << cudaGetErrorString(status) << endl;
        return std::make_tuple(mean, covariance);
    }
    //*///

    // Copy results to the host
    if (!CopyDev2HstMemory(d_cumulants, (double *)&h_cumulants[0], outputSize))
        return std::make_tuple(mean, covariance);

    // Free temporary device memory
    if (!freeDev(&d_cumulants, "d_cumulants"))
        return std::make_tuple(mean, covariance);

    Matrix3d cumulant = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < h_cumulants.size(); i++) {
        cumulant[0][0] += (double)h_cumulants[i][0][0];
        cumulant[0][1] += (double)h_cumulants[i][0][1];
        cumulant[0][2] += (double)h_cumulants[i][0][2];
        cumulant[1][0] += (double)h_cumulants[i][1][0];
        cumulant[1][1] += (double)h_cumulants[i][1][1];
        cumulant[1][2] += (double)h_cumulants[i][1][2];
        cumulant[2][0] += (double)h_cumulants[i][2][0];
        cumulant[2][1] += (double)h_cumulants[i][2][1];
        cumulant[2][2] += (double)h_cumulants[i][2][2];
    }

    mean[0] = cumulant[0][0];
    mean[1] = cumulant[0][1];
    mean[2] = cumulant[0][2];

    covariance[0][0] = cumulant[1][0] - cumulant[0][0] * cumulant[0][0];
    covariance[1][1] = cumulant[2][0] - cumulant[0][1] * cumulant[0][1];
    covariance[2][2] = cumulant[2][2] - cumulant[0][2] * cumulant[0][2];
    covariance[0][1] = cumulant[1][1] - cumulant[0][0] * cumulant[0][1];
    covariance[1][0] = covariance[0][1];
    covariance[0][2] = cumulant[1][2] - cumulant[0][0] * cumulant[0][2];
    covariance[2][0] = covariance[0][2];
    covariance[1][2] = cumulant[2][1] - cumulant[0][1] * cumulant[0][2];
    covariance[2][1] = covariance[1][2];

    return std::make_tuple(mean, covariance);
}