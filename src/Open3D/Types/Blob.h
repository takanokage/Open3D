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

#pragma once

#include <algorithm>
#include <initializer_list>
#include <memory>
#include <vector>
#include "Mat.h"

#ifdef OPEN3D_USE_CUDA
#include "Open3D/Utility/CUDA.cuh"
#endif

#include "Open3D/Types/DeviceID.h"

namespace open3d {

template <typename V, typename T>
struct Blob {
    typedef struct _Type {
        _Type() {}
        _Type(const int &num_elements,
              const DeviceID::Type &device_id = DeviceID::CPU)
            : num_elements(num_elements), device_id(device_id) {
            Initialize();
        }
        // copy constructor
        _Type(const _Type &t)
            : num_elements(t.num_elements), device_id(t.device_id) {
            Initialize();

            // copy host data
            if (OnCPU()) memcpy(h_data.data(), t.h_data.data(), num_of_bytes());

#ifdef OPEN3D_USE_CUDA
            // copy device data
            if (OnGPU()) cuda::CopyDev2DevMemory(t.d_data, d_data, num_of_Ts());
#endif
        }
        // move constructor
        _Type(_Type &&t)
            : num_elements(t.num_elements),
              device_id(t.device_id),
              h_data(std::move(t.h_data)),
              d_data(t.d_data) {
            t.d_data = NULL;
            t.num_elements = 0;
            t.device_id = DeviceID::CPU;
        }
        // init constructor
        _Type(const std::vector<V> &v, const DeviceID::Type &device_id)
            : num_elements(v.size()), device_id(device_id) {
            Initialize();

            // init host data
            if (OnCPU()) memcpy(h_data.data(), v.data(), num_of_bytes());

#ifdef OPEN3D_USE_CUDA
            // init device data
            if (OnGPU())
                cuda::CopyHst2DevMemory((const double *const)v.data(), d_data,
                                        num_of_Ts());
#endif
        }
        ~_Type() { Reset(); }

        // allocate memory
        void Initialize(const int &num_elements,
                        const DeviceID::Type &device_id) {
            this->num_elements = num_elements;
            this->device_id = device_id;

            if ((0 == h_data.size()) && (OnCPU()))
                h_data = std::vector<V>(num_elements);

#ifdef OPEN3D_USE_CUDA
            if ((NULL == d_data) && (OnGPU()))
                cuda::AllocateDeviceMemory(&d_data, num_of_Ts(), device_id);
#endif
        }
        void Initialize() { Initialize(num_elements, device_id); }
        // deallocate memory
        void Reset() {
            h_data.clear();

#ifdef OPEN3D_USE_CUDA
            cuda::ReleaseDeviceMemory(&d_data);
#endif

            num_elements = 0;
            device_id = DeviceID::CPU;
        }

        // total number of elements in this structure
        size_t num_elements{};
        // device id
        DeviceID::Type device_id = DeviceID::CPU;
        // host data container
        std::vector<V> h_data{};
        // device data pointer
        T *d_data{};

        // return true if the device_id includes the CPU.
        inline bool OnCPU() const { return DeviceID::CPU & device_id; }
        // return true if the device_id matches with one of the GPUs.
        inline bool OnGPU() const { return DeviceID::CPU != device_id; }

        // subscript operator: readwrite, host side only
        inline V &operator[](const uint &i) {
            if (OnCPU()) return h_data[i];
        }
        // subscript operator: readonly, host side only
        inline const V &operator[](const uint &i) const {
            if (OnCPU()) return h_data[i];
        }

        // compare contents for equality
        inline bool operator==(const _Type &r) {
            if (num_elements != r.num_elements) return false;

            return Read() == r.Read();
        }
        // compare contents for inequality
        inline bool operator!=(const _Type &r) { return !(*this == r); }

        // copy from another Blob
        // reset pointers, reinitialize and copy the data to hst/dev pointers
        inline _Type &operator=(const _Type &t) {
            Reset();

            num_elements = t.num_elements;
            device_id = t.device_id;

            Initialize();

            // copy host data
            if (OnCPU()) memcpy(h_data.data(), t.h_data.data(), num_of_bytes());

#ifdef OPEN3D_USE_CUDA
            // copy device data
            if (OnGPU()) cuda::CopyDev2DevMemory(t.d_data, d_data, num_of_Ts());
#endif

            return *this;
        }
        // move from another Blob
        inline _Type &operator=(_Type &&t) {
            Reset();

            num_elements = t.num_elements;
            device_id = t.device_id;

            // move host data
            if (OnCPU()) {
                h_data = std::move(t.h_data);
            }

#ifdef OPEN3D_USE_CUDA
            // move device data
            if (OnGPU()) {
                d_data = t.d_data;
                t.d_data = NULL;
            }
#endif

            t.num_elements = 0;
            t.device_id = DeviceID::CPU;

            return *this;
        }
        // initialize with host data
        // reset pointers, reinitialize and copy the data to hst/dev pointers
        inline _Type &operator=(const std::vector<V> &v) {
            DeviceID::Type bkp_device_id = device_id;

            Reset();

            num_elements = v.size();
            device_id = bkp_device_id;

            Initialize();

            // initialize host memory
            if (OnCPU()) memcpy(h_data.data(), v.data(), num_of_bytes());

#ifdef OPEN3D_USE_CUDA
            // initialize device memory
            if (OnGPU())
                cuda::CopyHst2DevMemory((const T *const)v.data(), d_data,
                                        num_of_Ts());
#endif

            return *this;
        }
        // initialize from an initializer list
        // reset pointers, reinitialize and copy the data to hst/dev pointers
        inline _Type &operator=(std::initializer_list<V> il) {
            DeviceID::Type bkp_device_id = device_id;

            Reset();

            std::vector<V> v(il);
            num_elements = v.size();
            device_id = bkp_device_id;

            Initialize();

            // initialize host memory
            if (OnCPU()) memcpy(h_data.data(), v.data(), num_of_bytes());

#ifdef OPEN3D_USE_CUDA
            // initialize device memory
            if (OnGPU())
                cuda::CopyHst2DevMemory((const T *const)v.data(), d_data,
                                        num_of_Ts());
#endif

            return *this;
        }

        // compose two Blobs to form another
        inline _Type operator+(const _Type &t) {
            // what if *this and t are on different devices?
            if (device_id != t.device_id) return _Type();

            size_t out_num_elements = num_elements + t.num_elements;
            DeviceID::Type out_device_id = device_id;

            _Type output(out_num_elements, out_device_id);

            // copy host data
            if (OnCPU()) {
                memcpy(output.h_data.data(), h_data.data(), num_of_bytes());
                memcpy(output.h_data.data() + num_elements, t.h_data.data(),
                       t.num_of_bytes());
            }

#ifdef OPEN3D_USE_CUDA
            // copy device data
            if (OnGPU()) {
                cuda::CopyDev2DevMemory(d_data, output.d_data, num_of_Ts());
                cuda::CopyDev2DevMemory(t.d_data, output.d_data + num_of_Ts(),
                                        t.num_of_Ts());
            }
#endif

            return output;
        }
        // append two Blobs
        inline _Type &operator+=(const _Type &t) {
            *this = *this + t;

            return *this;
        }

        // redirect to std:vector<V>::data()
        inline V *data() noexcept {
            // host only
            if (OnCPU()) return h_data.data();

            return NULL;
        }
        // redirect to std:vector<V>::data()
        inline const V *data() const noexcept {
            // host only
            if (OnCPU()) return h_data.data();

            return NULL;
        }
        // redirect to std:vector<V>::begin()
        inline typename std::vector<V>::iterator begin() noexcept {
            typename std::vector<V>::iterator output;

            // host only
            if (OnCPU()) output = h_data.begin();

            return output;
        }
        // redirect to std:vector<V>::begin()
        inline typename std::vector<V>::const_iterator begin() const noexcept {
            typename std::vector<V>::const_iterator output;

            // host only
            if (OnCPU()) output = h_data.begin();

            return output;
        }
        inline void clear() noexcept {
            // clear host memory
            // redirect to std:vector<V>::clear()
            if (OnCPU()) h_data.clear();

#ifdef OPEN3D_USE_CUDA
            // clear device memory
            if (OnGPU()) cuda::ReleaseDeviceMemory(&d_data);
#endif

            num_elements = 0;
        }
        // redirect to std:vector<V>::end()
        inline typename std::vector<V>::iterator end() noexcept {
            typename std::vector<V>::iterator output;

            // host only
            if (OnCPU()) output = h_data.end();

            return output;
        }
        // redirect to std:vector<V>::end()
        inline typename std::vector<V>::const_iterator end() const noexcept {
            typename std::vector<V>::const_iterator output;

            // host only
            if (OnCPU()) output = h_data.end();

            return output;
        }
        // redirect to std:vector<V>::empty()
        inline bool empty() const noexcept { return num_elements <= 0; }

        // TODO: insert works only on the host side for the moment.
        // Q: how to deal with device side?
        inline typename std::vector<V>::iterator insert(
                typename std::vector<V>::const_iterator position,
                const V &val) {
            typename std::vector<V>::iterator output;

            // host only
            if (OnCPU()) {
                // redirect to std:vector<V>::insert(...)
                output = h_data.insert(position, val);

                num_elements = h_data.size();
            }

            return output;
        }
        // redirect to std:vector<V>::insert(...)
        inline typename std::vector<V>::iterator insert(
                typename std::vector<V>::const_iterator position,
                size_t n,
                const V &val) {
            typename std::vector<V>::iterator output;

            // host only
            if (OnCPU()) {
                output = h_data.insert(position, n, val);

                num_elements = h_data.size();
            }

            return output;
        }
        // redirect to std:vector<V>::insert(...)
        template <typename InputIterator>
        inline typename std::vector<V>::iterator insert(
                typename std::vector<V>::const_iterator position,
                InputIterator first,
                InputIterator last) {
            typename std::vector<V>::iterator output;

            // host only
            if (OnCPU()) {
                output = h_data.insert(position, first, last);

                num_elements = h_data.size();
            }

            return output;
        }
        // redirect to std:vector<V>::insert(...)
        inline typename std::vector<V>::iterator insert(
                typename std::vector<V>::const_iterator position, V &&val) {
            typename std::vector<V>::iterator output;

            // host only
            if (OnCPU()) {
                output = h_data.insert(position, val);

                num_elements = h_data.size();
            }

            return output;
        }
        // redirect to std:vector<V>::insert(...)
        inline typename std::vector<V>::iterator insert(
                typename std::vector<V>::const_iterator position,
                std::initializer_list<V> il) {
            typename std::vector<V>::iterator output;

            // host only
            if (OnCPU()) {
                output = h_data.insert(position, il);

                num_elements = h_data.size();
            }

            return output;
        }
        inline void push_back(const V &val) {
            // host only
            if (OnCPU()) {
                // redirect to std:vector<V>::push_back(...)
                h_data.push_back(val);

                num_elements = h_data.size();
            }

#ifdef OPEN3D_USE_CUDA
            // device side
            // delete/reallocate device memory
            // Note: the overhead can be reduced at the cost of more complexity
            if (OnGPU()) {
                std::vector<V> data(num_elements);
                cuda::CopyDev2HstMemory(d_data, (T *const)data.data(),
                                        num_of_Ts());

                cuda::ReleaseDeviceMemory(&d_data);

                data.push_back(val);

                size_t new_size = data.size() * sizeof(V) / sizeof(T);
                cuda::AllocateDeviceMemory(&d_data, new_size, device_id);

                cuda::CopyHst2DevMemory((const T *const)data.data(), d_data,
                                        new_size);

                num_elements = data.size();
            }
#endif
        }
        inline void push_back(V &&val) { push_back(val); }
        // resize the memory allocated for storage.
        // this will actually resize both the host data and device data.
        // we could, in principle, avoid the resize depending on the usecase.
        // in the current mode memory is released/allocated on the spot.
        inline void resize(size_t n) {
            if (num_elements == n) return;

            // resize host data
            // redirect std:vector<V>::resize(...)
            if (OnCPU()) h_data.resize(n);

#ifdef OPEN3D_USE_CUDA
            // resize device data
            // delete/reallocate device memory
            // Note: the overhead can be reduced at the cost of more complexity
            if (OnGPU()) {
                std::vector<V> data(num_elements);
                cuda::CopyDev2HstMemory(d_data, (T *const)data.data(),
                                        num_of_Ts());

                cuda::ReleaseDeviceMemory(&d_data);

                data.resize(n);

                size_t new_size = n * sizeof(V) / sizeof(T);
                cuda::AllocateDeviceMemory(&d_data, new_size, device_id);

                cuda::CopyHst2DevMemory((const T *const)data.data(), d_data,
                                        new_size);
            }
#endif

            num_elements = n;
        }
        // redirect to std:vector<V>::resize(...)
        inline void resize(size_t n, const V &val) {
            if (num_elements == n) return;

            // resize host data
            // redirect std:vector<V>::resize(...)
            if (OnCPU()) h_data.resize(n, val);

#ifdef OPEN3D_USE_CUDA
            // resize device data
            // delete/reallocate device memory
            // Note: the overhead can be reduced at the cost of more complexity
            if (OnGPU()) {
                std::vector<V> data(num_elements);
                cuda::CopyDev2HstMemory(d_data, (T *const)data.data(),
                                        num_of_Ts());

                cuda::ReleaseDeviceMemory(&d_data);

                data.resize(n, val);

                size_t new_size = n * sizeof(V) / sizeof(T);
                cuda::AllocateDeviceMemory(&d_data, new_size, device_id);

                cuda::CopyHst2DevMemory((const T *const)data.data(), d_data,
                                        new_size);
            }
#endif

            num_elements = n;
        }
        // number of elements
        inline size_t size() const { return num_elements; }

    private:
        // number of T elements
        inline size_t num_of_Ts() const {
            return num_elements * sizeof(V) / sizeof(T);
        }
        // number of bytes
        inline size_t num_of_bytes() const { return num_elements * sizeof(V); }
        static inline bool Near(const V &a, const V &b) {
            for (size_t i = 0; i < a.size(); i++)
                if (abs(a[i] - b[i]) > 1e-6) return false;
            return true;
        }

    public:
        inline std::vector<V> Read() const {
            // copy host data
            if (OnCPU()) return h_data;

#ifdef OPEN3D_USE_CUDA
            // copy device data
            if (OnGPU()) {
                std::vector<V> data(num_elements);
                cuda::CopyDev2HstMemory(d_data, (T *const)data.data(),
                                        num_of_Ts());
                return data;
            }
#endif

            return std::vector<V>();
        }
    } Type;
};

typedef Blob<Eigen::Vector2i, int>::Type Blob2i;
typedef Blob<Eigen::Vector3i, int>::Type Blob3i;
typedef Blob<Eigen::Vector4i, int>::Type Blob4i;
typedef Blob<Eigen::Vector2d, double>::Type Blob2d;
typedef Blob<Eigen::Vector3d, double>::Type Blob3d;

typedef Blob3d Points;
typedef Blob3d Normals;
typedef Blob3d Colors;
typedef Blob3d Vertices;
typedef Blob3d Vertex_normals;
typedef Blob3d Vertex_colors;
typedef Blob3d Triangle_normals;
typedef Blob2i Lines;
typedef Blob3i Voxels;
typedef Blob3i Triangles;
typedef Blob2i CorrespondenceSet;
}  // namespace open3d
