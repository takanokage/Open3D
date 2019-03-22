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

#include <Eigen/Core>
#include <Open3D/Utility/Eigen.h>

namespace Json {
class Value;
}  // namespace Json

namespace open3d {
namespace utility {

/// Class IJsonConvertible defines the behavior of a class that can convert
/// itself to/from a json::Value.
class IJsonConvertible {
public:
    virtual ~IJsonConvertible() {}

public:
    virtual bool ConvertToJsonValue(Json::Value &value) const = 0;
    virtual bool ConvertFromJsonValue(const Json::Value &value) = 0;

public:
    static bool Vector3dFromJsonArray(Vec3d &vec, const Json::Value &value);
    static bool Vector3dToJsonArray(const Vec3d &vec, Json::Value &value);
    static bool Vector4dFromJsonArray(Vec4d &vec, const Json::Value &value);
    static bool Vector4dToJsonArray(const Vec4d &vec, Json::Value &value);
    static bool Matrix3dFromJsonArray(Mat3d &mat, const Json::Value &value);
    static bool Matrix3dToJsonArray(const Mat3d &mat, Json::Value &value);
    static bool Matrix4dFromJsonArray(Mat4d &mat, const Json::Value &value);
    static bool Matrix4dToJsonArray(const Mat4d &mat, Json::Value &value);
    static bool Matrix6dFromJsonArray(Mat6d &mat, const Json::Value &value);
    static bool Matrix6dToJsonArray(const Mat6d &mat, Json::Value &value);
};

}  // namespace utility
}  // namespace open3d
