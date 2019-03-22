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

#include "IJsonConvertible.h"

#include <json/json.h>

namespace open3d {
namespace utility {

bool IJsonConvertible::Vector3dFromJsonArray(Vec3d &vec,
                                             const Json::Value &value) {
    if (value.size() != Vec3d::Size) return false;

    vec[0] = value[0].asDouble();
    vec[1] = value[1].asDouble();
    vec[2] = value[2].asDouble();

    return true;
}

bool IJsonConvertible::Vector3dToJsonArray(const Vec3d &vec,
                                           Json::Value &value) {
    value.clear();

    value.append(vec[0]);
    value.append(vec[1]);
    value.append(vec[2]);

    return true;
}

bool IJsonConvertible::Vector4dFromJsonArray(Vec4d &vec,
                                             const Json::Value &value) {
    if (value.size() != Vec4d::Size) return false;

    vec[0] = value[0].asDouble();
    vec[1] = value[1].asDouble();
    vec[2] = value[2].asDouble();
    vec[3] = value[3].asDouble();

    return true;
}

bool IJsonConvertible::Vector4dToJsonArray(const Vec4d &vec,
                                           Json::Value &value) {
    value.clear();

    value.append(vec[0]);
    value.append(vec[1]);
    value.append(vec[2]);
    value.append(vec[3]);

    return true;
}

bool IJsonConvertible::Matrix3dFromJsonArray(Mat3d &mat,
                                             const Json::Value &value) {
    if (value.size() != Mat3d::Size) return false;

    for (uint r = 0; r < Mat3d::Rows; r++)
        for (uint c = 0; c < Mat3d::Cols; c++)
            mat.s[r][c] = value[r * Mat3d::Cols + c].asDouble();

    return true;
}

bool IJsonConvertible::Matrix3dToJsonArray(const Mat3d &mat,
                                           Json::Value &value) {
    value.clear();

    for (uint r = 0; r < Mat3d::Rows; r++)
        for (uint c = 0; c < Mat3d::Cols; c++) value.append(mat.s[r][c]);

    return true;
}

bool IJsonConvertible::Matrix4dFromJsonArray(Mat4d &mat,
                                             const Json::Value &value) {
    if (value.size() != Mat4d::Size) return false;

    for (uint r = 0; r < Mat4d::Rows; r++)
        for (uint c = 0; c < Mat4d::Cols; c++)
            mat.s[r][c] = value[r * Mat4d::Cols + c].asDouble();

    return true;
}

bool IJsonConvertible::Matrix4dToJsonArray(const Mat4d &mat,
                                           Json::Value &value) {
    value.clear();

    for (uint r = 0; r < Mat4d::Rows; r++)
        for (uint c = 0; c < Mat4d::Cols; c++) value.append(mat.s[r][c]);

    return true;
}

bool IJsonConvertible::Matrix6dFromJsonArray(Mat6d &mat,
                                             const Json::Value &value) {
    if (value.size() != Mat6d::Size) return false;

    for (uint r = 0; r < Mat6d::Rows; r++)
        for (uint c = 0; c < Mat6d::Cols; c++)
            mat.s[r][c] = value[r * Mat6d::Cols + c].asDouble();

    return true;
}

bool IJsonConvertible::Matrix6dToJsonArray(const Mat6d &mat,
                                           Json::Value &value) {
    value.clear();

    for (uint r = 0; r < Mat6d::Rows; r++)
        for (uint c = 0; c < Mat6d::Cols; c++) value.append(mat.s[r][c]);

    return true;
}

}  // namespace utility
}  // namespace open3d
