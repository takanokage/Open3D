#pragma once

#include <cassert>
#include <cmath>

#include <iostream>

typedef unsigned int uint;

namespace open3d
{
    // 2D tensor, row major
    template<typename T, uint ROWS, uint COLS>
    struct Matrix
    {
        typedef struct _Type
        {
            T s[ROWS][COLS];

            // subscript operator: readwrite
            T* operator [](const uint& i);
            // subscript operator: readonly
            const T* operator [](const uint& i) const;

            // casting operator: readwrite
            explicit operator T* const();
            // casting operator: readonly
            explicit operator const T* const();

            bool operator ==(const _Type& m);
            bool operator !=(const _Type& m);
            bool operator <=(const _Type& m);
            bool operator >=(const _Type& m);

            // addition
            _Type operator +(const _Type& v) const;
            // subtraction
            _Type operator -(const _Type& v) const;
            // addition assignment
            _Type& operator +=(const _Type& v);
            // subtraction assignment
            _Type& operator -=(const _Type& v);
            // addition
            _Type operator +(const T& v) const;
            // subtraction
            _Type operator -(const T& v) const;
            // multiply with scalar
            _Type operator *(const T& v) const;
            // divide by scalar
            _Type operator /(const T& v) const;
            // addition assignment
            _Type& operator +=(const T& v);
            // subtraction assignment
            _Type& operator -=(const T& v);
            // multiplication assignment
            _Type& operator *=(const T& v);
            // division assignment
            _Type& operator /=(const T& v);
        } Type;
    };

    // 1D tensor, row major
    template<typename T, uint COLS>
    struct Matrix<T, 1, COLS>
    {
        typedef struct _Type
        {
            T s[COLS];

            // subscript operator: readwrite
            T& operator [](const uint& i);
            // subscript operator: readonly
            const T& operator [](const uint& i) const;

            // casting operator: readwrite
            explicit operator T* const();
            // casting operator: readonly
            explicit operator const T* const();

            bool operator ==(const _Type& m);
            bool operator !=(const _Type& m);
            bool operator <=(const _Type& m);
            bool operator >=(const _Type& m);

            // addition
            _Type operator +(const _Type& v) const;
            // subtraction
            _Type operator -(const _Type& v) const;
            // addition assignment
            _Type& operator +=(const _Type& v);
            // subtraction assignment
            _Type& operator -=(const _Type& v);
            // addition
            _Type operator +(const T& v) const;
            // subtraction
            _Type operator -(const T& v) const;
            // multiply with scalar
            _Type operator *(const T& v) const;
            // divide by scalar
            _Type operator /(const T& v) const;
            // addition assignment
            _Type& operator +=(const T& v);
            // subtraction assignment
            _Type& operator -=(const T& v);
            // multiplication assignment
            _Type& operator *=(const T& v);
            // division assignment
            _Type& operator /=(const T& v);
        } Type;
    };

    // 2D tensor, row major, size 3x3
    template<typename T>
    using Matrix3 = typename Matrix<T, 3, 3>::Type;
    // 2D tensor, row major, size 4x4
    template<typename T>
    using Matrix4 = typename Matrix<T, 4, 4>::Type;
    // 2D tensor, row major, size 6x6
    template<typename T>
    using Matrix6 = typename Matrix<T, 6, 6>::Type;

    // 1D tensor, row major, size 3
    template<typename T>
    using Vector3 = typename Matrix<T, 1, 3>::Type;

    // 2D 3x3 tensor
    typedef Matrix3<double> Matrix3d;
    typedef Matrix3<float> Matrix3f;

    // 2D 4x4 tensor
    typedef Matrix4<double> Matrix4d;
    typedef Matrix4<float> Matrix4f;

    // 2D 6x6 tensor
    typedef Matrix6<double> Matrix6d;
    typedef Matrix6<float> Matrix6f;

    // 1D 1x3 tensor
    typedef Vector3<double> Vector3d;
    typedef Vector3<float> Vector3f;
    typedef Vector3<int> Vector3i;
}
