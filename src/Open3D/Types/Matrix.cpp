
#include <cassert>
#include <cmath>

#include "Matrix.h"

#include <iomanip>
using namespace std;

// ****************************************************************************
// Matrix<T, ROWS, COLS>
// ****************************************************************************

// ----------------------------------------------------------------------------
// subscript operator: readwrite
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
T* open3d::Matrix<T, ROWS, COLS>::Type::operator [](const uint& i)
{
    // catch error in debug mode
    assert(0 <= i && i < 3);

    return (T*)&s[i];
}

// ----------------------------------------------------------------------------
// subscript operator: readonly
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
const T* open3d::Matrix<T, ROWS, COLS>::Type::operator [](const uint& i) const
{
    // catch error in debug mode
    assert(0 <= i && i < 3);

    return (const T* const)&s[i];
}

// ----------------------------------------------------------------------------
// casting operator: readwrite
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
open3d::Matrix<T, ROWS, COLS>::Type::operator T* const()
{
    return reinterpret_cast<T*>(s);
}

// ----------------------------------------------------------------------------
// casting operator: readonly
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
open3d::Matrix<T, ROWS, COLS>::Type::operator const T* const()
{
    return reinterpret_cast<const T* const>(s);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
bool open3d::Matrix<T, ROWS, COLS>::Type::operator ==(
    const open3d::Matrix<T, ROWS, COLS>::Type& m)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            if ((*this)[r][c] != m[r][c])
                return false;

    return true;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
bool open3d::Matrix<T, ROWS, COLS>::Type::operator !=(
    const open3d::Matrix<T, ROWS, COLS>::Type& m)
{
    return !(*this == m);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
bool open3d::Matrix<T, ROWS, COLS>::Type::operator <=(
    const open3d::Matrix<T, ROWS, COLS>::Type& m)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            if ((*this)[r][c] > m[r][c])
                return false;

    return true;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
bool open3d::Matrix<T, ROWS, COLS>::Type::operator >=(
    const open3d::Matrix<T, ROWS, COLS>::Type& m)
{
    for (uint r = 0; r < ROWS; r++)
        if ((*this)[r] < m[r])
            return false;

    return true;
}

// ----------------------------------------------------------------------------
// addtion.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type
open3d::Matrix<T, ROWS, COLS>::Type::operator +(
    const open3d::Matrix<T, ROWS, COLS>::Type& m) const
{
    Matrix<T, ROWS, COLS>::Type output;
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            output[r][c] = (*this)[r][c] + m[r][c];

    return output;
}

// ----------------------------------------------------------------------------
// subtraction.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type
open3d::Matrix<T, ROWS, COLS>::Type::operator -(
    const open3d::Matrix<T, ROWS, COLS>::Type& m) const
{
    Matrix<T, ROWS, COLS>::Type output;
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            output[r][c] = (*this)[r][c] - m[r][c];

    return output;
}

// ----------------------------------------------------------------------------
// addtion assignment.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type&
open3d::Matrix<T, ROWS, COLS>::Type::operator +=(
    const open3d::Matrix<T, ROWS, COLS>::Type& m)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            (*this)[r][c] += m[r][c];

    return *this;
}

// ----------------------------------------------------------------------------
// subtraction assignment.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type&
open3d::Matrix<T, ROWS, COLS>::Type::operator -=(
    const open3d::Matrix<T, ROWS, COLS>::Type& m)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            (*this)[r][c] -= m[r][c];

    return *this;
}

// ----------------------------------------------------------------------------
// addtion.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type
open3d::Matrix<T, ROWS, COLS>::Type::operator +(const T& t) const
{
    Matrix<T, ROWS, COLS>::Type output;
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            output[r][c] = (*this)[r][c] + t;

    return output;
}

// ----------------------------------------------------------------------------
// subtraction.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type
open3d::Matrix<T, ROWS, COLS>::Type::operator -(const T& t) const
{
    Matrix<T, ROWS, COLS>::Type output;
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            output[r][c] = (*this)[r][c] - t;

    return output;
}

// ----------------------------------------------------------------------------
// multiply with scalar.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type
open3d::Matrix<T, ROWS, COLS>::Type::operator *(const T& t) const
{
    Matrix<T, ROWS, COLS>::Type output;
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            output[r][c] = (*this)[r][c] * t;

    return output;
}

// ----------------------------------------------------------------------------
// divide by scalar.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type
open3d::Matrix<T, ROWS, COLS>::Type::operator /(const T& t) const
{
    Matrix<T, ROWS, COLS>::Type output;
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            output[r][c] = (*this)[r][c] / t;

    return output;
}

// ----------------------------------------------------------------------------
// addtion assignment.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type&
open3d::Matrix<T, ROWS, COLS>::Type::operator +=(const T& t)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            (*this)[r][c] += t;

    return *this;
}

// ----------------------------------------------------------------------------
// subtraction assignment.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type&
open3d::Matrix<T, ROWS, COLS>::Type::operator -=(const T& t)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            (*this)[r][c] -= t;

    return *this;
}

// ----------------------------------------------------------------------------
// multiplication assignment.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type&
open3d::Matrix<T, ROWS, COLS>::Type::operator *=(const T& t)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            (*this)[r][c] *= t;

    return *this;
}

// ----------------------------------------------------------------------------
// division assignment.
// ----------------------------------------------------------------------------
template<typename T, uint ROWS, uint COLS>
typename open3d::Matrix<T, ROWS, COLS>::Type&
open3d::Matrix<T, ROWS, COLS>::Type::operator /=(const T& t)
{
    for (uint r = 0; r < ROWS; r++)
        for (uint c = 0; c < COLS; c++)
            (*this)[r][c] /= t;

    return *this;
}

// ****************************************************************************
// Vector<T, COLS>
// ****************************************************************************

// ----------------------------------------------------------------------------
// subscript operator: readwrite
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
T& open3d::Matrix<T, 1, COLS>::Type::operator [](const uint& i)
{
    // catch error in debug mode
    assert(0 <= i && i < 3);

    return s[i];
}

// ----------------------------------------------------------------------------
// subscript operator: readonly
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
const T& open3d::Matrix<T, 1, COLS>::Type::operator [](const uint& i) const
{
    // catch error in debug mode
    assert(0 <= i && i < 3);

    return s[i];
}

// ----------------------------------------------------------------------------
// casting operator: readwrite
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
open3d::Matrix<T, 1, COLS>::Type::operator T* const()
{
    return reinterpret_cast<T*>(s);
}

// ----------------------------------------------------------------------------
// casting operator: readonly
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
open3d::Matrix<T, 1, COLS>::Type::operator const T* const()
{
    return reinterpret_cast<const T* const>(s);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
bool open3d::Matrix<T, 1, COLS>::Type::operator ==(
    const open3d::Matrix<T, 1, COLS>::Type& v)
{
    for (uint c = 0; c < COLS; c++)
        if ((*this)[c] != v[c])
            return false;

    return true;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
bool open3d::Matrix<T, 1, COLS>::Type::operator !=(
    const open3d::Matrix<T, 1, COLS>::Type& v)
{
    return !(*this == v);
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
bool open3d::Matrix<T, 1, COLS>::Type::operator <=(
    const open3d::Matrix<T, 1, COLS>::Type& v)
{
    for (uint c = 0; c < COLS; c++)
        if ((*this)[c] > v[c])
            return false;

    return true;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
bool open3d::Matrix<T, 1, COLS>::Type::operator >=(
    const open3d::Matrix<T, 1, COLS>::Type& v)
{
    for (uint c = 0; c < COLS; c++)
        if ((*this)[c] < v[c])
            return false;

    return true;
}

// ----------------------------------------------------------------------------
// addtion.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type
open3d::Matrix<T, 1, COLS>::Type::operator +(
    const open3d::Matrix<T, 1, COLS>::Type& m) const
{
    Matrix<T, 1, COLS>::Type output;
    for (uint c = 0; c < COLS; c++)
        output[c] = (*this)[c] + m[c];

    return output;
}

// ----------------------------------------------------------------------------
// subtraction.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type
open3d::Matrix<T, 1, COLS>::Type::operator -(
    const open3d::Matrix<T, 1, COLS>::Type& m) const
{
    Matrix<T, 1, COLS>::Type output;
    for (uint c = 0; c < COLS; c++)
        output[c] = (*this)[c] - m[c];

    return output;
}

// ----------------------------------------------------------------------------
// addtion assignment.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type&
open3d::Matrix<T, 1, COLS>::Type::operator +=(
    const open3d::Matrix<T, 1, COLS>::Type& m)
{
    for (uint c = 0; c < COLS; c++)
        (*this)[c] += m[c];

    return *this;
}

// ----------------------------------------------------------------------------
// subtraction assignment.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type&
open3d::Matrix<T, 1, COLS>::Type::operator -=(
    const open3d::Matrix<T, 1, COLS>::Type& m)
{
    for (uint c = 0; c < COLS; c++)
        (*this)[c] -= m[c];

    return *this;
}

// ----------------------------------------------------------------------------
// addtion.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type
open3d::Matrix<T, 1, COLS>::Type::operator +(const T& t) const
{
    Matrix<T, 1, COLS>::Type output;
    for (uint c = 0; c < COLS; c++)
        output[c] = (*this)[c] + t;

    return output;
}

// ----------------------------------------------------------------------------
// subtraction.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type
open3d::Matrix<T, 1, COLS>::Type::operator -(const T& t) const
{
    Matrix<T, 1, COLS>::Type output;
    for (uint c = 0; c < COLS; c++)
        output[c] = (*this)[c] - t;

    return output;
}

// ----------------------------------------------------------------------------
// multiply with scalar.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type
open3d::Matrix<T, 1, COLS>::Type::operator *(const T& t) const
{
    Matrix<T, 1, COLS>::Type output;
    for (uint c = 0; c < COLS; c++)
        output[c] = (*this)[c] * t;

    return output;
}

// ----------------------------------------------------------------------------
// divide by scalar.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type
open3d::Matrix<T, 1, COLS>::Type::operator /(const T& t) const
{
    Matrix<T, 1, COLS>::Type output;
    for (uint c = 0; c < COLS; c++)
        output[c] = (*this)[c] / t;

    return output;
}

// ----------------------------------------------------------------------------
// addtion assignment.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type&
open3d::Matrix<T, 1, COLS>::Type::operator +=(const T& t)
{
    for (uint c = 0; c < COLS; c++)
        (*this)[c] += t;

    return *this;
}

// ----------------------------------------------------------------------------
// subtraction assignment.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type&
open3d::Matrix<T, 1, COLS>::Type::operator -=(const T& t)
{
    for (uint c = 0; c < COLS; c++)
        (*this)[c] -= t;

    return *this;
}

// ----------------------------------------------------------------------------
// multiplication assignment.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type&
open3d::Matrix<T, 1, COLS>::Type::operator *=(const T& t)
{
    for (uint c = 0; c < COLS; c++)
        (*this)[c] *= t;

    return *this;
}

// ----------------------------------------------------------------------------
// division assignment.
// ----------------------------------------------------------------------------
template<typename T, uint COLS>
typename open3d::Matrix<T, 1, COLS>::Type&
open3d::Matrix<T, 1, COLS>::Type::operator /=(const T& t)
{
    for (uint c = 0; c < COLS; c++)
        (*this)[c] /= t;

    return *this;
}

// ****************************************************************************
// Template instantiations:
// ****************************************************************************
// open3d::Matrix3d
template class open3d::Matrix<double, 3, 3>;
// open3d::Matrix3f
template class open3d::Matrix<float, 3, 3>;
// open3d::Matrix4d
template class open3d::Matrix<double, 4, 4>;
// open3d::Matrix4f
template class open3d::Matrix<float, 4, 4>;
// open3d::Matrix6d
template class open3d::Matrix<double, 6, 6>;
// open3d::Matrix6f
template class open3d::Matrix<float, 6, 6>;
// open3d::Vector3d
template class open3d::Matrix<double, 1, 3>;
// open3d::Vector3f
template class open3d::Matrix<float, 1, 3>;
// open3d::Vector3i
template class open3d::Matrix<int, 1, 3>;
