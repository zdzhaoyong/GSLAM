// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao)
//
// Matrix provide basic matrix computation like Eigen

#ifndef GSLAM_CORE_MATRIX_H
#define GSLAM_CORE_MATRIX_H

#include <string.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <array>
#include <limits>

namespace GSLAM{

template<typename Scalar, size_t Rows, size_t Cols>
class Matrix
{
public:
    Scalar _data[Rows][Cols] {};

    Matrix() = default;

    Matrix(const std::array<Scalar,Rows*Cols>& c){
        memcpy(_data, c.data(), sizeof(_data));
    }

    Matrix(const std::initializer_list<Scalar>& c)
    {
        assert(c.size()==Rows*Cols);
        int i=0;
        for(auto it=c.begin();it!=c.end();it++,i++){
            data()[i]=*it;
        }
    }

    Matrix(const Scalar data_[Rows*Cols])
    {
        memcpy(_data, data_, sizeof(_data));
    }

    Matrix(const Scalar data_[Rows][Cols])
    {
        memcpy(_data, data_, sizeof(_data));
    }

    Matrix(const Matrix &other)
    {
        memcpy(_data, other._data, sizeof(_data));
    }

    static Matrix<Scalar, Rows, Cols> zeros() {
        Matrix<Scalar, Rows, Cols> m;
        m.setZero();
        return m;
    }

    static Matrix<Scalar, Rows, Cols> ones() {
        Matrix<Scalar, Rows, Cols> m;
        m.setOne();
        return m;
    }

    Scalar *data()
    {
        return _data[0];
    }

    const Scalar *data() const
    {
        return _data[0];
    }

    inline Scalar operator()(size_t i, size_t j) const
    {
        return _data[i][j];
    }

    inline Scalar &operator()(size_t i, size_t j)
    {
        return _data[i][j];
    }

    Matrix<Scalar, Rows, Cols> & operator=(const Matrix<Scalar, Rows, Cols> &other)
    {
        if (this != &other) {
            memcpy(_data, other._data, sizeof(_data));
        }
        return (*this);
    }

    void copyTo(Scalar (&dst)[Rows*Cols]) const
    {
        memcpy(dst, _data, sizeof(dst));
    }

    void copyToRaw(Scalar *dst) const
    {
        memcpy(dst, _data, sizeof(_data));
    }

    void copyToColumnMajor(Scalar (&dst)[Rows*Cols]) const
    {
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                dst[i+(j*Rows)] = self(i, j);
            }
        }
    }

    template<size_t P>
    Matrix<Scalar, Rows, P> operator*(const Matrix<Scalar, Cols, P> &other) const
    {
        const Matrix<Scalar, Rows, Cols> &self = *this;
        Matrix<Scalar, Rows, P> res;
        res.setZero();

        for (size_t i = 0; i < Rows; i++) {
            for (size_t k = 0; k < P; k++) {
                for (size_t j = 0; j < Cols; j++) {
                    res(i, k) += self(i, j) * other(j, k);
                }
            }
        }

        return res;
    }

    Matrix<Scalar, Rows, Cols> emult(const Matrix<Scalar, Rows, Cols> &other) const
    {
        Matrix<Scalar, Rows, Cols> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(i, j) = self(i, j)*other(i, j);
            }
        }

        return res;
    }

    Matrix<Scalar, Rows, Cols> edivide(const Matrix<Scalar, Rows, Cols> &other) const
    {
        Matrix<Scalar, Rows, Cols> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(i, j) = self(i, j)/other(i, j);
            }
        }

        return res;
    }

    Matrix<Scalar, Rows, Cols> operator+(const Matrix<Scalar, Rows, Cols> &other) const
    {
        Matrix<Scalar, Rows, Cols> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(i, j) = self(i, j) + other(i, j);
            }
        }

        return res;
    }

    Matrix<Scalar, Rows, Cols> operator-(const Matrix<Scalar, Rows, Cols> &other) const
    {
        Matrix<Scalar, Rows, Cols> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(i, j) = self(i, j) - other(i, j);
            }
        }

        return res;
    }

    Matrix<Scalar, Rows, Cols> operator-() const
    {
        Matrix<Scalar, Rows, Cols> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(i, j) = -self(i, j);
            }
        }

        return res;
    }

    void operator+=(const Matrix<Scalar, Rows, Cols> &other)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;
        self = self + other;
    }

    void operator-=(const Matrix<Scalar, Rows, Cols> &other)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;
        self = self - other;
    }

    template<size_t P>
    void operator*=(const Matrix<Scalar, Cols, P> &other)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;
        self = self * other;
    }

    Matrix<Scalar, Rows, Cols> operator*(Scalar scalar) const
    {
        Matrix<Scalar, Rows, Cols> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(i, j) = self(i, j) * scalar;
            }
        }

        return res;
    }

    inline Matrix<Scalar, Rows, Cols> operator/(Scalar scalar) const
    {
        return (*this)*(1/scalar);
    }

    Matrix<Scalar, Rows, Cols> operator+(Scalar scalar) const
    {
        Matrix<Scalar, Rows, Cols> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(i, j) = self(i, j) + scalar;
            }
        }

        return res;
    }

    inline Matrix<Scalar, Rows, Cols> operator-(Scalar scalar) const
    {
        return (*this) + (-1*scalar);
    }

    void operator*=(Scalar scalar)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                self(i, j) = self(i, j) * scalar;
            }
        }
    }

    void operator/=(Scalar scalar)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;
        self = self * (1.0f / scalar);
    }

    inline void operator+=(Scalar scalar)
    {
        *this = (*this) + scalar;
    }

    inline void operator-=(Scalar scalar)
    {
        *this = (*this) - scalar;
    }

    bool operator==(const Matrix<Scalar, Rows, Cols> &other) const
    {
        const Matrix<Scalar, Rows, Cols> &self = *this;

        // TODO: set this based on Type
        static constexpr Scalar eps = std::numeric_limits<Scalar>::epsilon();

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                if (fabs(self(i, j) - other(i, j)) > eps) {
                    return false;
                }
            }
        }

        return true;
    }

    bool operator!=(const Matrix<Scalar, Rows, Cols> &other) const
    {
        const Matrix<Scalar, Rows, Cols> &self = *this;
        return !(self == other);
    }

    bool equal(const Matrix<Scalar, Rows, Cols> &y, const Scalar eps=1e-4f) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                if (fabs((*this)(i, j) - y(i, j)) > eps) {
                    return false;
                }
            }
        }
        return true;
    }

    Matrix<Scalar, Cols, Rows> transpose() const
    {
        Matrix<Scalar, Cols, Rows> res;
        const Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                res(j, i) = self(i, j);
            }
        }

        return res;
    }

    // tranpose alias
    inline Matrix<Scalar, Cols, Rows> T() const
    {
        return transpose();
    }

    template<size_t P, size_t Q>
    Matrix<Scalar, P, Q> slice(size_t x0, size_t y0) const
    {
        Matrix<Scalar, P, Q> res(&(_data[x0][y0]));
        return res;
    }

    template<size_t P, size_t Q>
    void set(const Matrix<Scalar, P, Q> &m, size_t x0, size_t y0)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;
        for (size_t i = 0; i < P; i++) {
            for (size_t j = 0; j < Q; j++) {
                self(i + x0, j + y0) = m(i, j);
            }
        }
    }

    void setRow(size_t i, const Matrix<Scalar, Cols, 1> &row)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;
        for (size_t j = 0; j < Cols; j++) {
            self(i, j) = row(j, 0);
        }
    }

    void setCol(size_t j, const Matrix<Scalar, Rows, 1> &col)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;
        for (size_t i = 0; i < Rows; i++) {
            self(i, j) = col(i, 0);
        }
    }

    void setZero()
    {
        memset(_data, 0, sizeof(_data));
    }

    inline void zero()
    {
        setZero();
    }

    void setAll(Scalar val)
    {
        Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                self(i, j) = val;
            }
        }
    }

    inline void setOne()
    {
        setAll(1);
    }

    void setIdentity()
    {
        setZero();
        Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows && i < Cols; i++) {
            self(i, i) = 1;
        }
    }

    inline void identity()
    {
        setIdentity();
    }

    inline void swapRows(size_t a, size_t b)
    {
        if (a == b) {
            return;
        }

        Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t j = 0; j < Cols; j++) {
            Scalar tmp = self(a, j);
            self(a, j) = self(b, j);
            self(b, j) = tmp;
        }
    }

    inline void swapCols(size_t a, size_t b)
    {
        if (a == b) {
            return;
        }

        Matrix<Scalar, Rows, Cols> &self = *this;

        for (size_t i = 0; i < Rows; i++) {
            Scalar tmp = self(i, a);
            self(i, a) = self(i, b);
            self(i, b) = tmp;
        }
    }

    Matrix<Scalar, Rows, Cols> abs() const
    {
        Matrix<Scalar, Rows, Cols> r;
        for (size_t i=0; i<Rows; i++) {
            for (size_t j=0; j<Cols; j++) {
                r(i,j) = Scalar(fabs((*this)(i,j)));
            }
        }
        return r;
    }

    Scalar max() const
    {
        Scalar max_val = (*this)(0,0);
        for (size_t i=0; i<Rows; i++) {
            for (size_t j=0; j<Cols; j++) {
                Scalar val = (*this)(i,j);
                if (val > max_val) {
                    max_val = val;
                }
            }
        }
        return max_val;
    }

    Scalar min() const
    {
        Scalar min_val = (*this)(0,0);
        for (size_t i=0; i<Rows; i++) {
            for (size_t j=0; j<Cols; j++) {
                Scalar val = (*this)(i,j);
                if (val < min_val) {
                    min_val = val;
                }
            }
        }
        return min_val;
    }

    int rows() const {return Rows;}

    int cols() const {return Cols;}

    // Vector functions
    inline Scalar operator()(size_t i) const
    {
        static_assert(Cols==1,"Cols is not one.");
        return (*this)(i, 0);
    }

    inline Scalar &operator()(size_t i)
    {
        static_assert(Cols==1,"Cols is not one.");
        return (*this)(i, 0);
    }

    inline Scalar operator[](size_t i) const
    {
        static_assert(Cols==1,"Cols is not one.");
        return (*this)(i, 0);
    }

    inline Scalar &operator[](size_t i)
    {
        static_assert(Cols==1,"Cols is not one.");
        return (*this)(i, 0);
    }

    Scalar dot(const Matrix & b) const {
        static_assert(Cols==1,"Cols is not one.");
        Scalar r = 0;
        for (size_t i = 0; i<Rows; i++) {
            r += (*this)(i)*b(i,0);
        }
        return r;
    }

    Scalar norm() const {
        static_assert(Cols==1,"Cols must be one.");
        const Matrix &a(*this);
        return Scalar(sqrt(a.dot(a)));
    }

    Scalar norm_squared() const {
        static_assert(Cols==1,"Cols must be one.");
        const Matrix &a(*this);
        return a.dot(a);
    }

    Scalar length() const {
        static_assert(Cols==1,"Cols must be one.");
        return norm();
    }

    inline void normalize() {
        static_assert(Cols==1,"Cols must be one.");
        (*this) /= norm();
    }

    Matrix unit() const {
        static_assert(Cols==1,"Cols must be one.");
        return (*this) / norm();
    }

    Matrix unit_or_zero(const Scalar eps = Scalar(1e-5)) {
        static_assert(Cols==1,"Cols must be one.");
        const Scalar n = norm();
        if (n > eps) {
            return (*this) / n;
        }
        return Matrix();
    }

    Matrix normalized() const {
        return unit();
    }

    Matrix pow(Scalar v) const {
        static_assert(Cols==1,"Cols must be one.");
        Matrix r;
        for (size_t i = 0; i<Rows; i++) {
            r(i) = Scalar(::pow((*this)(i), v));
        }
        return r;
    }

    // Vector2

    Matrix(Scalar x, Scalar y)
    {
        static_assert(Cols==1&&Rows==2,"Vector3d not corrected constructed");
        Matrix &v(*this);
        v(0) = x;
        v(1) = y;
    }

    Scalar cross(const Matrix<Scalar,2,1> & b) const {
        static_assert(Cols==1&&Rows==2,"Vector3d not corrected constructed");
        const Matrix &a(*this);
        return a(0)*b(1, 0) - a(1)*b(0, 0);
    }

    Scalar operator^(const Matrix<Scalar,2,1> & b) const {
        static_assert(Cols==1&&Rows==2,"Vector3d not corrected constructed");
        return (*this).cross(b);
    }

    // Vector3
    Matrix(Scalar x, Scalar y, Scalar z) {
        static_assert(Cols==1&&Rows==3,"Vector3d not corrected constructed");
        Matrix &v(*this);
        v(0) = x;
        v(1) = y;
        v(2) = z;
    }

    Matrix cross(const Matrix<Scalar, 3, 1> & b) const {
        static_assert(Cols==1&&Rows==3,"Vector3d not corrected constructed");
        const Matrix &a(*this);
        Matrix c;
        c(0) = a(1)*b(2,0) - a(2)*b(1,0);
        c(1) = -a(0)*b(2,0) + a(2)*b(0,0);
        c(2) = a(0)*b(1,0) - a(1)*b(0,0);
        return c;
    }

    Matrix operator^(const Matrix<Scalar, 3, 1> & b) const {
        static_assert(Cols==1&&Rows==3,"Vector3d not corrected constructed");
        return (*this).cross(b);
    }

    Matrix<Scalar,3,3> hat() const {
        static_assert(Cols==1&&Rows==3,"Vector3d not corrected constructed");
        const Matrix &v(*this);
        Matrix<Scalar,3,3> A;
        A(0,0) = 0;
        A(0,1) = -v(2);
        A(0,2) = v(1);
        A(1,0) = v(2);
        A(1,1) = 0;
        A(1,2) = -v(0);
        A(2,0) = -v(1);
        A(2,1) = v(0);
        A(2,2) = 0;
        return A;
    }

#ifdef EIGEN_MATRIX_H
    operator Eigen::Matrix<Scalar,Rows,Cols,Eigen::ColMajor>()
    {
        return Eigen::Map<Eigen::Matrix<Scalar,Rows,Cols,Eigen::RowMajor> >(_data[0]);
    }

    Matrix(const Eigen::Matrix<Scalar,Rows,Cols,Eigen::ColMajor>& eigen)
    {
        Eigen::Matrix<Scalar,Rows,Cols,Eigen::RowMajor> rowM=eigen;
        memcpy(_data,rowM.data(),sizeof(_data));
    }

    Matrix(const Eigen::Matrix<Scalar,Rows,Cols,Eigen::RowMajor>& eigen)
    {
        memcpy(_data,eigen.data(),sizeof(_data));
    }
#endif

};
template<typename Scalar,size_t Rows,size_t Cols>
std::ostream& operator<<(std::ostream& os,const Matrix<Scalar, Rows, Cols>& matrix)
{
    os.precision(6);
    for (size_t i = 0; i < Rows; ++i) {
        for (size_t j = 0; j < Cols; ++j) {
            os << std::setw(9)
               << matrix(i, j)
               << ((j<Cols-1)?" ":"\n");
        }
    }
    return os;
}

template<typename Scalar, size_t  Rows, size_t Cols>
Matrix<Scalar, Rows, Cols> operator*(Scalar scalar, const Matrix<Scalar, Rows, Cols> &other)
{
    return other * scalar;
}

template <typename Scalar,size_t Rows>
using Vector=Matrix<Scalar,Rows,1>;

typedef Vector<float,2> Vector2f;
typedef Vector<float,3> Vector3f;
typedef Vector<float,4> Vector4f;
typedef Vector<float,5> Vector5f;
typedef Vector<float,6> Vector6f;
typedef Vector<double,2> Vector2d;
typedef Vector<double,3> Vector3d;
typedef Vector<double,4> Vector4d;
typedef Vector<double,5> Vector5d;
typedef Vector<double,6> Vector6d;

typedef Matrix<float,2,2> Matrix2f;
typedef Matrix<float,3,3> Matrix3f;
typedef Matrix<float,4,4> Matrix4f;
typedef Matrix<double,2,2> Matrix2d;
typedef Matrix<double,3,3> Matrix3d;
typedef Matrix<double,4,4> Matrix4d;

}

#endif
