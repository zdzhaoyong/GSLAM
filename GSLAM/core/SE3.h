#ifndef GSLAM_SE3_H
#define GSLAM_SE3_H

#include "SO3.h"

namespace pi
{
/**
    Represent a three-dimensional Euclidean transformation (a rotation and a translation).
              | r00 r01 r02 t0 |
M = | R  T |= | r10 r11 r12 t1 | = | Rx Ry Rz T |
    | 0  1 |  | r20 r21 r22 t2 |   | 0  0  0  1 |
              |  0   0   0   1 |
Here R=(Rx,Ry,Rz) is the rotation , T=(x,y,z)^T is the translation.

For the matrix M (only if the matrix means a camera to world transform):
Rx means the direction of X axis;
Ry means the direction of Y axis;
Rz means the direction of Z axis;
T  means the translation of this coordinate.


The coordinates of point P=(X,Y,Z)^T in this coordinate can be computed by left-multiply the matrix M:
@code
SE3<double> C2W;  // Camera to world SE3
Point3d P_cam(1,2,3); // Coordinates in the camera coordinate
cout<<"Coordinates in the world coordinate:"<<C2W*P_cam<<endl;

SE3<double> W2C=C2W.inv(); // It also can present a world to camera transform
Point3d P_w(4,5,6); // A point in the world
cout<<"The coodinates in the camera is "<<W2C*P_w;
@endcode

It should be noticed that when a SE3 is representing the world-to-camera transform,
the function <code>get_translation()</code> does not acqually return the translation.

 */
template <class Precision = double>
class SE3
{
public:
    typedef Point3_<Precision> Vec3;

public:
    SE3():my_rotation(0,0,0,1),my_translation(0,0,0){}

    SE3(const Precision& x,const Precision& y,const Precision& z,
        const Precision& wx,const Precision& wy,const Precision& wz,const Precision& w)
        :my_rotation(wx,wy,wz,w),my_translation(x,y,z){}

    SE3(const SO3<Precision>& r,const Vec3& t)
        :my_rotation(r),my_translation(t){}

    template <typename Scalar>
    operator SE3<Scalar>()
    {
        return SE3<Scalar>(my_rotation,my_translation);
    }

    /// Returns the rotation part of the transformation as a SO3
    inline SO3<Precision>& get_rotation(){return my_rotation;}
    /// @overload
    inline const SO3<Precision>& get_rotation() const {return my_rotation;}

    /// Returns the translation part of the transformation as a Vector
    inline Vec3& get_translation() {return my_translation;}
    /// @overload
    inline const Vec3& get_translation() const {return my_translation;}

    inline SE3 inverse() const {
        const SO3<Precision> rinv(my_rotation.inv());
        return SE3(rinv, -(rinv*my_translation));
    }

    /// Right-multiply by another SE3 (concatenate the two transformations)
    /// @param rhs The multipier
    template<typename P>
    inline SE3& operator *=(const SE3<P> & rhs) {
        my_translation = my_translation + my_rotation * rhs.get_translation();
        my_rotation = my_rotation*rhs.get_rotation();
        return *this;
    }

    template<typename P>
    inline SE3 operator *(const SE3<P>& rhs) const
    {
        return SE3(my_rotation*rhs.get_rotation(),
                    my_translation + my_rotation*rhs.get_translation());
    }

    template<typename P>
    inline bool operator <(const SE3<P>& rhs) const
    {
        return false;
    }

    /// Right-multiply by a Vector
    /// @relates SE3
    friend Vec3 operator*(const SE3<Precision>& lhs, const Vec3& rhs){
        return lhs.get_translation() + lhs.get_rotation() * rhs;
    }

    /// Write an SE3 to a stream
    /// @relates SE3
    friend inline std::ostream& operator <<(std::ostream& os, const SE3& rhs){
        os<<rhs.get_translation();
        os<<" "<<rhs.get_rotation();
        return os;
    }
    /// Write an SE3 from a stream
    /// @relates SE3
    friend inline std::istream& operator >>(std::istream& is, SE3& rhs){
        Precision x,y,z,rx,ry,rz,w;
        is>>x>>y>>z>>rx>>ry>>rz>>w;
        rhs=SE3(x,y,z,rx,ry,rz,w);
        return is;
    }


    /// R is described as follow, M is a 4*4 Homography matrix
    /// |r0 r1 r2|
    /// |r3 r4 r5|
    /// |r6 r7 r8|
    bool fromMatrix(const Precision* m)
    {
        Precision r[9];
        r[0]=m[0];  r[1]=m[1];  r[2] =m[2];  my_translation[0] =m[3];
        r[3]=m[4];  r[4]=m[5];  r[5] =m[6];  my_translation[1] =m[7];
        r[6]=m[8];  r[7]=m[9];  r[8]=m[10];  my_translation[2] =m[11];
        return my_rotation.fromMatrix(r);
    }

    template <typename T>
    bool fromMatrixUnsafe(T& matrix)
    {
        return fromMatrix((Precision*)&matrix);
    }

    //return the matrix
    bool getMatrix(Precision* m)const
    {
        Precision r[9];
        my_rotation.getMatrix(r);

        m[0]=r[0];  m[1]=r[1];  m[2] =r[2];  m[3] =my_translation[0];
        m[4]=r[3];  m[5]=r[4];  m[6] =r[5];  m[7] =my_translation[1];
        m[8]=r[6];  m[9]=r[7];  m[10]=r[8];  m[11]=my_translation[2];

        return true;
    }

    template <typename T>
    bool getMatrixUnsafe(T& matrix)
    {
        return getMatrix((Precision*)&matrix);
    }

#ifdef HAS_TOON
    operator TooN::SE3<Precision>()
    {
        return TooN::SE3<Precision>(my_rotation,*(TooN::Vector<3,Precision>*)&my_translation);
    }

    SE3<Precision>(const TooN::SE3<Precision>& tn)
    {
        my_rotation.fromMatrixUnsafe(tn.get_rotation());
        my_translation=*(Vec3*)&tn.get_translation();
    }
#endif

#ifdef SOPHUS_SE3_HPP
    operator Sophus::SE3Group<Precision>()
    {
        assert(sizeof(Sophus::SE3Group<Precision>) != sizeof(*this));
//        return *(Sophus::SE3Group<Precision>*)&my_rotation;// FIXME: segment fault occurs

        Sophus::SE3Group<Precision> sophusSE3;
        for(int i=0;i<7;i++) sophusSE3.data()[i]=((double*)this)[i];
        return sophusSE3;
    }

    SE3<Precision>(const Sophus::SE3Group<Precision>& sophus)
        : my_rotation(sophus.so3()),
          my_translation(*(Vec3*)&sophus.translation())
    {
    }
#endif

    inline Array_<Precision,6> log()const
    {
        Array_<Precision,6> result;
        const auto& l=my_rotation;
        const auto& t=my_translation;
        const Precision squared_w = l.w*l.w;
        const Precision n = sqrt(l.x*l.x+l.y*l.y+l.z*l.z);

        Precision A_inv;
        // Atan-based log thanks to
        //
        // C. Hertzberg et al.:
        // "Integrating Generic Sensor Fusion Algorithms with Sound State
        // Representation through Encapsulation of Manifolds"
        // Information Fusion, 2011

        if (n < NEAR_ZERO||true)
        {
            //If n is too small
            A_inv = 2./l.w - 2.*(1.0-squared_w)/(l.w*squared_w);
            Point3_<Precision> r(l.x*A_inv,l.y*A_inv,l.z*A_inv);
            Point3_<Precision> p=t-0.5*r.cross(t)+static_cast<Precision>(1. / 12.)*r.cross(r.cross(t));
            *(Point3_<Precision>*)&result.data=p;
            *(Point3_<Precision>*)&result.data[3]=r;
        }
        else
        {
            if (fabs(l.w)<NEAR_ZERO)
            {
                //If w is too small
                if (l.w>0)
                {
                    A_inv = M_PI/n;
                }
                else
                {
                    A_inv = -M_PI/n;
                }
            }
            else
                A_inv = 2*atan(n/l.w)/n;

            auto theta=A_inv*n;
            Point3_<Precision> r(l.x*A_inv,l.y*A_inv,l.z*A_inv);
            Point3_<Precision> a=r/theta;
            Point3_<Precision> p=t-0.5*r.cross(t)+(1-theta/(2*tan(0.5*theta)))*a.cross(a.cross(t));
            *(Point3_<Precision>*)&result.data=p;
            *(Point3_<Precision>*)&result.data[3]=r;
        }
        return result;
    }

    template <typename Scalar>
    static inline SE3<Scalar> exp(const Array_<Scalar,6>& l)
    {
        Point3_<Scalar> p(l.data[0],l.data[1],l.data[2]);
        Point3_<Scalar> r(l.data[3],l.data[4],l.data[5]);
        Scalar theta_sq = r.dot(r);
        Scalar theta    = sqrt(theta_sq);
        Scalar half_theta = static_cast<Scalar>(0.5) * theta;

        Scalar imag_factor;
        Scalar real_factor;

        if (theta < static_cast<Scalar>(1e-10)) {
          Scalar theta_po4 = theta_sq * theta_sq;
          imag_factor = static_cast<Scalar>(0.5) -
                        static_cast<Scalar>(1.0 / 48.0) * theta_sq +
                        static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
          real_factor = static_cast<Scalar>(1) -
                        static_cast<Scalar>(0.5) * theta_sq +
                        static_cast<Scalar>(1.0 / 384.0) * theta_po4;
        } else {
          Scalar sin_half_theta = sin(half_theta);
          imag_factor = sin_half_theta / theta;
          real_factor = cos(half_theta);
        }

        SO3<Scalar> R( imag_factor * r.x, imag_factor * r.y,imag_factor * r.z,real_factor);
        auto t= p+(1-cos(theta))/theta_sq*r.cross(p)+
                (theta-sin(theta))/(theta_sq*theta)*r.cross(r.cross(p));
        return SE3<Scalar>(R,t);
    }

    template <typename Scalar>
    static inline SE3<Scalar> expFast(const Array_<Scalar,6>& l)
    {
        Point3_<Scalar> p(l.data[0],l.data[1],l.data[2]);
        Point3_<Scalar> r(l.data[3],l.data[4],l.data[5]);
        Scalar theta_sq = r.dot(r);
        Scalar theta    = sqrt(theta_sq);
        Scalar half_theta = static_cast<Scalar>(0.5) * theta;

        Scalar imag_factor;
        Scalar real_factor;

        if (theta < static_cast<Scalar>(1e-10)) {
          Scalar theta_po4 = theta_sq * theta_sq;
          imag_factor = static_cast<Scalar>(0.5) -
                        static_cast<Scalar>(1.0 / 48.0) * theta_sq +
                        static_cast<Scalar>(1.0 / 3840.0) * theta_po4;
          real_factor = static_cast<Scalar>(1) -
                        static_cast<Scalar>(0.5) * theta_sq +
                        static_cast<Scalar>(1.0 / 384.0) * theta_po4;
        } else {
          Scalar sin_half_theta = SO3<Scalar>::sine(half_theta);
          imag_factor = sin_half_theta / theta;
          real_factor = SO3<Scalar>::cosine(half_theta);
        }

        SO3<Scalar> R( imag_factor * r.x, imag_factor * r.y,imag_factor * r.z,real_factor);
        auto t= p+(1-SO3<Scalar>::cosine(theta))/theta_sq*r.cross(p)+
                (theta-SO3<Scalar>::sine(theta))/(theta_sq*theta)*r.cross(r.cross(p));
        return SE3<Scalar>(R,t);
    }

    Array_<Precision,6> ln()const
    {
        return log();
    }

    SO3<Precision> getRotation()const{return get_rotation();}
    Point3_<Precision> getTranslation()const{return get_translation();}

    void setRotation(SO3<Precision> R){my_rotation=R;}
    void setTranslation(Point3_<Precision> t){my_translation=t;}

    SE3  mul (const SE3& rq) const{return (*this)*rq;}
    Point3_<Precision> trans(const Point3_<Precision>& p) const{return (*this)*p;}

    std::string toString()const{std::stringstream sst;sst<<*this;return sst.str();}

protected:
    SO3<Precision> my_rotation;
    Vec3 my_translation;
};

typedef SE3<double> SE3d;
typedef SE3<float > SE3f;

}
#endif // SE3_H
