#ifndef SIM3_H
#define SIM3_H

#include "SE3.h"

namespace pi {

/**
Represent a three-dimensional similarity transformation,7 degrees of freedom
a rotation:     R (Here is represented by 4 paraments quaternion:Class SO3)--3 DoF
a translation:  T (a 3 paraments vector)--3 DoF
a scale:        s  --1 DoF
It is the equivalent of a 4*4 Matrix:
 | R*s T^T|
 |  0   1 |

 This transformation is a member of the Lie group SIM3.
 These can be parameterised with seven numbers (in the space of the Lie Algebra).
 In this class, the first three parameters are a translation vector,
 while the second three are a rotation vector,whose direction is the axis of rotation,
 and length the amount of rotation (in radians), as for SO3.
 The seventh parameter is the log of the scale of the transformation.

**/

template <typename Precision =double>
class SIM3
{
public:
    /// Default constructor. Initialises the the rotation to zero (the identity),
    /// the scale to one and the translation to zero
    inline SIM3():my_scale(1){}

    SIM3(const SO3<Precision> &R,const Point3_<Precision> &T,const Precision& S=1.0)
        :my_se3(R,T),my_scale(S){}

    SIM3(const SE3<Precision> &T,const Precision& S=1.0)
        :my_se3(T),my_scale(S){}

    template<typename Scalar>
    inline operator SIM3<Scalar>()
    {
        return SIM3<Scalar>(my_se3,my_scale);
    }

    inline Point3_<Precision> get_translation()const
    {
        return my_se3.get_translation();
    }

    inline Point3_<Precision>& get_translation()
    {
        return my_se3.get_translation();
    }

    inline pi::SO3<Precision> get_rotation()const
    {
        return my_se3.get_rotation();
    }
    inline pi::SO3<Precision>& get_rotation()
    {
        return my_se3.get_rotation();
    }

    inline pi::SE3<Precision> get_se3()const
    {
        return my_se3;
    }
    inline pi::SE3<Precision>& get_se3()
    {
        return my_se3;
    }

    inline Precision& get_scale()
    {
        return my_scale;
    }

    inline Precision get_scale()const
    {
        return my_scale;
    }

    inline SIM3<Precision> operator *(const SIM3<Precision>& rhs) const
    {
        return SIM3<Precision>(my_se3.get_rotation()*rhs.get_rotation(),
                      get_translation() + get_rotation()*(get_scale()*rhs.get_translation()),
                      get_scale()*rhs.get_scale());
    }

    inline Point3_<Precision> operator *(const Point3_<Precision>& P) const
    {
        return get_rotation()*(P*my_scale)+my_se3.get_translation();
    }

    inline SIM3<Precision> inv()const
    {
        const SO3<Precision> rinv= get_rotation().inv();
        const Precision inv_scale= 1./my_scale;
        return SIM3(rinv,(-inv_scale)*(rinv*get_translation()),inv_scale);
    }

    /// v=(x,y,z,rx,ry,rz,s_log)^T
    inline SIM3<Precision> exp(const Array_<Precision,7>& v);

    inline Array_<Precision,7> ln()const;
protected:
    SE3<Precision> my_se3;
    Precision my_scale;
};


namespace Internal {

/// internal function that calculates the coefficients for the Rodrigues formula for SIM3 translation
template <typename Precision>
inline Point3_<Precision> compute_rodrigues_coefficients_sim3( const Precision & s, const Precision & t ){

    Point3_<Precision> coeff;
    const Precision es = exp(s);

    // 4 cases for s -> 0 and/or theta -> 0
    // the Taylor expansions were calculated with Maple 12 and truncated at the 3rd power,
    // such that eps^3 < 1e-18 which results in approximately 1 + eps^3 = 1
    static const Precision eps = 1e-6;

    if(fabs(s) < eps && fabs(t) < eps){
        coeff[0] = 1 + s/2 + s*s/6;
        coeff[1] = 1/2 + s/3 - t*t/24 + s*s/8;
        coeff[2] = 1/6 + s/8 - t*t/120 + s*s/20;
    } else if(fabs(s) < eps) {
        coeff[0] = 1 + s/2 + s*s/6;
        coeff[1] = (1-cos(t))/(t*t) + (sin(t)-cos(t)*t)*s/(t*t*t)+(2*sin(t)*t-t*t*cos(t)-2+2*cos(t))*s*s/(2*t*t*t*t);
        coeff[2] = (t-sin(t))/(t*t*t) - (-t*t-2+2*cos(t)+2*sin(t)*t)*s/(2*t*t*t*t) - (-t*t*t+6*cos(t)*t+3*sin(t)*t*t-6*sin(t))*s*s/(6*t*t*t*t*t);
    } else if(fabs(t) < eps) {
        coeff[0] = (es - 1)/s;
        coeff[1] = (s*es+1-es)/(s*s) - (6*s*es+6-6*es+es*s*s*s-3*es*s*s)*t*t/(6*s*s*s*s);
        coeff[2] = (es*s*s-2*s*es+2*es-2)/(2*s*s*s) - (es*s*s*s*s-4*es*s*s*s+12*es*s*s-24*s*es+24*es-24)*t*t/(24*s*s*s*s*s);
    } else {
        const Precision a = es * sin(t);
        const Precision b = es * cos(t);
        const Precision inv_s_theta = 1/(s*s + t*t);

        coeff[0] = (es - 1)/s;
        coeff[1] = (a*s + (1-b)*t) * inv_s_theta / t;
        coeff[2] = (coeff[0] - ((b-1)*s + a*t) * inv_s_theta) / (t*t);
    }

    return coeff;
}

}

template <typename Precision>
inline SIM3<Precision> SIM3<Precision>::exp(const Array_<Precision,7>& mu)
{

    SIM3<Precision> result;
#ifdef HAS_TOON
    // scale
    result.get_scale() = exp(mu.data[6]);

    // rotation
    const Point3_<Precision> w =*((Point3_<Precision>*)&mu.data[3]);
    const Precision t = sqrt(w.x*w.x+w.y*w.y+w.z*w.z);
    result.get_rotation() = SO3<Precision>::exp(w);

    // translation
    const Point3_<Precision> coeff = Internal::compute_rodrigues_coefficients_sim3(mu.data[6],t);
    const Point3_<Precision> trans =*((Point3_<Precision>*)&mu);
    const TooN::Vector<3,Precision> cross = w ^ trans;
    result.get_translation() = coeff[0] * mu.template slice<0,3>() + TooN::operator*(coeff[1], cross) + TooN::operator*(coeff[2], (w ^ cross));
#endif
    return result;
}

template <typename Precision>
inline Array_<Precision,7> SIM3<Precision>::ln()const
{

}

typedef SIM3<double> SIM3d;
typedef SIM3<float>  SIM3f;

} // end of namespace pi

#endif // SIM3_H
