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

#ifndef GSLAM_SIM3_H
#define GSLAM_SIM3_H

#include "SE3.h"

namespace GSLAM {

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
class SIM3_
{
public:
    /// Default constructor. Initialises the the rotation to zero (the identity),
    /// the scale to one and the translation to zero
    inline SIM3_():my_scale(1){}

    SIM3_(const SO3_<Precision> &R,const Point3_<Precision> &T,const Precision& S=1.0)
        :my_se3(R,T),my_scale(S){}

    SIM3_(const SE3_<Precision> &T,const Precision& S=1.0)
        :my_se3(T),my_scale(S){}

    template<typename Scalar>
    inline operator SIM3_<Scalar>()
    {
        return SIM3_<Scalar>(my_se3,my_scale);
    }

    inline Point3_<Precision> get_translation()const
    {
        return my_se3.get_translation();
    }

    inline Point3_<Precision>& get_translation()
    {
        return my_se3.get_translation();
    }

    inline SO3_<Precision> get_rotation()const
    {
        return my_se3.get_rotation();
    }
    inline SO3_<Precision>& get_rotation()
    {
        return my_se3.get_rotation();
    }

    inline SE3_<Precision> get_se3()const
    {
        return my_se3;
    }
    inline SE3_<Precision>& get_se3()
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

    inline SIM3_<Precision> operator *(const SIM3_<Precision>& rhs) const
    {
        return SIM3_<Precision>(my_se3.get_rotation()*rhs.get_rotation(),
                      get_translation() + get_rotation()*(get_scale()*rhs.get_translation()),
                      get_scale()*rhs.get_scale());
    }

    inline Point3_<Precision> operator *(const Point3_<Precision>& P) const
    {
        return get_rotation()*(P*my_scale)+my_se3.get_translation();
    }

    inline SIM3_<Precision> inv()const
    {
        const SO3_<Precision> rinv= get_rotation().inv();
        const Precision inv_scale= 1./my_scale;
        return SIM3_(rinv,(-inv_scale)*(rinv*get_translation()),inv_scale);
    }

    static SIM3_ exp(const Vector<Precision,7>& mu)
    {
        Point3_<Precision> p(mu[0],mu[1],mu[2]);
        Point3_<Precision> r(mu[3],mu[4],mu[5]);
        Precision sigma=mu[6];
        Precision scale=::exp(sigma);
        Precision theta_sq = r.dot(r);
        Precision theta    = sqrt(theta_sq);
        Precision half_theta = static_cast<Precision>(0.5) * theta;

        Precision imag_factor;
        Precision real_factor;

        if (theta < static_cast<Precision>(1e-10)) {
          Precision theta_po4 = theta_sq * theta_sq;
          imag_factor = static_cast<Precision>(0.5) -
                        static_cast<Precision>(1.0 / 48.0) * theta_sq +
                        static_cast<Precision>(1.0 / 3840.0) * theta_po4;
          real_factor = static_cast<Precision>(1) -
                        static_cast<Precision>(0.5) * theta_sq +
                        static_cast<Precision>(1.0 / 384.0) * theta_po4;
        } else {
          Precision sin_half_theta = sin(half_theta);
          imag_factor = sin_half_theta / theta;
          real_factor = cos(half_theta);
        }

        Precision A, B, C;
        if (abs(sigma) < NEAR_ZERO) {
          C = 1.;
          if (abs(theta) < NEAR_ZERO) {
            A = 0.5;
            B = static_cast<Precision>(1. / 6.);
          } else {
            Precision theta_sq = theta * theta;
            A = (1. - cos(theta)) / theta_sq;
            B = (theta - sin(theta)) / (theta_sq * theta);
          }
        } else {
          C = (scale - 1.) / sigma;
          if (abs(theta) < NEAR_ZERO) {
            Precision sigma_sq = sigma * sigma;
            A = ((sigma - 1.) * scale + 1.) / sigma_sq;
            B = ((0.5 * sigma_sq - sigma + 1.) * scale) / (sigma_sq * sigma);
          } else {
            Precision a = scale * sin(theta);
            Precision b = scale * cos(theta);
            Precision c = theta_sq + sigma * sigma;
            A = (a * sigma + (1. - b) * theta) / (theta * c);
            B = (C - ((b - 1.) * sigma + a * theta) / (c)) * 1. / (theta_sq);
          }
        }

        SO3_<Precision> R( imag_factor * r.x, imag_factor * r.y,imag_factor * r.z,real_factor);
        auto rcp=r.cross(p);
        auto t= A*rcp+B*r.cross(rcp)+C*p;
        return SIM3_<Precision>(R,t,scale);
    }

    Vector<Precision,7> log() const{
        Vector<Precision,7> result;
        const auto& l=get_rotation();
        const auto& t=get_translation();
        const auto& scale=my_scale;
        const Precision squared_w = l.w*l.w;
        const Precision n = sqrt(l.x*l.x+l.y*l.y+l.z*l.z);
        const Precision sigma= ::log(scale);

        Precision A_inv;

        if (n < NEAR_ZERO)
        {
            //If n is too small
            A_inv = 2./l.w - 2.*(1.0-squared_w)/(l.w*squared_w);
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
        }

        Precision theta=A_inv*n;
        Point3_<Precision> r(l.x*A_inv,l.y*A_inv,l.z*A_inv);

        const Precision scale_sq = scale * scale;
        const Precision theta_sq = theta * theta;
        const Precision sin_theta = sin(theta);
        const Precision cos_theta = cos(theta);

        Precision a, b, c;
        if (abs(sigma * sigma) < NEAR_ZERO) {
            c = 1. - 0.5 * sigma;
            a = -0.5;
            if (abs(theta_sq) < NEAR_ZERO) {
                b = Precision(1. / 12.);
            } else {
                b = (theta * sin_theta + 2. * cos_theta - 2.) /
                        (2. * theta_sq * (cos_theta - 1.));
            }
        } else {
            const Precision scale_cu = scale_sq * scale;
            c = sigma / (scale - 1.);
            if (abs(theta_sq) < NEAR_ZERO) {
                a = (-sigma * scale + scale - 1.) / ((scale - 1.) * (scale - 1.));
                b = (scale_sq * sigma - 2. * scale_sq + scale * sigma + 2. * scale) /
                        (2. * scale_cu - 6. * scale_sq + 6. * scale - 2.);
            } else {
                const Precision s_sin_theta = scale * sin_theta;
                const Precision s_cos_theta = scale * cos_theta;
                a = (theta * s_cos_theta - theta - sigma * s_sin_theta) /
                        (theta * (scale_sq - 2. * s_cos_theta + 1.));
                b = -scale *
                        (theta * s_sin_theta - theta * sin_theta + sigma * s_cos_theta -
                         scale * sigma + sigma * cos_theta - sigma) /
                        (theta_sq * (scale_cu - 2. * scale * s_cos_theta - scale_sq +
                                     2. * s_cos_theta + scale - 1.));
            }
        }

        auto rcrosst=r.cross(t);
        Point3_<Precision> p=a*rcrosst+b*r.cross(rcrosst)+c*t;
        result.set(Vector3<Precision>(p.x,p.y,p.z),0,0);
        result.set(Vector3<Precision>(r.x,r.y,r.z),3,0);
        result[6]=sigma;
        return result;
    }

#ifdef SOPHUS_SIM3_HPP
    SIM3_(const Sophus::Sim3Group<Precision>& sim3)
    {
        Eigen::Quaternion<Precision> quat(sim3.rxso3().quaternion().coeffs());
        quat.normalize();
        get_rotation()=Sophus::SO3Group<Precision>(quat);
        get_translation()=*(Point3_<Precision>*)sim3.translation().data();
        get_scale()=sim3.scale();
    }
    operator Sophus::Sim3Group<Precision>()
    {
        auto translation=get_translation();
        return Sophus::Sim3Group<Precision>(Sophus::RxSO3Group<Precision>(std::sqrt(get_scale()),
                                                          get_rotation()),
                                    Eigen::Map<Eigen::Matrix<Precision, 3, 1>>(&translation.x));
    }
#endif
protected:
    SE3_<Precision> my_se3;
    Precision my_scale;
};

typedef SIM3_<double> SIM3d;
typedef SIM3_<float>  SIM3f;
typedef SIM3d SIM3;

} // end of namespace GSLAM


#endif // SIM3_H
