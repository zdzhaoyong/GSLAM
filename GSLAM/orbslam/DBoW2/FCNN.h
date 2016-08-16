#ifndef FCNN_H
#define FCNN_H


#include <opencv/cv.h>
#include <vector>
#include <string>

#include "FClass.h"

#define CNN_FEATRUE_SIZE 256 //in byte

namespace DBoW2 {


/// Functions to manipulate ORB descriptors
class FCNN: protected FClass
{
public:

    /// Descriptor type
    //  typedef cv::Mat TDescriptor; // CV_8U
    struct TDescriptor
    {
        u_int8_t data[CNN_FEATRUE_SIZE];

        void fromMat(const cv::Mat &desc) {
            memcpy(data,desc.data,CNN_FEATRUE_SIZE);
        }

        static std::vector<TDescriptor> toVec(const cv::Mat &desc)
        {
            std::vector<TDescriptor> result;
            result.resize(desc.rows);
            memcpy(result.data(),desc.data,CNN_FEATRUE_SIZE*result.size());
            return result;
        }

        cv::Mat toMat()
        {
            cv::Mat result=cv::Mat::zeros(1, FCNN::L, CV_8U);
            memcpy(result.data,data,CNN_FEATRUE_SIZE);
            return result.clone();
        }

        const float& at(size_t i)const{return ((float*)data)[i];}
        float& at(size_t i) {return ((float*)data)[i];}

        friend TDescriptor operator + (const TDescriptor& a,const TDescriptor& b)
        {
            TDescriptor result;
            for(int i=0,iend=CNN_FEATRUE_SIZE/sizeof(float);i<iend;i++)
                result.at(i)=a.at(i)+b.at(i);
            return result;
        }

        friend TDescriptor operator * (const TDescriptor& a,const float& b)
        {
            TDescriptor result;
            for(int i=0,iend=CNN_FEATRUE_SIZE/sizeof(float);i<iend;i++)
                result.at(i)=a.at(i)*b;
            return result;
        }
    };

    /// Pointer to a single descriptor
    typedef const TDescriptor *pDescriptor;
    /// Descriptor length (in bytes)
    static const int L;

    //  /**
    //   * Calculates the mean value of a set of descriptors
    //   * @param descriptors
    //   * @param mean mean descriptor
    //   */
    static void meanValue(const std::vector<pDescriptor> &descriptors,
                          TDescriptor &mean);

    /**
   * Calculates the distance between two descriptors
   * @param a
   * @param b
   * @return distance
   */
    static int distance(const TDescriptor &a, const TDescriptor &b);

    static int distance(const cv::Mat &a, const cv::Mat &b);

    /**
   * Returns a string version of the descriptor
   * @param a descriptor
   * @return string version
   */
    static std::string toString(const TDescriptor &a);

    /**
   * Returns a descriptor from a string
   * @param a descriptor
   * @param s string version
   */
    static void fromString(TDescriptor &a, const std::string &s);

    /**
   * Returns a mat with the descriptors in float format
   * @param descriptors
   * @param mat (out) NxL 32F matrix
   */
    static void toMat32F(const std::vector<TDescriptor> &descriptors,
                         cv::Mat &mat);

    static void toMat8U(const std::vector<TDescriptor> &descriptors,
                        cv::Mat &mat);

};

} // namespace DBoW2
#endif // FCNN_H
