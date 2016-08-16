

#include <vector>
#include <string>
#include <sstream>
#include <stdint-gcc.h>
#include <opencv2/features2d/features2d.hpp>

#include "FCNN.h"

using namespace std;
cv::SL2<float> sl2;

namespace DBoW2 {

// --------------------------------------------------------------------------

const int FCNN::L=CNN_FEATRUE_SIZE;

void FCNN::meanValue(const std::vector<FCNN::pDescriptor> &descriptors,
  FCNN::TDescriptor &mean)
{
  if(descriptors.empty())
  {
    return;
  }
  else if(descriptors.size() == 1)
  {
    mean = *descriptors[0];
  }
  else
  {
      FCNN::TDescriptor result=*descriptors[0]+*descriptors[1];
      for(int i=2,iend=descriptors.size();i<iend;i++)
          result=result+*descriptors[i];
      mean=result*(1./descriptors.size());
  }
}

// --------------------------------------------------------------------------


int FCNN::distance(const cv::Mat &a, const cv::Mat &b)
{
    return distance(*(FCNN::TDescriptor*)a.data,*(FCNN::TDescriptor*)b.data);
}

int FCNN::distance(const FCNN::TDescriptor &a,
  const FCNN::TDescriptor &b)
{
  return sl2((float*)a.data,(float*)b.data,CNN_FEATRUE_SIZE/sizeof(float));
}

// --------------------------------------------------------------------------

std::string FCNN::toString(const FCNN::TDescriptor &a)
{
  stringstream ss;

  float* p=(float*)a.data;
  for(int i = 0,iend=CNN_FEATRUE_SIZE/sizeof(float); i < iend; ++i, ++p)
  {
    ss << *p << " ";
  }

  return ss.str();
}

// --------------------------------------------------------------------------

void FCNN::fromString(FCNN::TDescriptor &a, const std::string &s)
{
    float *p =  (float*)a.data;

    stringstream ss(s);
    for(int i = 0,iend=CNN_FEATRUE_SIZE/sizeof(float); i < iend; ++i, ++p)
    {
        float n;
        ss >> n;

        if(!ss.fail())
            *p = n;
    }
}

// --------------------------------------------------------------------------

void FCNN::toMat32F(const std::vector<TDescriptor> &descriptors,
  cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  const size_t N = descriptors.size();
  mat.create(N,CNN_FEATRUE_SIZE/sizeof(float),CV_32F);



  mat.create(N, FCNN::L*8, CV_32F);

  if(0)
  {
      uchar* p=mat.data;
      for(size_t i = 0; i < N; ++i)
      {
          memcpy(p,descriptors[i].data,CNN_FEATRUE_SIZE);
          p+=CNN_FEATRUE_SIZE;
      }
  }
  else
  {
      memcpy(mat.data,descriptors.data(),CNN_FEATRUE_SIZE*N);
  }

}

// --------------------------------------------------------------------------

void FCNN::toMat8U(const std::vector<TDescriptor> &descriptors,
  cv::Mat &mat)
{
  mat.create(descriptors.size(), CNN_FEATRUE_SIZE, CV_8U);
  memcpy(mat.data,descriptors.data(),CNN_FEATRUE_SIZE*descriptors.size());
}

// --------------------------------------------------------------------------

} // namespace DBoW2


