#ifndef GSLAM_ARRAY_H
#define GSLAM_ARRAY_H
#include <iostream>

namespace pi {

template <class Type,int Size>
struct Array_
{
    Array_(){}
    Array_(Type def){
        for(int i=0;i<Size;i++)
            data[i]=def;
    }
    Type data[Size];

    inline friend std::ostream& operator <<(std::ostream& os,const Array_<Type,Size>& p)
    {
        for(int i=0;i<Size;i++)
            os<<p.data[i]<<" ";
        return os;
    }

    inline friend std::istream& operator >>(std::istream& os,const Array_<Type,Size>& p)
    {
        for(int i=0;i<Size;i++)
            os>>p.data[i];
        return os;
    }

    const int size(){return Size;}
};


template <int Size=2>
struct Byte
{
    unsigned char data[Size];
};

} // end namespace pi

#endif // ARRAY_H
