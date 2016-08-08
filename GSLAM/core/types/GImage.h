//#ifndef GIMAGE_H
//#define GIMAGE_H
//#include <GSLAM/core/GSLAM.h>

//namespace GSLAM{

//enum GElementType{
//    GElementType_8U =0,
//    GElementType_8S =1,
//    GElementType_16U=2,
//    GElementType_16S=3,
//    GElementType_32S=4,
//    GElementType_32F=5,
//    GElementType_64F=6,
//    GElementType_UserType=7
//};

//template <typename C>
//class GElement
//{
//    enum{Type=GElementType_UserType};
//};

//template <typename C>
//class GElement<uchar>
//{
//    enum{Type=GElementType_8U};
//};

//template <typename C>
//class GElement<char>
//{
//    enum{Type=GElementType_8S};
//};

//template <typename C>
//class GElement<pi::Int16>
//{
//    enum{Type=GElementType_16S};
//};

//template <typename C>
//class GElement<pi::UInt16>
//{
//    enum{Type=GElementType_16U};
//};

//template <typename C>
//class GElement<pi::Int32>
//{
//    enum{Type=GElementType_32S};
//};

//template <typename C>
//class GElement<float>
//{
//    enum{Type=GElementType_32F};
//};

//template <typename C>
//class GElement<double>
//{
//    enum{Type=GElementType_64F};
//};

//template <typename EleType=uchar,int channelSize=1>
//class GImageType
//{
//    enum{Type=((GElement<EleType>::Type&0x7)+((channelSize-1)<<3))};
//};

//class GImage
//{
//public:
//    GImage();
//    GImage(int width,int height,int type=GImageType<>::Type);
//    GImage(const GImage& ref);
//    ~GImage();

//    bool empty(){return !data;}
//    int  eleSize(){return channels()*((type&0x7)>>1);}
//    int  channels(){return (type>>3);}
//    int  type(){return flags;}
//    int  total(){return cols*rows*eleSize();}

//    int  cols,rows,flags;
//    uchar*          data;
//    int*            refCount;
//};

//}
//#endif // GIMAGE_H
