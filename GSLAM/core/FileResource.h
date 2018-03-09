#include <fstream>
#include <istream>
#include <stdio.h>
#include <vector>
#include <GSLAM/core/Svar.h>
#include <GSLAM/core/SPtr.h>

#define GSLAM_REGISTER_RESOURCE(R) \
    class ResourceRegister##R{\
public:\
    ResourceRegister##R(){GSLAM::FileResource::Register(resource_names,resource_index,resource_data);}\
}ResourceRegister##R##inst;
typedef unsigned char u_char;
namespace GSLAM{

class FileResource{
public:
    typedef std::vector<std::pair<std::string,std::string> > FileLUT;

    static std::string toHex(u_char* buf,int size){
        std::string str;
        str.resize(5*size,'0');
        for(int i=0;i<size;i++){
            u_char c=buf[i];
            sprintf(&str.at(5*i),"0x%02x,",c);
        }
        return str;
    }

    static bool exportResourceFile(const FileLUT& lut,const std::string& to){
        std::vector<std::string> names;
        std::vector<int>         idxes;
        std::vector<u_char>      data;

        // load
        for(int i=0;i<lut.size();i++){
            std::pair<std::string,std::string> corr=lut[i];
            FILE *in=fopen(corr.first.c_str(),"r");
            if(!in) {
                std::cerr<<"Failed to load file "<<corr.first<<std::endl;
                continue;
            }
            while(true)
            {
                int ch=fgetc(in);
                if(ch==EOF) break;
                data.push_back(ch);
            }
            fclose(in);
            int finishIdx=data.size();
            names.push_back(corr.second);
            idxes.push_back(finishIdx);
        }
        if(data.empty()) return false;

        // export
        std::ofstream fileOut(to,std::ios::out);
        if(!fileOut.is_open()) return false;
        fileOut<<"#include \"GSLAM/core/FileResource.h\"\n\n\n\n";

        fileOut<<"static const std::string resource_names[]={\n";
        for(int i=0;i<names.size();i++)
        {
            fileOut<<"\""<<names[i]<<"\",\n";
        }
        fileOut<<"\"\"};\n\n";

        fileOut<<"static const int resource_index[]={";
        for(int i=0;i<idxes.size();i++){
            fileOut<<idxes[i];
            if(i+1!=names.size()) fileOut<<",\n";
            else fileOut<<"};\n\n";
        }

        fileOut<<"static const unsigned char resource_data[]= {\n";
        for(int i=0;i<data.size();i+=16){
            if(i+16<=data.size()) fileOut<<toHex(&data[i],16)<<"\n";
            else fileOut<<toHex(&data[i],data.size()-i)<<"\n";
        }
        fileOut<<"};\n\n";
        fileOut<<"GSLAM_REGISTER_RESOURCE("<<GSLAM::Svar::getBaseName(to)<<")";
        fileOut.close();
        return true;
    }

    struct FileBuffer{
        FileBuffer(const u_char* b=NULL,int sz=0):buf(b),size(sz){}
        const u_char* buf;
        int     size;
    };

    static void Register(const std::string resource_names[],
                         const int resource_index[],
                         const unsigned char resource_data[])
    {
        for(int i=0;;i++)
        {
            std::string name=resource_names[i];
            if(name.empty()) break;
            int start= i==0?0:resource_index[i-1];
            int end  =resource_index[i];
            GSLAM::SvarWithType<FileBuffer>::instance().insert(name,{&resource_data[start],end-start});
        }
    }
    static char* ccchar(const char* cch)
    {
     char* pcch = (char*)cch;
     return pcch;
    }

    static std::pair<char*,int> getResource(const std::string& resource){
        FileBuffer buf=GSLAM::SvarWithType<FileBuffer>::instance()[resource];
        if(!buf.buf||buf.size<=0) std::pair<char*,int>(NULL,0);
        return std::pair<char*,int>(ccchar((const char*)buf.buf),buf.size);
    }

    static bool saveResource2File(const std::string& resource,
                                  const std::string& file){
        FileBuffer buf=GSLAM::SvarWithType<FileBuffer>::instance()[resource];
        if(!buf.buf||buf.size<=0) return false;
        std::ofstream ofs(file,std::ios::out|std::ios::binary);
        if(!ofs.is_open()) return false;
        ofs.write((const char*)buf.buf,buf.size);
        return ofs.good();
    }
};

}

