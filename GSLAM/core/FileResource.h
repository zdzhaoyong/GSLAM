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
// FileResource provides builtin resource file as buffer like Qt resources

#ifndef GSLAM_FILERESOURCE_H
#define GSLAM_FILERESOURCE_H

#include <fstream>
#include <istream>
#include <stdio.h>
#include <vector>
#include "Svar.h"

#define GSLAM_REGISTER_RESOURCE(R) \
    class ResourceRegister##R{\
public:\
    ResourceRegister##R(){GSLAM::FileResource::Register(resource_names,resource_index,resource_data);}\
}ResourceRegister##R##inst;

typedef unsigned char u_char;

namespace GSLAM{

class FileResource{
public:
    static std::string toHex(u_char* buf,int size){
        std::string str;
        str.resize(5*size,'0');
        for(int i=0;i<size;i++){
            u_char c=buf[i];
            sprintf(&str.at(5*i),"0x%02x,",c);
        }
        return str;
    }

    static std::string getBaseName(const std::string& path) {
      std::string filename = getFileName(path);
      auto idx = filename.find_last_of('.');
      if (idx == std::string::npos)
        return filename;
      else
        return filename.substr(0, idx);
    }

    static std::string getFileName(const std::string& path) {
      auto idx = std::string::npos;
      if ((idx = path.find_last_of('/')) == std::string::npos)
        idx = path.find_last_of('\\');
      if (idx != std::string::npos)
        return path.substr(idx + 1);
      else
        return path;
    }

    static bool exportResourceFile(const std::map<std::string,std::string>& lut,const std::string& to){
        std::vector<std::string> names;
        std::vector<int>         idxes;
        std::vector<u_char>      data;

        // load
        for(std::pair<std::string,std::string> corr:lut){
            FILE *in=fopen(corr.second.c_str(),"r");
            if(!in) {
                std::cerr<<"Failed to load file "<<corr.second<<std::endl;
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
            names.push_back(corr.first);
            idxes.push_back(finishIdx);
        }
        if(data.empty()) return false;

        // export
        std::ofstream fileOut(to,std::ios::out);
        if(!fileOut.is_open()) return false;
        fileOut<<"#include \"GSLAM/core/FileResource.h\"\n\n\n\n";

        fileOut<<"static const std::string resource_names[]={\n";
        for(size_t i=0;i<names.size();i++)
        {
            fileOut<<"\""<<names[i]<<"\",\n";
        }
        fileOut<<"\"\"};\n\n";

        fileOut<<"static const int resource_index[]={";
        for(size_t i=0;i<idxes.size();i++){
            fileOut<<idxes[i];
            if(i+1!=names.size()) fileOut<<",\n";
            else fileOut<<"};\n\n";
        }

        fileOut<<"static const unsigned char resource_data[]= {\n";
        for(size_t i=0;i<data.size();i+=16){
            if(i+16<=data.size()) fileOut<<toHex(&data[i],16)<<"\n";
            else fileOut<<toHex(&data[i],data.size()-i)<<"\n";
        }
        fileOut<<"};\n\n";
        fileOut<<"GSLAM_REGISTER_RESOURCE("<<getBaseName(to)<<")";
        fileOut.close();
        return true;
    }

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
            resources().set(name,SvarBuffer(&resource_data[start],end-start));
        }
    }

    static std::istream& getIStream(const std::string& resource){
        static Svar files;
        if(!files.exist(resource)) {
            SvarBuffer buffer=getBuffer(resource);
            files[resource]=std::shared_ptr<std::istream>(
                        new std::stringstream(std::string((const char*)buffer.ptr(),buffer.length())));
        }

        return files[resource].as<std::istream>();
    }

    static bool saveResource2File(const std::string& resource,
                                  const std::string& file){
        return getBuffer(resource).save(file);
    }

    static SvarBuffer getBuffer(const std::string& resource){
        return resources().get(resource,SvarBuffer(nullptr,0));
    }

    static Svar& resources(){
        static Svar r=Svar::object();
        return r;
    }
};

}

#endif
