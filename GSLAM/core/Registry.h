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
// Registry is used to load/get gslam plugins as Svar or SharedLibrary

#ifndef GSLAM_REGISTRY_H
#define GSLAM_REGISTRY_H
#include "SharedLibrary.h"
#include "Svar.h"

namespace GSLAM {

class Registry
{
public:
    typedef std::set<std::string> FilePathList;
    Registry(){
        updatePaths();
    }

    static Svar load(std::string pluginName){
        SharedLibraryPtr plugin=get(pluginName);
        if(!plugin)
        {
            std::cerr<<"Unable to load plugin "<<pluginName<<std::endl;
            std::cerr<<"PATH=";
            for(std::string p:instance()._libraryFilePath)
                std::cerr<<p<<":";
            std::cerr<<std::endl;
            return Svar();
        }
        if(!plugin->hasSymbol("svarInstance"))
        {
            std::cerr<<"Unable to find symbol svarInstance."<<std::endl;
            return Svar();
        }

        GSLAM::Svar* (*getInst)()=(GSLAM::Svar* (*)())plugin->getSymbol("svarInstance");
        if(!getInst){
            std::cerr<<"No svarInstance found in "<<pluginName<<std::endl;
            return Svar();
        }
        GSLAM::Svar* inst=getInst();
        if(!inst){
            std::cerr<<"svarInstance returned null.\n";
            return Svar();
        }

        return *inst;
    }

    static Registry& instance()
    {
        static std::shared_ptr<Registry> reg(new Registry);
        return *reg;
    }

    static SharedLibraryPtr get(std::string pluginName)
    {
        if(pluginName.empty()) return SharedLibraryPtr();
        Registry& inst=instance();
        pluginName=inst.getPluginName(pluginName);

        if(inst._registedLibs[pluginName].is<SharedLibraryPtr>())
            return inst._registedLibs.Get<SharedLibraryPtr>(pluginName);

        // find out and load the SharedLibrary
        for(std::string dir:inst._libraryFilePath)
        {
            std::string pluginPath=dir+"/libgslam_"+pluginName;
            if(!fileExists(pluginPath)) continue;
            SharedLibraryPtr lib(new SharedLibrary(pluginPath));
            if(lib->isLoaded())
            {
                inst._registedLibs.set(pluginName,lib);
                return lib;
            }
        }

        // find out and load the SharedLibrary
        for(std::string dir:inst._libraryFilePath)
        {
            std::string pluginPath=dir+"/"+pluginName;
            if(!fileExists(pluginPath)) continue;
            SharedLibraryPtr lib(new SharedLibrary(pluginPath));
            if(lib->isLoaded())
            {
                inst._registedLibs.set(pluginName,lib);
                return lib;
            }
        }

        // find out and load the SharedLibrary
        for(std::string dir:inst._libraryFilePath)
        {
            std::string pluginPath=dir+"/lib"+pluginName;
            if(!fileExists(pluginPath)) continue;
            SharedLibraryPtr lib(new SharedLibrary(pluginPath));
            if(lib->isLoaded())
            {
                inst._registedLibs.set(pluginName,lib);
                return lib;
            }
        }
        // failed to find the library
        return SharedLibraryPtr();
    }

    static bool erase(std::string pluginName)
    {
        if(pluginName.empty()) return false;
        Registry& inst=instance();
        pluginName=inst.getPluginName(pluginName);
        inst._registedLibs.set(pluginName,Svar());
        return true;
    }

    std::set<std::string>& paths(){return _libraryFilePath;}
protected:
    static bool fileExists(const std::string& filename)
    {
        return access( filename.c_str(), 0 ) == 0;
    }

    static void convertStringPathIntoFilePathList(const std::string& paths,FilePathList& filepath)
    {
    #if defined(WIN32) && !defined(__CYGWIN__)
        char delimitor = ';';
        if(paths.find(delimitor)==std::string::npos) delimitor=':';
    #else
        char delimitor = ':';
        if(paths.find(delimitor)==std::string::npos) delimitor=';';
    #endif

        if (!paths.empty())
        {
            std::string::size_type start = 0;
            std::string::size_type end;
            while ((end = paths.find_first_of(delimitor,start))!=std::string::npos)
            {
                filepath.insert(std::string(paths,start,end-start));
                start = end+1;
            }

            std::string lastPath(paths,start,std::string::npos);
            if (!lastPath.empty())
                filepath.insert(lastPath);
        }

    }
    std::string getPluginName(std::string pluginName)
    {
        std::string suffix;
        size_t idx=pluginName.find_last_of('.');
        if(idx!=std::string::npos)
        suffix=pluginName.substr(idx);
        if(suffix!=SharedLibrary::suffix())
        {
            pluginName+=SharedLibrary::suffix();
        }

        std::string folder=getFolderPath(pluginName);
        pluginName=getFileName(pluginName);
        if(folder.size()){
            _libraryFilePath.insert(folder);
        }
        return pluginName;
    }

    void updatePaths()
    {
        _libraryFilePath.clear();

        char** argv=svar.get<char**>("argv",nullptr);
        if(argv)
        {
            _libraryFilePath.insert(getFolderPath(argv[0]));//application folder
        }
        _libraryFilePath.insert(".");

        FilePathList envs={"GSLAM_LIBRARY_PATH","GSLAM_LD_LIBRARY_PATH"};
        FilePathList paths;
#ifdef __linux

#if defined(__ia64__) || defined(__x86_64__)
        paths.insert("/usr/lib/:/usr/lib64/:/usr/local/lib/:/usr/local/lib64/");
#else
        paths.insert("/usr/lib/:/usr/local/lib/");
#endif
        envs.insert("LD_LIBRARY_PATH");
#elif defined(__CYGWIN__)
        envs.insert("PATH");
        paths.insert("/usr/bin/:/usr/local/bin/");
#elif defined(WIN32)
        envs.insert("PATH");
#endif
        for(std::string env:envs)
        {
            std::string ptr=svar.GetString(env,"");
            if(!ptr.empty())
                convertStringPathIntoFilePathList(ptr,_libraryFilePath);
        }
        for(std::string ptr:paths)
            convertStringPathIntoFilePathList(ptr,_libraryFilePath);
    }

    inline std::string getFolderPath(const std::string& path) {
      auto idx = std::string::npos;
      if ((idx = path.find_last_of('/')) == std::string::npos)
        idx = path.find_last_of('\\');
      if (idx != std::string::npos)
        return path.substr(0, idx);
      else
        return "";
    }

    inline std::string getBaseName(const std::string& path) {
      std::string filename = getFileName(path);
      auto idx = filename.find_last_of('.');
      if (idx == std::string::npos)
        return filename;
      else
        return filename.substr(0, idx);
    }

    inline std::string getFileName(const std::string& path) {
      auto idx = std::string::npos;
      if ((idx = path.find_last_of('/')) == std::string::npos)
        idx = path.find_last_of('\\');
      if (idx != std::string::npos)
        return path.substr(idx + 1);
      else
        return path;
    }



    std::set<std::string>               _libraryFilePath;// where to search?
    Svar                                _registedLibs;   // already loaded
};

}

#endif
