#ifndef GSLAM_SharedLibrary_INCLUDED
#define GSLAM_SharedLibrary_INCLUDED

#ifdef HAS_PIL0
#include <pil/base/ClassLoader/SharedLibrary.h>
namespace GSLAM {
typedef pi::SharedLibrary SharedLibrary;
}
#else

#include <iostream>
#include "Mutex.h"

#ifdef __linux

#include <dlfcn.h>
// Note: cygwin is missing RTLD_LOCAL, set it to 0
#if defined(__CYGWIN__) && !defined(RTLD_LOCAL)
#define RTLD_LOCAL 0
#endif

#elif defined(_WIN32) || defined(_WIN64)
#include <windows.h>

// A list of annoying macros to #undef.
// Feel free to extend as required.
#undef GetBinaryType
#undef GetShortPathName
#undef GetLongPathName
#undef GetEnvironmentStrings
#undef SetEnvironmentStrings
#undef FreeEnvironmentStrings
#undef FormatMessage
#undef EncryptFile
#undef DecryptFile
#undef CreateMutex
#undef OpenMutex
#undef CreateEvent
#undef OpenEvent
#undef CreateSemaphore
#undef OpenSemaphore
#undef LoadLibrary
#undef GetModuleFileName
#undef CreateProcess
#undef GetCommandLine
#undef GetEnvironmentVariable
#undef SetEnvironmentVariable
#undef ExpandEnvironmentStrings
#undef OutputDebugString
#undef FindResource
#undef UpdateResource
#undef FindAtom
#undef AddAtom
#undef GetSystemDirector
#undef GetTempPath
#undef GetTempFileName
#undef SetCurrentDirectory
#undef GetCurrentDirectory
#undef CreateDirectory
#undef RemoveDirectory
#undef CreateFile
#undef DeleteFile
#undef SearchPath
#undef CopyFile
#undef MoveFile
#undef ReplaceFile
#undef GetComputerName
#undef SetComputerName
#undef GetUserName
#undef LogonUser
#undef GetVersion
#undef GetObject
#endif


namespace GSLAM {
class SharedLibrary
    /// The SharedLibrary class dynamically
    /// loads shared libraries at run-time.
{
    enum Flags
    {
        SHLIB_GLOBAL_IMPL = 1,
        SHLIB_LOCAL_IMPL  = 2
    };
public:
    SharedLibrary():_handle(NULL){}
        /// Creates a SharedLibrary object.

    SharedLibrary(const std::string& path)
    {
    }
        /// Creates a SharedLibrary object and loads a library
        /// from the given path.

    virtual ~SharedLibrary(){}
        /// Destroys the SharedLibrary. The actual library
        /// remains loaded.

    bool load(const std::string& path,int flags=0)
    {

        WriteMutex lock(_mutex);

        if (_handle)
            return false;

#ifdef __linux
        int realFlags = RTLD_LAZY;
        if (flags & SHLIB_LOCAL_IMPL)
            realFlags |= RTLD_LOCAL;
        else
            realFlags |= RTLD_GLOBAL;
        _handle = dlopen(path.c_str(), realFlags);
        if (!_handle)
        {
            const char* err = dlerror();
            std::cerr<<"Can't open file "<<path<<" since "<<err<<std::endl;
            return false;
        }
#else
        DWORD flags(0);

        flags |= LOAD_WITH_ALTERED_SEARCH_PATH;
        _handle = LoadLibraryExA(path.c_str(), 0, flags);
        if (!_handle) return false;
#endif
        _path = path;
    }
        /// Loads a shared library from the given path.
        /// Throws a LibraryAlreadyLoadedException if
        /// a library has already been loaded.
        /// Throws a LibraryLoadException if the library
        /// cannot be loaded.

    void unload()
    {
        WriteMutex lock(_mutex);

        if (_handle)
        {
#ifdef __linux
            dlclose(_handle);
#else
            FreeLibrary((HMODULE) _handle);
#endif
            _handle = 0;
            _path.clear();
        }
    }
        /// Unloads a shared library.

    bool isLoaded() const
    {
        return _handle!=0;
    }
        /// Returns true iff a library has been loaded.

    bool hasSymbol(const std::string& name)
    {
        return getSymbol(name)!=0;
    }
        /// Returns true iff the loaded library contains
        /// a symbol with the given name.

    void* getSymbol(const std::string& name)
    {
        WriteMutex lock(_mutex);

        void* result = 0;
        if (_handle)
        {
#ifdef __linux
            result = dlsym(_handle, name.c_str());
#else

            return (void*) GetProcAddress((HMODULE) _handle, name.c_str());
#endif
        }
        return result;
    }
        /// Returns the address of the symbol with
        /// the given name. For functions, this
        /// is the entry point of the function.
        /// Throws a NotFoundException if the symbol
        /// does not exist.

    const std::string& getPath() const
    {
        return _path;
    }
        /// Returns the path of the library, as
        /// specified in a call to load() or the
        /// constructor.

    static std::string suffix()
    {
#if defined(__APPLE__)
#if defined(_DEBUG)
        return "d.dylib";
#else
        return ".dylib";
#endif
#elif defined(hpux) || defined(_hpux)
#if defined(_DEBUG)
        return "d.sl";
#else
        return ".sl";
#endif
#elif defined(__CYGWIN__)
#if defined(_DEBUG)
        return "d.dll";
#else
        return ".dll";
#endif
#else
#if defined(_DEBUG)
        return "d.so";
#else
        return ".so";
#endif
#endif
    }
        /// Returns the platform-specific filename suffix
        /// for shared libraries (including the period).
        /// In debug mode, the suffix also includes a
        /// "d" to specify the debug version of a library.

private:
    SharedLibrary(const SharedLibrary&);
    SharedLibrary& operator = (const SharedLibrary&);
    MutexRW     _mutex;
    std::string _path;
    void*       _handle;
};


} // namespace pi


#endif // PIL_SharedLibrary_INCLUDED
#endif
