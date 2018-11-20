#include <unordered_map>

#ifdef __linux
#include <unistd.h>
#endif

namespace GSLAM{

class MemoryMetric{
public:
    MemoryMetric():_usage(0),_enabled(false),_shouldIgnore(false){}
    ~MemoryMetric(){_enabled=false;}

    static MemoryMetric& instanceCPU(){
        static MemoryMetric inst;
        return inst;
    }

    bool isEnabled()const{return _enabled;}

    void enable(){_enabled=true;}

    size_t usage()const{
        if(_enabled)
            return _usage;
        else return processUsage();
    }

    size_t count()const{return _allocated_sizes.size();}

    void AddAllocation(void* ptr,size_t size){
        if(!_enabled) return;
        if(_shouldIgnore) return;

        {
            std::unique_lock<std::mutex> lock(_mutex);
            _shouldIgnore=true;
            _allocated_sizes[ptr] = size;
            _usage+=size;
            _shouldIgnore=false;
        }
    }

    void FreeAllocation(void* ptr){
        if(!_enabled) return;
        if(_shouldIgnore) return;
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _shouldIgnore=true;
            auto it=_allocated_sizes.find(ptr);
            if(it!=_allocated_sizes.end())
            {
                _usage-=it->second;
                _allocated_sizes.erase(it);
            }
            _shouldIgnore=false;
        }
    }

    operator bool(){return isEnabled();}

    static size_t processUsage(){
#ifdef __linux
        char file_name[64]={0};
        FILE *fd;
        char line_buff[512]={0};
        sprintf(file_name,"/proc/%d/status",getpid());

        fd =fopen(file_name,"r");
        if(nullptr == fd){
            return 0;
        }

        char name[64];
        int vmrss;
        const int VMRSS_LINE=17;
        for (int i=0; i<VMRSS_LINE-1;i++){
            fgets(line_buff,sizeof(line_buff),fd);
        }

        fgets(line_buff,sizeof(line_buff),fd);
        sscanf(line_buff,"%s %d",name,&vmrss);
        fclose(fd);
        return vmrss;
#else
        return 0;
#endif
    }

private:
    std::unordered_map<void*,size_t>  _allocated_sizes;
    std::mutex _mutex;
    size_t     _usage;
    bool       _enabled,_shouldIgnore;
};

}
