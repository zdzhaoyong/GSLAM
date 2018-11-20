#ifndef GSLAM_CORE_CPUMETRIC_H
#define GSLAM_CORE_CPUMETRIC_H

#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <thread>

#ifdef __unix
#include <unistd.h>

namespace GSLAM{

#define VMRSS_LINE 17
#define VMSIZE_LINE 13
#define PROCESS_ITEM 14

class CPUMetric{
public:
    CPUMetric(size_t pid=process_id())
        : id(pid),last_total(get_cpu_total_occupy()),
          last_proc(get_cpu_proc_occupy(pid))
    {

    }

    float usage(){
        unsigned long total=get_cpu_total_occupy();
        unsigned long proc =get_cpu_proc_occupy(id);
        if(proc-last_proc<100) return last_usage;
        last_usage=std::thread::hardware_concurrency()*100.*(proc-last_proc)/(total-last_total);
        last_proc=proc;
        last_total=total;
        return last_usage;
    }

    static size_t process_id(){
        return getpid();
    }

private:
    typedef struct {
            unsigned long user;
            unsigned long nice;
            unsigned long system;
            unsigned long idle;
    }Total_Cpu_Occupy_t;


    typedef struct {
            unsigned int  pid;
            unsigned long utime;  //user time
            unsigned long stime;  //kernel time
            unsigned long cutime; //all user time
      unsigned long cstime; //all dead time
    }Proc_Cpu_Occupy_t;


//获取第N项开始的指针
static const char* get_items(const char*buffer ,unsigned int item){
	
	const char *p =buffer;

	int len = strlen(buffer);
	int count = 0;
	
	for (int i=0; i<len;i++){
		if (' ' == *p){
			count ++;
			if(count == item -1){
				p++;
				break;
			}
		}
		p++;
	}

	return p;
}

//获取总的CPU时间
static unsigned long get_cpu_total_occupy(){
	
	FILE *fd;
	char buff[1024]={0};
	Total_Cpu_Occupy_t t;

	fd =fopen("/proc/stat","r");
	if (nullptr == fd){
		return 0;
	}
		
	if(!fgets(buff,sizeof(buff),fd)) return 0;
	char name[64]={0};
	sscanf(buff,"%s %ld %ld %ld %ld",name,&t.user,&t.nice,&t.system,&t.idle);
	fclose(fd);
	
	return (t.user + t.nice + t.system + t.idle);
}


//获取进程的CPU时间
static unsigned long get_cpu_proc_occupy(unsigned int pid){
	
	char file_name[64]={0};
	Proc_Cpu_Occupy_t t;
	FILE *fd;
	char line_buff[1024]={0};
	sprintf(file_name,"/proc/%d/stat",pid);
	
	fd = fopen(file_name,"r");
	if(nullptr == fd){
		return 0;
	}
	
	if(!fgets(line_buff,sizeof(line_buff),fd)) return 0;
	
	sscanf(line_buff,"%u",&t.pid);
	const char *q =get_items(line_buff,PROCESS_ITEM);
	sscanf(q,"%ld %ld %ld %ld",&t.utime,&t.stime,&t.cutime,&t.cstime);
	fclose(fd);
	
	return (t.utime + t.stime + t.cutime + t.cstime);
}

size_t        id;
unsigned long last_total=0,last_proc=0;
float         last_usage=0;

};

}
#else
namespace GSLAM{

class CPUMetric{
public:
    CPUMetric(){}

    float usage(){
        return -1;
    }

size_t        id;
unsigned long last_total=0,last_proc=0;
float         last_usage=0;

};

}
#endif
#endif
