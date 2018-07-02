#include "gtest.h"
#include "GSLAM/core/Mutex.h"
#include <GSLAM/core/Svar.h>
#include <GSLAM/core/Timer.h>

void sumFunc(double* begin,double* end){
    for(auto it=begin+1;it<=end;it++)
        *begin+=*it;
}


TEST(Thread,ThreadPool)
{
    std::vector<double> nums;
    nums.resize(svar.GetInt("ThreadPoolTest.Num",1e7));
    double sumCheck=0;
    {
        GSLAM::ScopedTimer tm("ThreadPollTest.SingleThread");
        for(int i=0;i<nums.size();i++) {nums[i]=i;sumCheck+=i;}
    }
    int threadNum=svar.GetInt("ThreadPoolTest.ThreadNum",4);
    int packageSize=nums.size()/threadNum;
    if(packageSize<1) packageSize++;

    if(threadNum>1)
    {
        GSLAM::ScopedTimer tm("ThreadPoolTest.MultiThread");
        GSLAM::ThreadPool threads(threadNum);
        for(int startIdx=0;startIdx<nums.size();startIdx+=packageSize)
        {
            int endIdx  =startIdx+packageSize-1;
            endIdx=endIdx<nums.size()?endIdx:(nums.size()-1);
            threads.Add(sumFunc,&nums[startIdx],&nums[endIdx]);
        }
    }

    double sumThreads=0;
    for(int startIdx=0;startIdx<nums.size();startIdx+=packageSize)
        sumThreads+=nums[startIdx];

    CHECK_EQ(sumCheck,sumThreads);
}
