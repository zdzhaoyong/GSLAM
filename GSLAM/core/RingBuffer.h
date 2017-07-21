#ifndef PIL_RINGBUFFER_H
#define PIL_RINGBUFFER_H

template <typename BufType=unsigned char>
class RingBuffer
{
    RingBuffer(int size=1024)
        :data(0),dataSize(size),readIdx(0),writeIdx(0)
    {
        data=new BufType[size];
    }

    ~RingBuffer()
    {
        if(data) delete[] data;
    }

    int total()const
    {
        return dataSize;
    }

    int used()const
    {
        int tmp=writeIdx-readIdx;
        if(tmp<0) tmp+=dataSize;
        return tmp;
    }

    int left()
    {
        int tmp=readIdx-writeIdx;
        if(tmp<0) tmp+=dataSize;
        return tmp;
    }

    bool write(BufType* buf,int length=1)
    {
        if(left()<length||length<=0) return false;
        for(int i=0;i<length;i++)
        {
            if(writeIdx>=dataSize) writeIdx-=dataSize;
            data[writeIdx++]=buf[i];
        }
        return true;
    }

    bool read(BufType* buf,int length=1)
    {
        if(used()<length||length<=0) return false;
        for(int i=0;i<length;i++)
        {
            if(readIdx>=dataSize) readIdx-=dataSize;
            buf[i]=data[readIdx++];
        }
        return true;
    }

private:
    BufType* data;
    int      dataSize,readIdx,writeIdx;
};
#endif // PIL_RINGBUFFER_H
