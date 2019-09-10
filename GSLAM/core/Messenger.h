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
// Messenger: A light-weight, efficient, thread-safe message publish and
// subscribe tool similar with ROS, a popular robot operating system
// The tool has the following features:
// * Header only based on c++11, no extra dependency, makes it portable.
// * Thread safe and support multi-thread condition notify mode by setting the queue size.
// * Able to transfer any classes efficiently, including ROS defined messages, which means it can replace ROS messagging or work with it.

#ifndef GSLAM_MESSENGER_H
#define GSLAM_MESSENGER_H


#include <signal.h>
#include <functional>
#include <iostream>
#include <iomanip>
#include <set>
#include <map>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>
#include <atomic>
#include <condition_variable>
#include <future>
#include <queue>
#include <thread>
#include "Svar.h"

#ifdef messenger
#undef messenger
#endif
#define messenger GSLAM::Messenger::instance()

namespace GSLAM {

#ifndef DOXYGEN_IGNORE_INTERNAL
namespace msg{

// A simple threadpool implementation.
class ThreadPool {
public:
    // All the threads are created upon construction.
    explicit ThreadPool(const int num_threads) : stop(false) {
        workerRunning.resize(num_threads, 0);

        for (size_t i = 0; i < num_threads; ++i) {
            workers.emplace_back(&ThreadPool::threadFunc, this, i);
        }
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread& worker : workers) worker.join();
    }

    void threadFunc(int tid) {
        for (;;) {
            std::function<void()> task;

            {
                std::unique_lock<std::mutex> lock(this->queue_mutex);
                this->condition.wait(
                            lock, [this] { return this->stop || !this->tasks.empty(); });
                if (this->stop && this->tasks.empty()) return;

                task = std::move(this->tasks.front());
                this->tasks.pop();
            }

            workerRunning[tid] = 1;
            task();
            workerRunning[tid] = 0;
        }
    }

    // Adds a task to the threadpool.
    template <class F, class... Args>
    auto Add(F&& f, Args&&... args)
    -> std::future<typename std::result_of<F(Args...)>::type>;

    size_t taskNumLeft() { return tasks.size(); }

    void   popTask(){
        std::unique_lock<std::mutex> lock(queue_mutex);
        tasks.pop();
    }

    int isRunning(void) {
        int nr = 0, nt = workerRunning.size();
        for(int i=0; i<nt; i++) nr += workerRunning[i];

        if( nr == 0 ) return 0;
        else return 1;
    }

private:
    // Keep track of threads so we can join them
    std::vector<std::thread> workers;
    // work running status (0 - idle, 1 - working)
    std::vector<int> workerRunning;
    // The task queue
    std::queue<std::function<void()> > tasks;

    // Synchronization
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

// add new work item to the pool
template <class F, class... Args>
auto ThreadPool::Add(F&& f, Args&&... args)
-> std::future<typename std::result_of<F(Args...)>::type> {
    using return_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<return_type()> >(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...));

    std::future<return_type> res = task->get_future();
    {
        std::unique_lock<std::mutex> lock(queue_mutex);

        // don't allow enqueueing after stopping the pool
        if (stop)
            std::cerr << "The ThreadPool object has been destroyed! Cannot add more "
                         "tasks to the ThreadPool!";

        tasks.emplace([task]() { (*task)(); });
    }
    condition.notify_one();
    return res;
}

} // end of namespace detail
#endif

class Messenger;
class Publisher;
class PubSubSpace;

class Subscriber {
public:
    Subscriber() {}
    Subscriber(const std::string& topic, const SvarFunction& callback, size_t queue_size = 0)
        : impl_(new Impl(topic,callback,queue_size)) {}
    ~Subscriber() {
        if(impl_.use_count()==2)
        {
            shutdown();
        }
    }

    /**
   * \brief Unsubscribe the callback associated with this Subscriber
   *
   * This method usually does not need to be explicitly called, as automatic
   * shutdown happens when
   * all copies of this Subscriber go out of scope
   *
   * This method overrides the automatic reference counted unsubscribe, and
   * immediately
   * unsubscribes the callback associated with this Subscriber
   */
    void shutdown() ;

    std::string getTopic() const {
        if (impl_) return impl_->topic_;
        return "";
    }

    std::string getTypeName() const {
        if (impl_) return impl_->type_.as<SvarClass>().name();
        return "";
    }

    /**
   * \brief Returns the number of publishers this subscriber is connected to
   */
    uint32_t getNumPublishers() const ;

    operator void*() const { return (impl_) ? (void*)1 : (void*)0; }

    bool operator<(const Subscriber& rhs) const { return impl_ < rhs.impl_; }

    bool operator==(const Subscriber& rhs) const { return impl_ == rhs.impl_; }

    bool operator!=(const Subscriber& rhs) const { return impl_ != rhs.impl_; }

protected:
    friend class Messenger;
    friend class Publisher;
    struct Impl {
        Impl(const std::string& topic, const SvarFunction& callback, size_t queue_size = 0)
            : topic_(topic),
              callback_(callback),
              unsubscribed_(false),
              queue_size_(queue_size),
              workthread_(queue_size ? new msg::ThreadPool(1) : nullptr) {
            assert(callback.arg_types.size()==1);
            type_=callback.arg_types[0];
        }

        ~Impl(){workthread_.reset();}

        void publish(Svar message) const {
            if (unsubscribed_) return;
            if (workthread_ ){
                if(workthread_->taskNumLeft() >= queue_size_)
                    workthread_->popTask();
                workthread_->Add([this, message]() {
                    if (unsubscribed_) return;
                    callback_.call(message);
                });
                return;
            }
            callback_.call(message);
        }

        std::string topic_;
        Svar        type_;
        SvarFunction callback_;
        bool unsubscribed_;
        std::shared_ptr<PubSubSpace> space_;
        size_t queue_size_;
        std::shared_ptr<msg::ThreadPool> workthread_;
    };

    Subscriber(std::shared_ptr<Subscriber::Impl> impl) : impl_(impl) {}

    virtual void publish(const Svar& message) const {
        if (!impl_) return;
        impl_->publish(message);
    }
    std::string key()const{return getTopic()+"#"+getTypeName();}

    std::shared_ptr<Impl> impl_;
};

class Publisher {
public:
    Publisher() {}
    Publisher(const std::string& topic, const Svar& type,
              size_t queue_size = 0)
        :impl_(new Impl(topic,type,queue_size)){}

    virtual ~Publisher() {
        if(impl_.use_count()==2)
        {
            shutdown();
        }
    }

    /**
   * \brief Publish a message without a copy!
   */
    void publish(const Svar& message) const;

    /**
   * \brief Shutdown the advertisement associated with this Publisher
   *
   * This method usually does not need to be explicitly called, as automatic
   * shutdown happens when
   * all copies of this Publisher go out of scope
   *
   * This method overrides the automatic reference counted unadvertise, and does
   * so immediately.
   * \note Note that if multiple advertisements were made through
   * NodeHandle::advertise(), this will
   * only remove the one associated with this Publisher
   */
    void shutdown() ;

    /**
   * \brief Returns the topic that this Publisher will publish on.
   */
    std::string getTopic() const {
        if (impl_) return impl_->topic_;
        return "";
    }

    /**
   * \brief Returns the topic that this Publisher will publish on.
   */
    std::string getTypeName() const {
        if (impl_) return impl_->type_.as<SvarClass>().name();
        return "";
    }

    /**
   * \brief Returns the number of subscribers that are currently connected to
   * this Publisher
   */
    uint32_t getNumSubscribers() const;

    operator void*() const { return (impl_) ? (void*)1 : (void*)0; }

    bool operator<(const Publisher& rhs) const { return impl_ < rhs.impl_; }

    bool operator==(const Publisher& rhs) const { return impl_ == rhs.impl_; }

    bool operator!=(const Publisher& rhs) const { return impl_ != rhs.impl_; }


protected:
    friend class Messenger;
    friend class Subscriber;

    struct Impl {
        Impl(const std::string& topic, const Svar& type,
             size_t queue_size = 0)
            : topic_(topic),
              type_(type),
              queue_size_(queue_size),
              workthread_(queue_size ? new msg::ThreadPool(1) : nullptr) {}

        std::string topic_;
        Svar        type_;
        std::shared_ptr<PubSubSpace> space_;
        size_t queue_size_;
        std::shared_ptr<msg::ThreadPool> workthread_;
        std::mutex mutex_;
    };
    Publisher(Impl* implement) : impl_(implement) {}
    std::string key()const{return getTopic()+"#"+getTypeName();}


    std::shared_ptr<Impl> impl_;
};

#ifndef DOXYGEN_IGNORE_INTERNAL
struct PubSubSpace
{
    std::mutex mtx_;
    std::set<Subscriber> subs_;
    std::set<Publisher>  pubs_;
};
#endif

/**
 @brief A tiny class implemented ROS like Pub/Sub messaging.

Messenger: A light-weight, efficient, thread-safe message publish and
 subscribe tool similar with ROS, a popular robot operating system.

The tool has the following features:
- Header only based on c++11, no extra dependency, makes it portable.
- Thread safe and support multi-thread condition notify mode by setting the queue size.
- Able to transfer any classes efficiently, including ROS defined messages, which means it can replace ROS messagging or work with it.

@code
// example.cpp
#include <GSLAM/core/Messenger.h>

using namespace GSLAM;

int main(int argc,char** argv)
{
    Subscriber sub=messenger.subscribe("topic_name",[](std::string msg){
        std::cerr<<"Received string msg "<<msg<<std::endl;
    });

    // publish message without creating a Publisher
    messenger.publish("topic_name","hello world!");

    Publisher pub=messenger.advertise<std::string>("topic_name");
    pub.publish("hello Messenger!");

    return 0;
}
@endcode

@warning The Subscriber will auto shutdown, please hold the Subscriber instance until you want to unsubscribe.
 */
class Messenger {
public:

    Messenger():d(new Data()){
        d->pubNewSub=advertise<Publisher>("messenger/newsub");
        d->pubNewPub=advertise<Subscriber>("messenger/newpub");
    }
    virtual ~Messenger() {}

    /// @return The Messenger instance (messenger)
    static Messenger& instance(){
        static std::shared_ptr<Messenger> inst(new Messenger());
        return *inst;
    }

    /// @brief Create a Publisher
    /// @param topic the topic name
    /// @param queue_size when queue_size>0, messages will be sent in another thread
    /// @return the Publisher created
    template <class M>
    Publisher advertise(const std::string& topic, uint32_t queue_size = 0) {
        Publisher pub(topic, SvarClass::instance<M>(),queue_size);
        join(pub);
        d->pubNewPub.publish(pub);
        return pub;
    }

    /// @brief publish a message without creating a publisher
    /// @param topic the topic name
    /// @param message the message will be sent in this thread
    template <class M>
    void publish(const std::string& topic,const M& message){
        std::unique_lock<std::mutex> lockSpaces(d->mutex_);
        std::shared_ptr<PubSubSpace> space=d->spaces_[topic];
        if(!space) return;
        if(space->subs_.empty()) return;
        std::unique_lock<std::mutex> lockSpace(space->mtx_);
        for(Subscriber sub:space->subs_)
            sub.publish(message);
    }

    /// @brief subscribe a topic with callback function
    /// @param topic the topic name
    /// @param queue_size when queue_size>0, messages will be handled in a separate thread
    /// @param callback the callback function used to handle the messages
    Subscriber subscribe(
            const std::string& topic, uint32_t queue_size,
            SvarFunction callback) {
        Subscriber sub(topic, callback,queue_size);
        join(sub);
        d->pubNewSub.publish(sub);
        return sub;
    }

    /// @brief subscribe a topic with callback function
    Subscriber subscribe(const std::string& topic, SvarFunction callback,
                         uint32_t queue_size=0){
        Subscriber sub(topic, callback,queue_size);
        join(sub);
        d->pubNewSub.publish(sub);
        return sub;
    }

    /// @brief subscribe a topic with callback member function
    template <class T, class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void (T::*fp)(const std::shared_ptr<M>&), T* obj) {
        std::function<void(const std::shared_ptr<M>&)> cbk =
                std::bind(fp, obj, std::placeholders::_1);
        return subscribe(topic, queue_size, cbk);
    }

    /// @brief subscribe a topic with callback function
    template <class M>
    Subscriber subscribe(const std::string& topic, int queue_size,
                         void (*fp)(const M&)){
        return subscribe(topic,queue_size,SvarFunction(fp));
    }

    /// @brief subscribe a topic with callback member function
    template <class T, class M>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size,
                         void (T::*fp)(const M&), T* obj) {
        std::function<void (const M&)> func=std::bind(fp, obj, std::placeholders::_1);
        return subscribe(topic, queue_size, [func](const M& m){return func(m);});
    }

    /// @brief get all publishers
    std::vector<Publisher> getPublishers()const{
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::vector<Publisher> pubs;
        for(auto it1:d->spaces_)
        {
            std::shared_ptr<PubSubSpace> space=it1.second;
            std::unique_lock<std::mutex> lock(space->mtx_);
            pubs.insert(pubs.end(),space->pubs_.begin(),space->pubs_.end());
        }
        return pubs;
    }

    /// @brief get all subscribers
    std::vector<Subscriber> getSubscribers()const{
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::vector<Subscriber> subs;
        for(auto it1:d->spaces_)
        {
            std::shared_ptr<PubSubSpace> space=it1.second;
            std::unique_lock<std::mutex> lock(space->mtx_);
            subs.insert(subs.end(),space->subs_.begin(),space->subs_.end());
        }
        return subs;
    }

    /// @brief list all publishers and subscribers
    std::string introduction(int width=80)const{
        if(getPublishers().size()+getSubscribers().size()==0)
            return "";
        std::stringstream sst;
        sst<<"Publisher and Subscriber lists:\n";
        sst<<printTable({{width/5-1,"Type"},
                         {width*2/5-1,"Topic"},
                         {width*2/5,"Payload"}});

        for(int i=0;i<width;i++)
            sst<<"-";
        sst<<std::endl;

        std::unique_lock<std::mutex> lock(d->mutex_);
        for(auto it1:d->spaces_)
        {
            std::shared_ptr<PubSubSpace> space=it1.second;
            std::unique_lock<std::mutex> lock(space->mtx_);
            if(!space->pubs_.empty())
                sst<<printTable({{width/5-1,"Publisher*"+std::to_string(space->pubs_.size())},
                                 {width*2/5-1,space->pubs_.begin()->getTopic()},
                                 {width*2/5,space->pubs_.begin()->getTypeName()}});

            if(!space->subs_.empty())
                sst<<printTable({{width/5-1,"Subscriber*"+std::to_string(space->subs_.size())},
                                 {width*2/5-1,space->subs_.begin()->getTopic()},
                                 {width*2/5,space->subs_.begin()->getTypeName()}});
        }
        return sst.str();
    }

    /// Let the Publisher join this Messenger space
    void join(const Publisher& pub){
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::shared_ptr<PubSubSpace>& space=d->spaces_[pub.getTopic()];
        if(!space) space=std::shared_ptr<PubSubSpace>(new PubSubSpace());
        std::unique_lock<std::mutex> lock1(space->mtx_);
        space->pubs_.insert(pub);
        pub.impl_->space_=space;
    }

    /// Let the Subscriber join this Messenger space
    void join(const Subscriber& sub){
        std::unique_lock<std::mutex> lock(d->mutex_);
        std::shared_ptr<PubSubSpace>& space=d->spaces_[sub.getTopic()];
        if(!space) space=std::shared_ptr<PubSubSpace>(new PubSubSpace());
        std::unique_lock<std::mutex> lock1(space->mtx_);
        space->subs_.insert(sub);
        sub.impl_->space_=space;
    }

    /// Let all publishers and subscribers left the old space
    void join(Messenger another){
        for(const Publisher& pub:another.getPublishers()){
            join(pub);
        }
        for(const Subscriber& sub:another.getSubscribers()){
            join(sub);
        }
    }

    /// Notify and wait Subscribers to shutdown
    static int exec(){
        std::promise<bool> stopSig;
        Subscriber subStop=instance().subscribe("messenger/stop",0,[&stopSig](bool b){
            stopSig.set_value(true);
        });
        signal(SIGINT, [](int sig){
            instance().publish("messenger/stop",true);
        });
        stopSig.get_future().wait();
        return 0;
    }

private:
    static std::string printTable(std::vector<std::pair<int,std::string> > line){
        std::stringstream sst;
        while(true){
            size_t emptyCount=0;
            for(auto& it:line){
                size_t width=it.first;
                std::string& str=it.second;
                if(str.size()<=width){
                    sst<< std::setw(width)
                       <<std::setiosflags(std::ios::left)
                      <<str<<" ";
                    str.clear();
                    emptyCount++;
                }else{
                    sst<<str.substr(0,width)<<" ";
                    str=str.substr(width);
                }
            }
            sst<<std::endl;
            if(emptyCount==line.size()) break;
        }
        return sst.str();
    }

    struct Data{
        std::mutex mutex_;
        std::unordered_map<std::string,std::shared_ptr<PubSubSpace>> spaces_;
        Publisher  pubNewPub,pubNewSub;
    };
    std::shared_ptr<Data> d;
};

inline void Publisher::shutdown()
{
    if (!impl_) return;
    auto space=impl_->space_;
    std::unique_lock<std::mutex> lock(space->mtx_);
    auto it=space->pubs_.find(*this);
    impl_.reset();
    space->pubs_.erase(it);
}

inline uint32_t Publisher::getNumSubscribers() const {
    if (impl_) return impl_->space_->subs_.size();
    return 0;
}

inline uint32_t Subscriber::getNumPublishers() const {
    if (!impl_) return 0;
    return impl_->space_->pubs_.size();
}

inline void Subscriber::shutdown()
{
    if (!impl_) return;

    impl_->unsubscribed_ = true;
    auto space=impl_->space_;
    std::unique_lock<std::mutex> lock(space->mtx_);
    auto it=space->subs_.find(*this);
    impl_.reset();
    space->subs_.erase(it);
}

inline void Publisher::publish(const Svar& message) const {
    if (!impl_) return;

    std::set<Subscriber> subscribers;
    {
        std::unique_lock<std::mutex> lock(impl_->space_->mtx_);
        subscribers = impl_->space_->subs_;
    }

    if (subscribers.empty()) {
        return;
    }

    if (impl_->workthread_ ) {

        if(impl_->workthread_->taskNumLeft() >= impl_->queue_size_)
            impl_->workthread_->popTask();
        impl_->workthread_->Add([subscribers, message]() {
            for (const Subscriber& s : subscribers) {
                s.publish(message);
            }
        });
        return;
    }

    for (Subscriber s : subscribers) {
        s.publish(message);
    }
}

}  // end of namespace GSLAM

#endif
