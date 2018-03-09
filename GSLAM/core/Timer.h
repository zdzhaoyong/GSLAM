#ifndef GSLAM_TIMER_H
#define GSLAM_TIMER_H
#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdint.h>
#include <vector>
#include <stack>
#include <map>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>

#define timer GSLAM::Timer::instance()
#define SCOPE_TIMER GSLAM::ScopedTimer scopedTimer(__func__);

namespace GSLAM {

class TicToc
{
public:
    TicToc(){Tic();}

    void Tic(){
        _tmBegin=std::chrono::high_resolution_clock::now();
    }
    double Toc(){
        return  std::chrono::duration<double>(std::chrono::high_resolution_clock::now()-_tmBegin).count();
    }

    double Tac(){return Toc();}

    static double timestamp(){
        return std::chrono::duration<double>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }

protected:
std::chrono::high_resolution_clock::time_point _tmBegin;
};

class Rate
{
public:
    Rate(double frequency=1.0)
    {
        _cycle=1./frequency;
        _tmBegin=std::chrono::system_clock::now();
    }

    static void sleep(double seconds)
    {
        if(seconds>0)
            std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    }

    void sleep(){
        std::chrono::system_clock::time_point now=std::chrono::system_clock::now();
        sleep(_cycle-std::chrono::duration<double>(now-_tmBegin).count());
        _tmBegin=now;
    }

    double                  _cycle;
    std::chrono::system_clock::time_point _tmBegin;
};

class Timer : public TicToc
{
private:
    bool		m_enabled;

    //! Data of all the calls:
    struct TCallData
    {
        TCallData();

        size_t n_calls;
        double min_t,max_t,mean_t;
        std::stack<double,std::vector<double> >   open_calls;
        bool has_time_units;
    };

    std::map<std::string,TCallData>  m_data;
    mutable std::mutex               mMutex;

    void do_enter( const char *func_name );
    double do_leave( const char *func_name );

public:
    Timer(bool enabled=true):m_enabled(enabled){Tic();}
    ~Timer();

    static Timer& instance(){
        static std::shared_ptr<Timer> globalTimer(new Timer());
        return *globalTimer;
    }

    void enable(bool enabled = true) { m_enabled = enabled; }
    void disable() { m_enabled = false; }

    /** Start of a named section \sa enter */
    inline void enter( const char *func_name ) {
        if (m_enabled)
            do_enter(func_name);
    }

    /** End of a named section \return The ellapsed time, in seconds or 0 if disabled. \sa enter */
    inline double leave( const char *func_name ) {
        return m_enabled ? do_leave(func_name) : 0;
    }

    /** Return the mean execution time of the given "section", or 0 if it hasn't ever been called "enter" with that section name */
    double getMeanTime(const std::string &name) const;
    std::string getStatsAsText(const size_t column_width=80) const; //!< Dump all stats to a multi-line text string. \sa dumpAllStats, saveToCVSFile
    void dumpAllStats(const size_t column_width=80) const; //!< Dump all stats through the CDebugOutputCapable interface. \sa getStatsAsText, saveToCVSFile

};


class ScopedTimer
{
public:
    ScopedTimer(const char* func_name):_func_name(func_name){timer.enter(func_name);}
    ~ScopedTimer(){timer.leave(_func_name);}
    const char* _func_name;
};
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline Timer::TCallData::TCallData() :
    n_calls	(0),
    min_t	(0),
    max_t	(0),
    mean_t	(0),
    has_time_units(true)
{

}

inline Timer::~Timer()
{
    dumpAllStats();
    m_data.clear();
}


inline void Timer::do_enter(const char *func_name)
{
    using namespace std;
    const string  s = func_name;
    TCallData *d=NULL;
    {
        std::unique_lock<std::mutex> lock(mMutex);
        map<string,TCallData>::iterator it=m_data.find(s);
        if(it==m_data.end())
        {
            it=m_data.insert(make_pair(s,TCallData())).first;
        }
        d =& it->second;
    }

    d->n_calls++;
    d->open_calls.push(0);  // Dummy value, it'll be written below
    d->open_calls.top() = Tac(); // to avoid possible delays.
}

inline double Timer::do_leave(const char *func_name)
{
    using namespace std;
    const double tim = Tac();

    const string  s = func_name;
    TCallData *d=NULL;
    {
        std::unique_lock<std::mutex> lock(mMutex);
        d = &m_data[s];
    }
    if (!d->open_calls.empty())
    {
        const double At = tim - d->open_calls.top();
        d->open_calls.pop();

        d->mean_t+=At;
        if (d->n_calls==1)
        {
            d->min_t= At;
            d->max_t= At;
        }
        else
        {
            if (d->min_t>At) d->min_t = At;
            if (d->max_t<At) d->max_t = At;
        }
        return At;
    }
    else return 0; // This shouldn't happen!
}

inline double Timer::getMeanTime(const std::string &name)  const
{
    using namespace std;
    std::unique_lock<std::mutex> lock(mMutex);
    map<string,TCallData>::const_iterator it = m_data.find(name);
    if (it==m_data.end())
         return 0;
    else return it->second.n_calls ? it->second.mean_t/it->second.n_calls : 0;
}

inline std::string unitsFormat(const double val,int nDecimalDigits, bool middle_space)
{
    using namespace std;
    char	prefix;
    double	mult;

    if (val>=1e12)
        {mult=1e-12; prefix='T';}
    else if (val>=1e9)
        {mult=1e-9; prefix='G';}
    else if (val>=1e6)
        {mult=1e-6; prefix='M';}
    else if (val>=1e3)
        {mult=1e-3; prefix='K';}
    else if (val>=1)
        {mult=1; prefix=' ';}
    else if (val>=1e-3)
        {mult=1e+3; prefix='m';}
    else if (val>=1e-6)
        {mult=1e+6; prefix='u';}
    else if (val>=1e-9)
        {mult=1e+9; prefix='n';}
    else if (val>=1e-12)
        {mult=1e+12; prefix='p';}
    else
        {mult=0; prefix='p';}

    ostringstream ost;
    ost<<setw(5) <<setiosflags(ios::fixed) <<setiosflags(ios::right)
      << setprecision(1)<<(val*mult);

    return ost.str()+char(prefix);
}

inline std::string rightPad(const std::string &str, const size_t total_len, bool truncate_if_larger)
{
    std::string r = str;
    if (r.size()<total_len || truncate_if_larger)
        r.resize(total_len,' ');
    return r;
}

inline std::string  aux_format_string_multilines(const std::string &s, const size_t len)
{
    std::string ret;

    for (size_t p=0;p<s.size();p+=len)
    {
        ret+=rightPad(s.c_str()+p,len,true);
        if (p+len<s.size())
            ret+="\n";
    }
    return ret;
}

inline std::string Timer::getStatsAsText(const size_t column_width)  const
{
    std::unique_lock<std::mutex> lock(mMutex);
    using namespace std;
    ostringstream ost;
    ost<<"------------------------------------  Timer report ------------------------------------\n";
    ost<<"           FUNCTION                       #CALLS  MIN.T  MEAN.T  MAX.T  TOTAL \n";
    ost<<"---------------------------------------------------------------------------------------\n";
    for (map<string,TCallData>::const_iterator i=m_data.begin();i!=m_data.end();++i)
    {
        const string sMinT   = unitsFormat(i->second.min_t,1,false);
        const string sMaxT   = unitsFormat(i->second.max_t,1,false);
        const string sTotalT = unitsFormat(i->second.mean_t,1,false);
        const string sMeanT  = unitsFormat(i->second.n_calls ? i->second.mean_t/i->second.n_calls : 0,1,false);

        ost << aux_format_string_multilines(i->first,39)
            << " " << setw(6) << setiosflags(ios::right) << i->second.n_calls <<"  "
            << sMinT << "s " << sMeanT << "s " << sMaxT << "s "
            << sTotalT << "s\n";
    }

    ost<<"--------------------------------- End of Timer report ---------------------------------\n";

    return ost.str();
}

inline void Timer::dumpAllStats(const size_t  column_width) const
{
    using namespace std;
    if(!m_data.size()||!m_enabled) return;
    string s = getStatsAsText(column_width);
    cout<<endl<<s<<endl;
}


}
#endif // TIME_H
