#ifndef VECPARAMENT_H
#define VECPARAMENT_H

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

template <typename VarType>
class VecParament
{
public:
    VecParament(std::string str="")
    {
        fromString(str);
    }
    VecParament(const std::vector<VarType>& vars):data(vars){}

    VecParament(int n, const VarType& defV) : data(n, defV)
    {
    }

    size_t size(){return data.size();}

    VarType& operator [](size_t idx){return data[idx];}

    bool fromString(std::string str)
    {
        data.clear();
        if(str.empty()) return false;
        std::string::size_type start,stop;
        start=str.find('[');
        stop =str.find(']');
        if(start==std::string::npos||stop==std::string::npos||start>=stop)
        {
        }
        else
            str=str.substr(start+1,stop-1);
        bool hasDot=(str.find(',')!=std::string::npos);
        char alige;
        if(hasDot)
        alige=',';
        else
        {
            alige=' ';
        }
        std::string str_num;
        while (str.size())
        {
            std::string::size_type n=str.find(alige);
            if(n!=std::string::npos)
            {
                str_num=str.substr(0,n);
                str=str.substr(n+1);
            }
            else
            {
                str_num=str;
                str="";
            }
            if(str_num==""||str_num==" ") continue;

            std::istringstream iss(str_num);
            VarType x;
            iss>>x;
            data.push_back(x);
        }
        return true;
    }

    std::string toString()
    {
        std::ostringstream ost;
        ost<<"[";
        for(size_t i=0;i<data.size()-1;i++)
            ost<<data.at(i)<<" ";
        ost<<data.at(size()-1)<<"]";
        return ost.str();
    }

    friend inline std::istream& operator >>(std::istream& is,VecParament & p)
    {
        std::string line;
        std::getline(is,line);
        p.fromString(line);
        return is;
    }

    friend inline std::ostream& operator <<(std::ostream& os,VecParament & p)
    {
        os<<p.toString();
        return os;
    }

    std::vector<VarType> data;
};

#endif // VECPARAMENT_H
