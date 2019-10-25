#ifndef GSLAM_DISPLAY_H
#define GSLAM_DISPLAY_H

#include "Messenger.h"

namespace GSLAM {



class DisplayProperty{
public:
  DisplayProperty(const Svar& value,
                  const Svar& callback=Svar(),
                  const std::string& describe="")
    :_value(value),_callback(callback),_describe(describe){}

  void refresh(){if(_refresh.isFunction()) _refresh();}

  void notify(){if(_callback.isFunction()) _callback();}

  Svar _value,_callback,_refresh;
  std::string  _describe;
};

}
#endif
