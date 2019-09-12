#ifndef GSLAM_DISPLAY_H
#define GSLAM_DISPLAY_H

#include "Messenger.h"

namespace GSLAM {

/// Class Topic is used by display panel to choose a Messenger Topic
class Topic{
public:
  Topic(const Svar& cls=SvarClass::instance<Svar>(),
        const std::string& name=""):
    _class(cls),_name(name) {}

  std::string name()const{return _name;}
  Svar        type()const{return _class;}

  Svar _class;
  std::string _name;
};

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
