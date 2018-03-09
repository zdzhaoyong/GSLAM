#ifndef SPTR_H
#define SPTR_H

#if 1//__cplusplus >= 201103L
#include <memory>

#define SPtr std::shared_ptr
#define WPtr std::weak_ptr

#elif  1
#include <tr1/memory>

#define SPtr std::tr1::shared_ptr
#define WPtr std::tr1::weak_ptr

#elif HAS_BOOST

#include <boost/shared_ptr.hpp>
#define SPtr boost::shared_ptr
#define WPtr boost::weak_ptr

#else
#error ("No implementation of SPtr!")
#endif //USE_TR1

#endif // SPTR_H
