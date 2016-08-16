#ifndef __OSLAM_H__
#define __OSLAM_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#define USE_DEFINED_OPERATOR_NEW

#ifdef USE_DEFINED_OPERATOR_NEW

inline void *operator new(size_t s)
{
    void *p = malloc(s);
    printf("operator_new:      a=%12llx, s=%8lld\n", p, s);
    return p;
}

inline void operator delete(void *p)
{
    printf("operator_delete:   a=%12llx\n", p);
    free(p);
}


inline void *operator new[](size_t s)
{
    void *p = malloc(s);
    printf("operator_new[]:    a=%12llx, s=%8lld\n", p, s);
    return p;
}

inline void operator delete[](void *p)
{
    printf("operator_delete[]: a=%llx\n", p);
    free(p);
}


#endif // end of USE_DEFINED_OPERATOR_NEW



#endif // end of __OSLAM_H__

