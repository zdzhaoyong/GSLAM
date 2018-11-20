#include "MemoryMetric.h"

#include <GSLAM/core/SharedLibrary.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <mutex>
#include <dlfcn.h>


#ifdef ANDROID
#define MALLOC_SUFFIX
#define FREE_SUFFIX
#define REALLOC_SUFFIX
#define CALLOC_SUFFIX
#elif __linux
#define MALLOC_SUFFIX throw()
#define FREE_SUFFIX throw()
#define REALLOC_SUFFIX throw()
#define CALLOC_SUFFIX throw()
#else
#define MALLOC_SUFFIX __result_use_check __alloc_size(1)
#define FREE_SUFFIX 
#define REALLOC_SUFFIX __result_use_check __alloc_size(2)
#define CALLOC_SUFFIX __result_use_check __alloc_size(1,2)
#endif


extern "C" {
static std::recursive_mutex gpu_alloc_lock;
typedef std::lock_guard<decltype(gpu_alloc_lock)> gpu_lock_guard_t;

static std::recursive_mutex alloc_lock;
typedef std::lock_guard<decltype(alloc_lock)> lock_guard_t;

// We provide our own malloc/calloc/realloc/free, which are then used
// by whatever we link against (i.e., SLAM implementations).

static void *load_and_call_libc_malloc(size_t);
static void load_and_call_libc_free(void*);
static void *load_and_call_libc_calloc(size_t, size_t);
static void *load_and_call_libc_realloc(void*, size_t);

typedef void *(*malloc_t)(size_t);
typedef void (*free_t)(void*);
typedef void *(*realloc_t)(void*, size_t);
typedef void *(*calloc_t)(size_t, size_t);

// TODO: make this thread-safe

static malloc_t libc_malloc = load_and_call_libc_malloc;
static free_t libc_free = load_and_call_libc_free;
static calloc_t libc_calloc = load_and_call_libc_calloc;
static realloc_t libc_realloc = load_and_call_libc_realloc;

static void* load_and_call_libc_malloc(size_t t) {
    libc_malloc = (malloc_t)dlsym(RTLD_NEXT, "malloc");
    return libc_malloc(t);
}
static void load_and_call_libc_free(void *t) {
    libc_free = (free_t)dlsym(RTLD_NEXT, "free");
    libc_free(t);
}

static void* load_and_call_libc_calloc(size_t nmemb, size_t size) {
    libc_calloc = (calloc_t)dlsym(RTLD_NEXT, "calloc");
    return libc_calloc(nmemb, size);
}
static void *load_and_call_libc_realloc(void *t, size_t size) {
    libc_realloc = (realloc_t)dlsym(RTLD_NEXT, "realloc");
    return libc_realloc(t, size);
}

//define a scratch space for calloc
static char calloc_scratch[4096];
static char* calloc_ptr = calloc_scratch;


void *malloc(size_t size) MALLOC_SUFFIX
{
    lock_guard_t lock(alloc_lock);
    void *ptr = libc_malloc(size);
    if(!GSLAM::MemoryMetric::instanceCPU())
        return ptr;

    GSLAM::MemoryMetric::instanceCPU().AddAllocation(ptr, size);

    return ptr;
}

void free(void* ptr) FREE_SUFFIX
{
    lock_guard_t lock(alloc_lock);
    GSLAM::MemoryMetric::instanceCPU().FreeAllocation(ptr);
    libc_free(ptr);
}

void *realloc(void *ptr, size_t newsize) REALLOC_SUFFIX {
    lock_guard_t lock(alloc_lock);

    auto newptr = libc_realloc(ptr, newsize);

    GSLAM::MemoryMetric::instanceCPU().FreeAllocation(ptr);
    GSLAM::MemoryMetric::instanceCPU().AddAllocation(newptr, newsize);

    return newptr;
}

void *calloc(size_t nmemb, size_t size) CALLOC_SUFFIX {
    lock_guard_t lock(alloc_lock);

    // need to be careful with calloc since dlsym will use it! need to do this
    // even if the profiler is disabled.
    static bool reentered = false;
    if(reentered) {
        void *ptr = calloc_ptr;
        calloc_ptr += nmemb*size;
        calloc_ptr += 16-(((uintptr_t)calloc_ptr) % 16);
        if(calloc_ptr >= (calloc_scratch + sizeof(calloc_scratch))) abort();
        return ptr;
    }
    reentered = true;

    void *result = libc_calloc(nmemb, size);
    GSLAM::MemoryMetric::instanceCPU().AddAllocation(result, nmemb * size);
    reentered = false;
    return result;
}

}

void *operator new(std::size_t size) {
    lock_guard_t lock(alloc_lock);

    void *mem = malloc(size);

    if(mem == nullptr) {
        throw std::bad_alloc();
    } else {
        return mem;
    }
}

void *operator new(std::size_t size, const std::nothrow_t &nothrow_value) MALLOC_SUFFIX {
    (void)nothrow_value;
    try {
        return operator new(size);
    } catch (std::exception &e) {
        return nullptr;
    }
}

void *operator new[](std::size_t size) {
    lock_guard_t lock(alloc_lock);

    void *mem = malloc(size);

    if(mem == nullptr) {
        throw std::bad_alloc();
    } else {
        return mem;
    }
}

void *operator new[](std::size_t size, const std::nothrow_t &nothrow_value) MALLOC_SUFFIX {
    (void)nothrow_value;
    try {
        return operator new[](size);
    } catch (std::exception &e) {
        return nullptr;
    }
}

