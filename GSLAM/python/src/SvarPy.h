#ifndef SVAR_SVARPY_H
#define SVAR_SVARPY_H

#include "GSLAM/core/GSLAM.h"
#include <Python.h>
#include <cstring>

#if PY_MAJOR_VERSION >= 3 /// Compatibility macros for various Python versions
#define SVAR_INSTANCE_METHOD_NEW(ptr, class_) PyInstanceMethod_New(ptr)
#define SVAR_INSTANCE_METHOD_CHECK PyInstanceMethod_Check
#define SVAR_INSTANCE_METHOD_GET_FUNCTION PyInstanceMethod_GET_FUNCTION
#define SVAR_BYTES_CHECK PyBytes_Check
#define SVAR_BYTES_FROM_STRING PyBytes_FromString
#define SVAR_BYTES_FROM_STRING_AND_SIZE PyBytes_FromStringAndSize
#define SVAR_BYTES_AS_STRING_AND_SIZE PyBytes_AsStringAndSize
#define SVAR_BYTES_AS_STRING PyBytes_AsString
#define SVAR_BYTES_SIZE PyBytes_Size
#define SVAR_LONG_CHECK(o) PyLong_Check(o)
#define SVAR_LONG_AS_LONGLONG(o) PyLong_AsLongLong(o)
#define SVAR_LONG_FROM_SIGNED(o) PyLong_FromSsize_t((ssize_t) o)
#define SVAR_LONG_FROM_UNSIGNED(o) PyLong_FromSize_t((size_t) o)
#define SVAR_BYTES_NAME "bytes"
#define SVAR_STRING_NAME "str"
#define SVAR_SLICE_OBJECT PyObject
#define SVAR_FROM_STRING PyUnicode_FromString
#define SVAR_STR_TYPE ::SVAR::str
#define SVAR_BOOL_ATTR "__bool__"
#define SVAR_NB_BOOL(ptr) ((ptr)->nb_bool)
#define SVAR_PYTHON_IMPL(name) \
    extern "C" SVAR_EXPORT PyObject *PyInit_##name()

#else
#define SVAR_INSTANCE_METHOD_NEW(ptr, class_) PyMethod_New(ptr, nullptr, class_)
#define SVAR_INSTANCE_METHOD_CHECK PyMethod_Check
#define SVAR_INSTANCE_METHOD_GET_FUNCTION PyMethod_GET_FUNCTION
#define SVAR_BYTES_CHECK PyString_Check
#define SVAR_BYTES_FROM_STRING PyString_FromString
#define SVAR_BYTES_FROM_STRING_AND_SIZE PyString_FromStringAndSize
#define SVAR_BYTES_AS_STRING_AND_SIZE PyString_AsStringAndSize
#define SVAR_BYTES_AS_STRING PyString_AsString
#define SVAR_BYTES_SIZE PyString_Size
#define SVAR_LONG_CHECK(o) (PyInt_Check(o) || PyLong_Check(o))
#define SVAR_LONG_AS_LONGLONG(o) (PyInt_Check(o) ? (long long) PyLong_AsLong(o) : PyLong_AsLongLong(o))
#define SVAR_LONG_FROM_SIGNED(o) PyInt_FromSsize_t((ssize_t) o) // Returns long if needed.
#define SVAR_LONG_FROM_UNSIGNED(o) PyInt_FromSize_t((size_t) o) // Returns long if needed.
#define SVAR_BYTES_NAME "str"
#define SVAR_STRING_NAME "unicode"
#define SVAR_SLICE_OBJECT PySliceObject
#define SVAR_FROM_STRING PyString_FromString
#define SVAR_STR_TYPE ::SVAR::bytes
#define SVAR_BOOL_ATTR "__nonzero__"
#define SVAR_NB_BOOL(ptr) ((ptr)->nb_nonzero)
#define SVAR_PYTHON_IMPL(name) \
    static PyObject *SVAR_init_wrapper();               \
    extern "C" SVAR_EXPORT void init##name() {          \
        (void)SVAR_init_wrapper();                      \
    }                                                       \
    PyObject *SVAR_init_wrapper()
#endif

namespace GSLAM {


struct PyObjectHolder{
    PyObjectHolder(PyObject* o):obj(o){
        Py_IncRef(obj);
    }
    ~PyObjectHolder(){
        Py_DecRef(obj);
    }
    PyObject* obj;
};

struct SvarPy: public PyObject{
    Svar* var;

    static PyTypeObject& svar_object()
    {
        static PyTypeObject* type=nullptr;
        if(type) return *type;
        constexpr auto *name = "svar_object";

        auto heap_type = (PyHeapTypeObject *) PyType_Type.tp_alloc(&PyType_Type, 0);
        if (!heap_type)
            LOG(FATAL)<<"svar_object(): error allocating svar_object!";

        heap_type->ht_name = SVAR_FROM_STRING(name);

        type = &heap_type->ht_type;
        type->tp_name = name;
        type->tp_doc="";
        type->tp_basicsize=sizeof(SvarPy);
        type->tp_base = type_incref(&PyType_Type);
        type->tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_HEAPTYPE;

        type->tp_as_number = &heap_type->as_number;
        type->tp_as_sequence = &heap_type->as_sequence;
        type->tp_as_mapping = &heap_type->as_mapping;

        type->tp_new=[](PyTypeObject *type, PyObject *, PyObject *)->PyObject * {
    #if defined(PYPY_VERSION)
            // PyPy gets tp_basicsize wrong (issue 2482) under multiple inheritance when the first inherited
            // object is a a plain Python type (i.e. not derived from an extension type).  Fix it.
            ssize_t instance_size = static_cast<ssize_t>(sizeof(instance));
            if (type->tp_basicsize < instance_size) {
                type->tp_basicsize = instance_size;
            }
    #endif
            PyObject *self = type->tp_alloc(type, 0);
            auto inst = reinterpret_cast<SvarPy*>(self);
            // Allocate the value/holder internals:
            inst->var=nullptr;
            LOG(INFO)<<"Creating <"<<type->tp_name<<" object at "<<self<<">";
            return self;
        };

        type->tp_repr = [](PyObject* obj){
            std::stringstream sst;
            sst<<"<"<<obj->ob_type->tp_name<<" object at "<<obj<<">";
            return SVAR_FROM_STRING(sst.str().c_str());
        };

        type->tp_init=[](PyObject *self, PyObject *, PyObject *)->int{
            PyTypeObject *type = Py_TYPE(self);
            std::string msg;
        #if defined(PYPY_VERSION)
            msg += handle((PyObject *) type).attr("__module__").cast<std::string>() + ".";
        #endif
            msg += type->tp_name;
            msg += ": No constructor defined!";
            PyErr_SetString(PyExc_TypeError, msg.c_str());
            return -1;
        };

        type->tp_dealloc=[](PyObject *self) {
            SvarPy* obj=(SvarPy*)self;
            if(obj->var)
            {
                LOG(INFO)<<"Deleting <"<<self->ob_type->tp_name<<" object at "<<self<<">";
                delete obj->var;
                obj->var=nullptr;
            }

            auto type = Py_TYPE(self);
            type->tp_free(self);
            Py_DECREF(type);
        };

        if (PyType_Ready(type) < 0)
            LOG(FATAL)<<("svar_object(): failure in PyType_Ready()!");

        PyObject_SetAttrString((PyObject *) type, "__module__", PyUnicode_FromString("svar_builtin"));
        return *type;
    }

    static PyObject* incref(PyObject* obj){
        Py_IncRef(obj);
        return obj;
    }

    static PyObject* decref(PyObject* obj){
        Py_DecRef(obj);
        return obj;
    }

    static PyTypeObject* type_incref(PyTypeObject* obj){
        Py_IncRef((PyObject*)obj);
        return obj;
    }

    static PyTypeObject* type_decref(PyTypeObject* obj){
        Py_DecRef((PyObject*)obj);
        return obj;
    }

    static PyObject* getPy(Svar src){
        if(src.is<PyObject*>()) return src.as<PyObject*>();
        if(src.is<PyObjectHolder>())
            return src.as<PyObjectHolder>().obj;

        SvarClass* cls=src.classPtr();
        Svar func=(*cls)["getPy"];
        if(func.isFunction())
            return func.as<SvarFunction>().Call({src}).as<PyObject*>();

        std::function<PyObject*(Svar)> convert;

        if(src.is<int>())
            convert=[](Svar src){return PyLong_FromLong(src.as<int>());};
        else if(src.is<double>())
            convert=[](Svar src){return PyFloat_FromDouble(src.as<double>());};

        else if(src.isNull()||src.isUndefined())
            convert=[](Svar src){return incref(Py_None);};

        else if(src.is<std::string>())
            convert=[](Svar src){
                std::string& str=src.as<std::string>();
                return PyUnicode_FromStringAndSize(str.data(),str.size());
            };
        else if(src.is<SvarArray>())
            convert=[](Svar src){

            PyObject* obj=PyTuple_New(src.length());
            size_t index = 0;
            for (Svar& value : src.as<SvarArray>()._var) {
                PyObject* value_ =SvarPy::getPy(value);
                if (!value_)
                {
                    delete value_;
                    return incref(Py_None);
                }
                PyTuple_SetItem(obj, (ssize_t) index++, value_); // steals a reference
            }
            return obj;
        };
        else if(src.is<SvarObject>())
            convert=[](Svar src){
            PyObject* dict=PyDict_New();
            for (auto kv : src.as<SvarObject>()._var) {
                PyObject* key = PyUnicode_FromStringAndSize(kv.first.data(),kv.first.size());
                PyObject* value = SvarPy::getPy(kv.second);
                if (!key || !value)
                    return incref(Py_None);
                PyDict_SetItem(dict,key,value);
            }
            return dict;
        };
        else if(src.is<SvarFunction>())
            convert=[](Svar src){return (PyObject*)SvarPy::getPyFunction(src);};
        else if(src.is<SvarClass>())
            convert=[](Svar src){ return (PyObject*)SvarPy::getPyClass(src);};
        else if(src.is<SvarProperty>()){
            convert=[](Svar src){return (PyObject*)SvarPy::getPyProperty(src);};
        }
        else convert=[](Svar src){
            PyTypeObject* py_class=getPyClass(src.classObject());
            SvarPy* obj=(SvarPy*)py_class->tp_new(py_class,nullptr,nullptr);
            obj->var=new Svar(src);
            return (PyObject*)obj;
        };

        cls->def("getPy",convert);

        return convert(src);
    }

    static PyObject* getPyProperty(Svar src){
      SvarProperty& property=src.as<SvarProperty>();
      PyObject *result = PyObject_Call((PyObject*)&PyProperty_Type,
                                       getPy({property._fget,property._fset,property._doc}),nullptr);
      if (!result){
        struct error_scope {
          PyObject *type, *value, *trace;
          error_scope() { PyErr_Fetch(&type, &value, &trace); }
          ~error_scope() { PyErr_Restore(type, value, trace); }
        }error;
        throw SvarExeption(fromPy(error.value));
      }
      return result;
    }

    static PyObject* getPyFunction(Svar src){
        SvarFunction& svar_func=src.as<SvarFunction>();
        PyMethodDef* func = new PyMethodDef();
        std::memset(func, 0, sizeof(PyMethodDef));
        func->ml_name = svar_func.name.c_str();
        func->ml_flags = METH_VARARGS| METH_KEYWORDS ;
        if(svar_func.is_constructor)
            func->ml_flags|=METH_CLASS;
        func->ml_doc=svar_func.signature.c_str();

        func->ml_meth = [](PyObject *capsule, PyObject *args)->PyObject*{
            Svar* func=(Svar*)PyCapsule_GetPointer(capsule,nullptr);
            SvarFunction& svarFunc=func->as<SvarFunction>();

            try{
                Svar svar_args=SvarPy::fromPy(args);
                if(svarFunc.is_constructor)
                {
                    return SvarPy::getPy(svarFunc.Call({svar_args}));
                }

                return SvarPy::getPy(svarFunc.Call(svar_args.as<SvarArray>()._var));
            }
            catch(SvarExeption e){
                PyErr_SetString(PyExc_RuntimeError, e.what());
                return incref(Py_None);
            }
        };

        PyObject* capsule=PyCapsule_New(new Svar(src),nullptr,[](PyObject *o) {
            delete (Svar*)PyCapsule_GetPointer(o, nullptr);
        });

        auto c_func=PyCFunction_NewEx(func,capsule,nullptr);
        if(!svar_func.is_method)
            return c_func;

        auto m_ptr = SVAR_INSTANCE_METHOD_NEW(c_func, nullptr);
        if (!m_ptr)
            std::cerr<<("cpp_function::cpp_function(): Could not allocate instance method object");
        Py_DECREF(c_func);
        return m_ptr;
    }

    static PyTypeObject* getPyClass(Svar var){
        SvarClass& cls=var.as<SvarClass>();
        if(cls._attr.exist("PyTypeObject"))
            return cls._attr["PyTypeObject"].castAs<PyTypeObject*>();

        auto heap_type = (PyHeapTypeObject *) PyType_Type.tp_alloc(&PyType_Type, 0);
        if (!heap_type){
            LOG(ERROR)<<("Error allocating type!")<<std::endl;
            return (PyTypeObject*)Py_None;
        }

        heap_type->ht_name = SVAR_FROM_STRING(cls.__name__.c_str());

        PyTypeObject* type = &heap_type->ht_type;
        type->tp_name = cls.__name__.c_str();
        type->tp_base = type_incref(&PyBaseObject_Type);// FIXME: Why cause segment fault help when inherit svar_object
        type->tp_basicsize=sizeof(SvarPy);
        type->tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_HEAPTYPE;
        type->tp_doc="";
        type->tp_as_number = &heap_type->as_number;
        type->tp_as_sequence = &heap_type->as_sequence;
        type->tp_as_mapping = &heap_type->as_mapping;

        type->tp_new=[](PyTypeObject *type, PyObject *, PyObject *)->PyObject * {
    #if defined(PYPY_VERSION)
            // PyPy gets tp_basicsize wrong (issue 2482) under multiple inheritance when the first inherited
            // object is a a plain Python type (i.e. not derived from an extension type).  Fix it.
            ssize_t instance_size = static_cast<ssize_t>(sizeof(instance));
            if (type->tp_basicsize < instance_size) {
                type->tp_basicsize = instance_size;
            }
    #endif
            PyObject *self = type->tp_alloc(type, 0);
            auto inst = reinterpret_cast<SvarPy*>(self);
            // Allocate the value/holder internals:
            inst->var=nullptr;
            DLOG(INFO)<<"Creating <"<<type->tp_name<<" object at "<<self<<">";
            return self;
        };

        type->tp_init=[](PyObject *self, PyObject *, PyObject *)->int{
            PyTypeObject *type = Py_TYPE(self);
            std::string msg;
        #if defined(PYPY_VERSION)
            msg += handle((PyObject *) type).attr("__module__").cast<std::string>() + ".";
        #endif
            msg += type->tp_name;
            msg += ": No constructor defined!";
            PyErr_SetString(PyExc_TypeError, msg.c_str());
            return -1;
        };

        type->tp_repr = [](PyObject* obj){
            std::stringstream sst;
            sst<<"<"<<obj->ob_type->tp_name<<" object at "<<obj<<">";
            return SVAR_FROM_STRING(sst.str().c_str());
        };

        type->tp_dealloc=[](PyObject *self) {
            SvarPy* obj=(SvarPy*)self;
            if(obj->var)
            {
                DLOG(INFO)<<"Deleting <"<<self->ob_type->tp_name<<" object at "<<self<<">";
                delete obj->var;
                obj->var=nullptr;
            }

            auto type = Py_TYPE(self);
            type->tp_free(self);
            Py_DECREF(type);
        };

        if (PyType_Ready(type) < 0)
            LOG(ERROR)<<("make_static_property_type(): failure in PyType_Ready()!");

        PyObject_SetAttrString((PyObject *) type, "__module__", SVAR_FROM_STRING("svar_builtins"));
        cls._attr.set("PyTypeObject",type);
        for(std::pair<std::string,Svar> f:cls._methods.as<SvarObject>()._var)
        {
            if(f.first=="__init__"){
                Svar func_init=Svar::lambda([f](std::vector<Svar> args){
                        SvarPy* self=args[0].as<SvarPy*>();
                        std::vector<Svar> used_args(args.begin()+1,args.end());
                        Svar ret=f.second.as<SvarFunction>().Call(used_args);
                        self->var=new Svar(ret);
                        Py_IncRef(self);
                        return incref(Py_None);
            });
                func_init.as<SvarFunction>().is_constructor=true;
                func_init.as<SvarFunction>().is_method=true;
                func_init.as<SvarFunction>().signature=f.second.as<SvarFunction>().signature;
                PyObject_SetAttrString((PyObject*)type,"__init__",getPy(func_init));
                continue;
            }
            PyObject_SetAttrString((PyObject*)type,f.first.c_str(),getPy(f.second));
        }

        PyObject* capsule=PyCapsule_New(new Svar(var),nullptr,[](PyObject *o) {
            delete (Svar*)PyCapsule_GetPointer(o, nullptr);
        });

        PyObject_SetAttrString((PyObject*)type,"svar_class",capsule);


        return (PyTypeObject*)type;
    }

    static Svar fromPy(PyObject* obj)// this never fails
    {
        PyTypeObject *type = Py_TYPE(obj);

        if(PyObject_HasAttrString((PyObject*)type,"svar_class"))
        {
    //        PyObject *capsule=PyObject_GetAttrString((PyObject*)type,"svar_class");
    //        assert(PyCapsule_GetPointer(capsule,nullptr));
            SvarPy* o=(SvarPy*)obj;
            if(!o->var)
            {
               return PyObjectHolder(obj);
            }
            return *o->var;
        }

        if(PyNumber_Check(obj))
        {
            if(PyFloat_Check(obj))
                return PyFloat_AsDouble(obj);
            else return (int)PyLong_AsLong(obj);
        }

        if(PyList_Check(obj)){
            std::vector<Svar> array(PyList_Size(obj));
            for(Py_ssize_t i=0;i<array.size();++i)
                array[i]=fromPy(PyList_GetItem(obj,i));
            return Svar(array);
        }

        if (PyUnicode_Check(obj)) {
            PyObject* buf=PyUnicode_AsUTF8String(obj);
            char *buffer=nullptr;
            ssize_t length=0;

            if (SVAR_BYTES_AS_STRING_AND_SIZE(buf, &buffer, &length))
                LOG(ERROR)<<("Unable to extract string contents! (invalid type)");
            return std::string(buffer, (size_t) length);
        }

        if(PyTuple_Check(obj)){
            std::vector<Svar> array(PyTuple_Size(obj));
            for(Py_ssize_t i=0;i<array.size();++i)
                array[i]=fromPy(PyTuple_GetItem(obj,i));
            return Svar(array);
        }

        if(PyDict_Check(obj)){
            std::map<std::string,Svar> dict;
            PyObject *key, *value;
            Py_ssize_t pos = 0;

            while (PyDict_Next(obj, &pos, &key, &value)) {
                dict.insert(make_pair(fromPy(key).castAs<std::string>(),
                                      fromPy(value)));
            }
            return dict;
        }

        if(PyFunction_Check(obj)){
          SvarFunction func;
          Svar holder=PyObjectHolder(obj);
          func._func=[holder](std::vector<Svar>& args)->Svar{
            return SvarPy::fromPy(PyObject_Call(holder.as<PyObjectHolder>().obj, SvarPy::getPy(args),nullptr));
          };
          func.do_argcheck=false;
          return func;
        }

        return Svar(PyObjectHolder(obj));
    }


    static PyObject* getModule(GSLAM::Svar var){
        if(var.isObject())
        {
            PyObject* pyModule = nullptr;
            const char* name="svar";
            const char* doc="";

            if(var.exist("__name__"))
                name= var["__name__"].castAs<std::string>().c_str();

            if(var.exist("__doc__"))
                doc = var["__doc__"].castAs<std::string>().c_str();

#if PY_MAJOR_VERSION<3
            pyModule=Py_InitModule3(name,nullptr,doc);
#else
            PyModuleDef *def = new PyModuleDef();
            memset(def, 0, sizeof(PyModuleDef));
            def->m_name=name;
            def->m_doc=doc;
            def->m_size = -1;
            Py_INCREF(def);

            pyModule = PyModule_Create(def);
#endif
            if(!pyModule)
                return nullptr;

            for(std::pair<std::string,Svar> pair:var.as<SvarObject>()._var)
            {
                PyObject* obj=getModule(pair.second);
                PyModule_AddObject(pyModule, pair.first.c_str(), obj);
            }

            return pyModule;
        }

        return getPy(var);
    }

};


}

#endif
