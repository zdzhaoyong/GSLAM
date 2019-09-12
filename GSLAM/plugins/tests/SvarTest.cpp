#include "GSLAM/core/GSLAM.h"
#include "gtest.h"

using namespace GSLAM;

TEST(Svar,Variable)
{
    EXPECT_TRUE(Svar()==Svar::Undefined());
    EXPECT_TRUE(Svar(nullptr)==Svar::Null());

    Svar var(false);
    EXPECT_EQ(var.typeName(),"bool");
    EXPECT_TRUE(var.is<bool>());
    EXPECT_FALSE(var.as<bool>());

    EXPECT_TRUE(Svar(1).is<int>());
    EXPECT_TRUE(Svar(1).as<int>()==1);

    EXPECT_TRUE(Svar("").as<std::string>().empty());
    EXPECT_TRUE(Svar(1.).as<double>()==1.);
    EXPECT_TRUE(Svar({1,2}).isArray());
    EXPECT_TRUE(Svar(std::map<int,Svar>({{1,2}})).isDict());

    Svar obj({{"1",1}});
    EXPECT_TRUE(obj.isObject());
    EXPECT_TRUE(obj["1"]==1);

    obj["left"]=Svar("hello");
    obj["parent"]["child"]=3;
    EXPECT_EQ(obj["parent"]["child"],3);
    obj["hello"]["world"]=false;
    EXPECT_EQ(obj["hello"]["world"],false);

    std::map<std::string,Svar> cMap=obj.castAs<std::map<std::string,Svar> >();
    EXPECT_EQ(cMap["1"],1);

    Svar vec({0,1,2,3});
    EXPECT_EQ(vec[1],1);
    vec[1]=2;
    EXPECT_EQ(vec[1],2);

    Svar strlit=R"(
                {"a":[true,1,12.3,"hello"]}
                )"_svar;// create from raw string literal

    EXPECT_TRUE(strlit["a"].isArray());

    EXPECT_EQ("[1,2]"_svar .length(),2);

    EXPECT_TRUE(Svar(std::shared_ptr<std::mutex>(new std::mutex())).is<std::mutex>());
    EXPECT_TRUE(Svar(std::unique_ptr<std::mutex>(new std::mutex())).is<std::mutex>());

    std::vector<int> rvec=vec.castAs<std::vector<int>>();
    EXPECT_EQ(rvec[0],0);


    Svar svarMtx(std::shared_ptr<std::mutex>(new std::mutex()));
    auto mtx=svarMtx.castAs<std::shared_ptr<std::mutex>>();
    mtx->lock();
    mtx->unlock();

    Svar js=Svar::json("");
    EXPECT_TRUE(js.isUndefined());
}

TEST(Svar,HoldEachOther){
    Svar a={"a",1};
    Svar b={"b",2};
    a["bobj"]=b;
    b["aobj"]=a;
}

std::string svar_test_add(std::string left,const std::string& r){
    return left+r;
}

TEST(Svar,Function)
{
    int intV=0,srcV=0;
    Svar intSvar(0);
    // Svar argument is recommended
    SvarFunction isBool([](Svar config){return config.is<bool>();});
    EXPECT_TRUE(isBool.call(false).as<bool>());
    // pointer
    Svar::lambda([](int* ref){*ref=10;})(&intV);
    Svar::lambda([](int* ref){*ref=10;})(intSvar);
    EXPECT_EQ(intV,10);
    EXPECT_EQ(intSvar,10);
    // WARNNING!! ref, should pay attention that when using ref it should always input the svar instead of the raw ref
    Svar::lambda([](int& ref){ref=20;})(intSvar);
    EXPECT_EQ(intSvar,20);
    // const ref
    Svar::lambda([](const int& ref,int* dst){*dst=ref;})(30,&intV);
    EXPECT_EQ(intV,30);
    // const pointer
    Svar::lambda([](const int* ref,int* dst){*dst=*ref;})(&srcV,&intV);
    EXPECT_EQ(intV,srcV);
    // overload is only called when cast is not available!
    Svar funcReturn=Svar::lambda([](int i){return i;});
    funcReturn.as<SvarFunction>().next=Svar::lambda([](char c){return c;});
    EXPECT_EQ(funcReturn(1).cpptype(),typeid(int));
    EXPECT_EQ(funcReturn('c').cpptype(),typeid(char));
    // string argument
    Svar s("hello");
    Svar::lambda([](std::string raw,
                 const std::string c,
                 const std::string& cref,
                 std::string& ref,
                 std::string* ptr,
                 const std::string* cptr){
        EXPECT_EQ(raw,"hello");
        EXPECT_EQ(c,"hello");
        EXPECT_EQ(ref,"hello");
        EXPECT_EQ(cref,"hello");
        EXPECT_EQ((*ptr),"hello");
        EXPECT_EQ((*cptr),"hello");
    })(s,s,s,s,s,s);

    // cpp function binding
    EXPECT_EQ(Svar(svar_test_add)(Svar("a"),std::string("b")).as<std::string>(),"ab");

    // FIXME: Why the below line won't pass and add returned "b"?
    // EXPECT_EQ(Svar(add)(Svar("a"),std::string("b")).as<std::string>(),"ab");

    // static method function binding
    EXPECT_TRUE(Svar::Null().isNull());
    EXPECT_TRUE(Svar(&Svar::Null)().isNull());
}

TEST(Svar,HexAndBase64){
    SvarBuffer buf=SvarBuffer::load(svar.GetString("md5",svar["argv"].as<char**>()[0]));

    EXPECT_EQ(buf.hex(),SvarBuffer::fromHex(buf.hex()).hex());
    EXPECT_EQ(buf.base64(),SvarBuffer::fromBase64(buf.base64()).base64());
    LOG(INFO)<<buf.length();
    std::cout<<buf.md5()<<std::endl;

    std::string str;
    {
        ScopedTimer tm("HexEncode");
        str=buf.hex();
    }
    {
        ScopedTimer tm("HexDecode");
        buf=SvarBuffer::fromHex(str);
    }
    {
        ScopedTimer tm("Base64Encode");
        str=buf.base64();
    }
    {
        ScopedTimer tm("Base64Decode");
        buf=SvarBuffer::fromBase64(str);
    }
}

class BBase{
public:
    virtual bool isBBase(){return true;}
};

class BaseClass:public BBase{
public:
    BaseClass():age_(0){}
    BaseClass(int age):age_(age){}

    int getAge()const{return age_;}
    void setAge(int a){age_=a;}

    static BaseClass create(int a){return BaseClass(a);}
    static BaseClass create1(){return BaseClass();}

    virtual std::string intro()const{return "age:"+std::to_string(age_);}
    virtual bool isBBase(){return false;}

    int age_;
};

class BaseClass1{
public:
    BaseClass1():score(100){}

    double getScore(){return score;}

    double score;
};

class InheritClass: public BaseClass,public BaseClass1
{
public:
    InheritClass(int age,std::string name):BaseClass(age),name_(name){}

    std::string name()const{return name_;}
    void setName(std::string name){name_=name;}

    virtual std::string intro() const{return BaseClass::intro()+", name:"+name_;}
    std::string name_;
};

TEST(Svar,Class){

    SvarClass::Class<BBase>()
            .def("isBBase",&BBase::isBBase);

    SvarClass::Class<BaseClass>()
            .inherit<BBase,BaseClass>()
            .def_static("__init__",[](){return BaseClass();})
            .def_static("__init__",[](int age){return BaseClass(age);})
            .def("getAge",&BaseClass::getAge)
            .def("setAge",&BaseClass::setAge)
            .def_static("create",&BaseClass::create)
            .def_static("create1",&BaseClass::create1)
            .def("intro",&BaseClass::intro);
    Svar a=Svar::create(BaseClass(10));
    Svar baseClass=a.classObject();
    a=baseClass["__init__"](10);
    a=baseClass();//
    a=baseClass(10);//
    EXPECT_EQ(a.call("getAge").as<int>(),10);
    EXPECT_EQ(a.call("intro").as<std::string>(),BaseClass(10).intro());

    SvarClass::Class<BaseClass1>()
            .def("getScore",&BaseClass1::getScore);

    SvarClass::Class<InheritClass>()
            .inherit<BaseClass,InheritClass>()
            .inherit<BaseClass1,InheritClass>()// for none first parents
            .def("__init__",[](int age,std::string name){return InheritClass(age,name);})
            .def("name",&InheritClass::name)
            .def("intro",&InheritClass::intro);
    Svar b=Svar::create(InheritClass(10,"xm"));
    EXPECT_EQ(b.call("getAge").as<int>(),10);
    EXPECT_EQ(b.call("name").as<std::string>(),"xm");
    EXPECT_EQ(b.call("intro").as<std::string>(),InheritClass(10,"xm").intro());
    b.call("setAge",20);
    EXPECT_EQ(b.call("getAge").as<int>(),20);
    EXPECT_EQ(b.call("getScore").as<double>(),100.);
    EXPECT_EQ(b.call("isBBase").as<bool>(),false);
}

TEST(Svar,GetSet){
    Svar var;
    int& testInt=var.GetInt("testInt",20);
    EXPECT_EQ(testInt,20);
    testInt=30;
    EXPECT_EQ(var.GetInt("testInt"),30);
    var["testInt"]=40;
    EXPECT_EQ(testInt,40);
    EXPECT_EQ(var["testInt"],40);
    var.set("int",100);
    var.set("double",100.);
    var.set<std::string>("string","100");
    var.set("bool",true);
    EXPECT_EQ(var["int"],100);
    EXPECT_EQ(var["double"],100.);
    EXPECT_EQ(var["string"],"100");
    // EXPECT_EQ(var["string"],100);
    // FIXME: why this equal, int casted to string? Segment fault in travis-ci
    EXPECT_EQ(var["bool"],true);
}

TEST(Svar,Call){
    EXPECT_EQ(Svar(1).call("__str__"),"1");// Call as member methods
    EXPECT_THROW(SvarClass::instance<int>().call("__str__",1),SvarExeption);// Call as static function
}

TEST(Svar,Cast){
    EXPECT_EQ(Svar(2.).castAs<int>(),2);
    EXPECT_TRUE(Svar(1).castAs<double>()==1.);
}

TEST(Svar,NumOp){
    EXPECT_EQ(-Svar(2.1),-2.1);
    EXPECT_EQ(-Svar(2),-2);
    EXPECT_EQ(Svar(2.1)+Svar(1),3.1);
    EXPECT_EQ(Svar(4.1)-Svar(2),4.1-2);
    EXPECT_EQ(Svar(3)*Svar(3.3),3*3.3);
    EXPECT_EQ(Svar(5.4)/Svar(2),5.4/2);
    EXPECT_EQ(Svar(5)%Svar(2),5%2);
    EXPECT_EQ(Svar(5)^Svar(2),5^2);
    EXPECT_EQ(Svar(5)|Svar(2),5|2);
    EXPECT_EQ(Svar(5)&Svar(2),5&2);
}

TEST(Svar,Dump){
    return;
    Svar obj(std::map<std::string,int>({{"1",1}}));
    std::cout<<obj<<std::endl;
    std::cout<<Svar::array({1,2,3})<<std::endl;
    std::cout<<Svar::create((std::type_index)typeid(1))<<std::endl;
    std::cout<<Svar::lambda([](std::string sig){})<<std::endl;
    std::cout<<SvarClass::Class<int>();
    std::cout<<Svar::instance();
}

TEST(Svar,Iterator){
    auto vec=Svar::array({1,2,3});
    Svar arrayiter=vec.call("__iter__");
    Svar next=arrayiter.classObject()["next"];
    int i=0;
    for(Svar it=next(arrayiter);!it.isUndefined();it=next(arrayiter)){
        EXPECT_EQ(vec[i++],it);
    }
    i=0;
    for(Svar it=arrayiter.call("next");!it.isUndefined();it=arrayiter.call("next"))
    {
        EXPECT_EQ(vec[i++],it);
    }
}

TEST(Svar,Json){
    Svar var;
    var.set("i",1);
    var.set("d",2.);
    var.set("s",Svar("str"));
    var.set("b",false);
    var.set("l",Svar::array({1,2,3}));
    var.set("m",Svar::object({{"a",1},{"b",false}}));
    std::string str=Json::dump(var);
    Svar varCopy=Json::load(str);
    EXPECT_EQ(str,Json::dump(varCopy));
}

TEST(Svar,Alias){
    Svar i=1;
    Svar alias=i;
    EXPECT_EQ(i,alias);
    i=2;
    EXPECT_EQ(i,alias);
    Svar copy=i.clone();
    EXPECT_EQ(i,copy);
    i=4;
    EXPECT_NE(i,copy);

    Svar var;
    var.set("i",i);
    var.set("alias",i);
    EXPECT_EQ(var["i"],var["alias"]);
    i=3;
    EXPECT_EQ(var["i"],3);
    EXPECT_EQ(var["i"],var["alias"]);

    Svar varCopy=var.clone();
    varCopy["i"]=5;
    EXPECT_EQ(var["i"],5);
    EXPECT_EQ(var["alias"],5);

    Svar varDeepCopy=var.clone(100);
    varDeepCopy["i"]=6;
    varDeepCopy["alias"]=7;
    EXPECT_NE(var["i"],6);
    EXPECT_NE(var["alias"],7);

    Svar vec={i,i};
    EXPECT_EQ(vec[0],i);
    i=8;
    EXPECT_EQ(vec[0],8);
    EXPECT_EQ(vec[1],8);

    Svar vecClone=vec.clone(1);
    EXPECT_EQ(vecClone[0],i);
    i=9;
    EXPECT_NE(vecClone[0],i);
}

TEST(Svar,Thread){
    auto doubleThread=[](){
        while(!svar.get<bool>("shouldstop",false)){
            double d=svar.get<double>("thread.Double",100);
            svar.set<double>("thread.Double",d++);

            // double& d=svar.get<double>("thread.Double",100); d++;
            // The above line is dangerous in Mac system since the value may set after memory free

            Svar v=svar.get("thread.Double",100);
            v=100;// this is much safer to set a copy of svar instead the raw pointer

            GSLAM::Rate::sleep(1e-4);
        }
    };
    auto intThread=[](){
        while(!svar.get<bool>("shouldstop",false)){
            // use int instead of double to violately testing robustness
            int i=svar.get<int>("thread.Double",10);
            svar.set<int>("thread.Double",i);

            GSLAM::Rate::sleep(1e-4);
        }
    };
    std::vector<std::thread> threads;
    for(int i=0;i<svar.GetInt("doubleThreads",4);i++) threads.push_back(std::thread(doubleThread));
    for(int i=0;i<svar.GetInt("intThreads",4);i++) threads.push_back(std::thread(intThread));

    GSLAM::Rate::sleep(1);
    svar.set("shouldstop",true);
    for(auto& it:threads) it.join();
}
