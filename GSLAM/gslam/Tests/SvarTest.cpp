#include "gtest.h"
#include <GSLAM/core/VecParament.h>
#include <GSLAM/core/Svar.h>

template <typename T>
void testSvarType(GSLAM::Svar var,std::string name,T def,T set){
    GSLAM::SvarWithType<T> instance;

    auto ret=instance.get_ptr(name);
    EXPECT_TRUE(ret==NULL);
    EXPECT_TRUE(instance.get_ptr(name,def)!=nullptr);
    EXPECT_TRUE(instance.get_var(name,def)==def);
    EXPECT_TRUE(instance[name]==def);

    T& ref=var.Get(name,def);
    EXPECT_EQ(var.Get(name,def),def);
    EXPECT_EQ(var.Get(name,set),def);
    EXPECT_EQ(ref,def);

    var.Get(name,def)=set;
    EXPECT_EQ(var.Get(name,def),set);
    EXPECT_EQ(var.Get(name,set),set);
    EXPECT_EQ(ref,set);

}

TEST(Svar,Int){
    GSLAM::Svar var;
    testSvarType(svar,"Int",100,200);
    testSvarType(var,"Int",100,200);
    EXPECT_EQ(var.Get("Int",0),200);

    svar.ParseLine("testInt=20");
    EXPECT_EQ(svar.GetInt("testInt"),20);
    svar.ParseLine("testInt?=30");
    EXPECT_EQ(svar.GetInt("testInt"),20);

    svar.ParseLine("InvalidInt=invalid?");
    EXPECT_EQ(var.Get<int>("InvalidInt"),0);
}

TEST(Svar,Double){
    GSLAM::Svar var;
    testSvarType(svar,"Double",0.,20.);
    testSvarType(var,"Double",0.,20.);
    EXPECT_EQ(var.Get("Double",0.),20.);

    svar.ParseLine("testDouble=20");
    EXPECT_EQ(svar.GetDouble("testDouble"),20);
    svar.ParseLine("testDouble?=30");
    EXPECT_EQ(svar.GetDouble("testDouble"),20);

    svar.ParseLine("InvalidDouble=invalid?");
    EXPECT_EQ(var.Get<double>("InvalidDouble"),0.);

}

TEST(Svar,String){
    GSLAM::Svar var;
    testSvarType<std::string>(svar,"String","world","hello");
    testSvarType<std::string>(var,"String","world","hello");
    EXPECT_EQ(var.GetString("String","world"),"hello");

    svar.ParseLine("testString=20");
    EXPECT_EQ(svar.GetString("testString"),"20");
    svar.ParseLine("testString?=30");
    EXPECT_EQ(svar.GetString("testString"),"20");
}

TEST(Svar,Pointer){
    GSLAM::Svar var;
    testSvarType<void*>(svar,"Pointer",NULL,&var);
    testSvarType<void*>(var,"Pointer",NULL,&var);
}

template <typename T>
bool operator==(VecParament<T> l,VecParament<T> r){
    if(l.size()!=r.size()) return false;
    for(int i=0;i<r.size();i++) if(l[i]!=r[i]) return false;
    return true;
}

TEST(Svar,VecParament){
    VecParament<double> vec={1,2},vec2;
    testSvarType(svar,"VecParament",vec,vec2);
}

TEST(Scommand,Regist){
    GSLAM::Svar     var;
    GSLAM::Scommand command(var);
    var.GetString("Result")="Success";
    EXPECT_FALSE(var.exist("Result"));
    command.Call("GetString Result");
    EXPECT_TRUE(var.exist("Result"));
}

TEST(Svar,Parse){
    std::string str=
            "Name=GSLAM\n"
            "Auther=ZhaoYong\n"
            "function ShowName\n"
            "echo Name is $(Name)\n"
            "FunctionCalled=1\n"
            "echo FunctionCalled=1\n"
            "endfunction\n"
            "if $(Auther)=ZhaoYong\n"
            "echo Auther of $(Name) is $(Auther)\n"
            "Name=ChangedName\n"
            "ShowName\n"
            "else\n"
            "echo Auther of $(Name) is not ZhaoYong\n"
            "ElseCalled=1\n"
            "endif";
    std::stringstream sst(str);
    GSLAM::Svar var;

    var.ParseStream(sst);
    EXPECT_TRUE(var.GetInt("FunctionCalled"));
    EXPECT_FALSE(var.GetInt("ElseCalled"));
    EXPECT_EQ(var.GetString("Name",""),"ChangedName");
}

