#!/bin/sh
##BEGIN_INTRODUCTION##	This shell is used to compile GSLAM quickly.
##END_INTRODUCTION##

#####################  Build in Things   ######################
#. $Function_Top/Include/Enviroment_Config.inc
Call_Path=$(pwd) #Where this call from?
#Where is here?
if [ -n "$Function_Path" ];then
	Here_Path=$Function_Path
else
	Here_Path=$(dirname $(readlink -f $0))
fi
#Who am I?
if [ -n "$Function_Name" ];then
	File_Name=$Function_Name.func
else
	File_Name=${0##*/};
	Function_Name=${File_Name%.*};
fi

echo_introduction() #Introduce myself
{
	if [ -n "$1" ];then
	FILE_Content=$(cat < $1)
	else
	FILE_Content=$(cat < $Here_Path/$File_Name)
	fi
	INTRO=${FILE_Content#*##BEGIN_INTRODUCTION##};
	INTRO=${INTRO%%##END_INTRODUCTION##*};
	echo "$INTRO"
}

echo_help() #Echo my help or others'
{
	if [ -n "$1" ];then
		FILE_Content=$(cat < $1)
	else
		FILE_Content=$(cat < $Here_Path/$File_Name)
	fi
	HELP=${FILE_Content##*##BEGIN_HELP##};
	HELP=${HELP%##END_HELP##*};
	echo "usage:  $Function_Name [options values]"
	echo options:"$HELP"
}

##################### other functions below ######################
installDependency()
{
	echo "Installing dependency for GSLAM."
	sudo apt-get install build-essential g++ libqt4-core libqt4-dev libqt4-gui qt4-doc qt4-designer 	-y
	sudo apt-get install libboost-all-dev libboost-thread* libboost-system* libboost-filesystem* -y
	sudo apt-get install libopencv-dev -y
	sudo apt-get install -y freeglut3 freeglut3-dev libglew-dev libglew1.10
	sudo apt-get install -y libqglviewer-dev libqglviewer2 
	sudo apt-get install -y libsuitesparse-dev libeigen3-dev	
}

doCompile()
{
	PIL_PATH=$Here_Path/ThirdParty/PIL-1.1.0
	echo "Building PIL at $PIL_PATH ."

	if [ -d "$PIL_PATH" ];then
		echo "PIL has already been downloaded, skip git clone."
	else
		git clone https://github.com/zdzhaoyong/PIL2		
	fi
	
	if [ -d "$PIL_PATH/build" ];then
		echo "PIL has already been built, skip building."
	else
		mkdir $PIL_PATH/build;cd $PIL_PATH/build;cmake $PIL_PATH;make
		echo "PIL building done!"
	fi

	G2O_PATH=$Here_Path/ThirdParty/g2o
	if [ -d "$G2O_PATH/build" ];then
		echo "g2o has already been built, skip building." 
	else
		echo "building g2o..."
		mkdir $G2O_PATH/build;cd $G2O_PATH/build;cmake $G2O_PATH;make
		echo "g2o building done."
	fi

	if [ -d "$Here_Path/build" ];then
		echo "GSLAM has been build, rebuilding..."
		cd $Here_Path/build;cmake $Here_Path;make
	else
		mkdir $Here_Path/build;cd $Here_Path/build;cmake $Here_Path;make
	fi
}

######################  main below  ##############################
if [ -n "$1" ];then
	while [ -n "$1" ]; do
	case $1 in
##BEGIN_HELP##
		-h)     shift 1;echo_help;exit 1;;                   #Show usages 
		-i)     shift 1;echo_introduction;exit 1;;           #Show introduction 
		-edit)  shift 1;gedit $Here_Path/$File_Name;exit 1;; #Edit this function 
		-d)     shift 1;installDependency;exit 1;; #Install dependency
		-c)     shift 1;doCompile ;exit 1;; #Do compile all 
		-a)     shift 1;installDependency ;doCompile;exit 1;; #Do compile all 
		-*)     echo "error: no such option $1. -h for help";exit 1;; 
		*)      $*;exit 1;;                                  #Call function here
##END_HELP##
	esac
	done
else
	echo "Please use: bash buildAll.sh -a "
	echo_help
fi
#echo ---------------------End Of $Function_Name-----------------------


