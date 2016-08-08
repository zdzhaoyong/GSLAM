# GSLAM (Gengeral Simultaneous Localization and Mapping Framework)

## 1. Introduction

### 1.1. What is GSLAM?
GSLAM is aimed to provide a general open-source SLAM framework with following features :
-> 1. Compatibility with feature based or direct methods
-> 2. Support Monocular, Stereo, RGB-D or any custom input types
-> 3. Internal hardware support of Cameras, IMUs and GPS; General optimization utils;
-> 4. Support coorperation SLAM to build a singular map

### 1.2. What we can do with GSLAM?
1. *For SLAM developers* : Everyone can develop their own SLAM implementation based on GSLAM and publish it as 
2. *For SLAM users* : Applications are able to use different SLAM implementations with the same API without recompilation and implementations are loaed at runtime.

### 1.3. Folder structure
* src -- source folder
 - GSLAM   -- Common SLAM APIs 
 - GUtils  -- Utils for GSLAM implementations including Pose optimization, BA frame work
 - ORBSLAM -- Implementation of ORBSLAM
 - test    -- Test system of GSLAM

* Thirdparty --- thirdparty libraries
 - PIL 		-- the basic c++ library for configuration, display, plugin loader
 - Eigen 	-- an opensoure linear algebra library
 - g2o 		-- a general optimization framework
 - pba 		-- an opensoure bundle implementation on GPU
 
## 2. Compilation

###2.2. Dependency

**OpenCV** : sudo apt-get install libopencv-dev 

**Qt** : sudo apt-get install build-essential g++ libqt4-core libqt4-dev libqt4-gui qt4-doc qt4-designer 

**QGLViewer** : sudo apt-get install libqglviewer-dev libqglviewer2 

## 3. Usage

