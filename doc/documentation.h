// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

///////////////////////////////////////////////////////
// General Doxygen documentation
///////////////////////////////////////////////////////

///////////////////////////////////////////////////////
// The main title page
/**
@mainpage

\section sIntro Introduction

The %GSLAM library is a set of C++11 header files which provide basic facilities:
	- @link Estimator
	- @link Optimizer
	- @link Vocabular

Estimator aims to provide a collection of close-form solvers cover all interesting cases with robust sample consensus (RANSAC); Optimizer aims to provide an unified interface for popular nonlinear SLAM problems; Vocabulary aims to provide an efficient and portable bag of words implementation for place recolonization with multi-thread and SIMD optimization.


\section sUsage How to use GSLAM 
\ingroup gInterface
This section is arranged as a FAQ. Most answers include code fragments. Assume
<code>using namespace GSLAM;</code>.

 - \ref sDevelopment

 	\subsection sDevelopment API Documentation is under developing
	
*/


/// @defgroup gInterface Interface of GSLAM
/// GSLAM provides a general SLAM interface with only headers depends on C++11 for deveopment and commercial usages.
///@{
///@}

/// @defgroup gUtils Utils of GSLAM
/// GSLAM provides a collection of util classes for developers.

/// @defgroup gApplication Applications of GSLAM
/// GSLAM provides some helpful applications for users based on the framework.

/// @ingroup gInterface


