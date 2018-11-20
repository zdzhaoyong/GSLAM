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

The %GSLAM library is a set of C++ header files which provide basic numerics facilities:
	- @link TooN::Vector Vectors@endlink, @link TooN::Matrix matrices@endlink and @link gLinAlg etc @endlink
	- @link gDecomps Matrix decompositions@endlink
	- @link gOptimize Function optimization@endlink
	- @link gTransforms Parameterized matrices (eg transformations)@endlink 
	- @link gEquations linear equations@endlink
	- @link gFunctions Functions (eg automatic differentiation and numerical derivatives) @endlink

It provides classes for statically- (known at compile time) and dynamically-
(unknown at compile time) sized vectors and matrices and it can delegate
advanced functions (like large SVD or multiplication of large matrices) to
LAPACK and BLAS (this means you will need libblas and liblapack).


\section sUsage How to use TooN 
\ingroup gInterface
This section is arranged as a FAQ. Most answers include code fragments. Assume
<code>using namespace TooN;</code>.

 - \ref sDownload
 - \ref sStart

 	\subsection sDownload Getting the code and installing
	
	To get the code from cvs use:

	cvs -z3 -d:pserver:anoncvs@cvs.savannah.nongnu.org:/cvsroot/toon co TooN

	The home page for the library with a version of this documentation is at:

	http://mi.eng.cam.ac.uk/~er258/cvd/toon.html

	The code will work as-is, and comes with a default configuration, which
	should work on any system.

	On a unix system, <code>./configure && make install </code> will  install
	TooN to the correct place.  Note there is no code to be compiled, but the
	configure script performs some basic checks.

	On non-unix systems, e.g. Windows and embedded systems, you may wish to 
	configure the library manually. See \ref sManualConfiguration.

\subsection sStart Getting started

To begin, just in include the right file:

@code
#include <TooN/TooN.h>
@endcode

Everything lives in the <code>TooN</code> namespace.

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
class Zhaoyong{
int name;
};



