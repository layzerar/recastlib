//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURASSERT_H
#define DETOURASSERT_H

// Note: This header file's only purpose is to include define assert.
// Feel free to change the file and include your own implementation instead.

#ifdef NDEBUG
// From http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/
#	define dtAssert(x) do { (void)sizeof(x); } while((void)(__LINE__==-1),false)  
#else
#	include <assert.h> 
#	define dtAssert assert
#endif

// Note: Replace the implement of dtAssert
#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>

#undef dtAssert
#define dtAssert(e) ((e) ? (void)0: _dt_assert(#e, __FILE__, __LINE__))

inline void _dt_assert(const char* arg1, const char* arg2, int lineno)
{
	using namespace boost::python;

	PyErr_SetString(PyExc_AssertionError, arg1);
	throw_error_already_set();
}

#endif // DETOURASSERT_H
