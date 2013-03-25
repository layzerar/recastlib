/*
 * @summary: recast common
 * @date: 2013-03-20
 * @author: zl
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "config.h"


inline void IndexError()
{
	using boost::python::throw_error_already_set;
	PyErr_SetString(PyExc_IndexError, "Index out of range");
	throw_error_already_set();
}

inline void MemoryError()
{
	using boost::python::throw_error_already_set;
	PyErr_SetString(PyExc_MemoryError, "System out of memory");
	throw_error_already_set();
}


#endif /* COMMON_H_ */
