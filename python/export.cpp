/*
 * @summary: recast exports
 * @date: 2013-03-17
 * @author: zl
 */

#include "config.h"

extern void export_math();
extern void export_detour();
extern void export_loader();


BOOST_PYTHON_MODULE(_recast)
{
	export_math();
	export_detour();
	export_loader();
}
