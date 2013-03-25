/*
 * @summary: recast exports
 * @date: 2013-03-17
 * @author: zl
 */

#include "config.h"
#include "export.h"


BOOST_PYTHON_MODULE(_recast)
{
	export_math();
	export_detour();
	export_loader();
}
