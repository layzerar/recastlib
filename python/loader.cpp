/*
 * @summary: recast loader
 * @date: 2013-03-17
 * @author: zl
 */

#include "config.h"
#include "loader.h"


void export_loader()
{
	using namespace boost::python;

	def("dtLoadSampleTileMesh", &dtLoadSampleTileMesh,
				return_value_policy<manage_new_object>());
	def("dtLoadUnityTileMesh", &dtLoadUnityTileMesh,
		return_value_policy<manage_new_object>());

	// set dtLoadSampleTileMesh as default loader.
	scope().attr("dtLoadNavMesh") = scope().attr("dtLoadSampleTileMesh");
}
