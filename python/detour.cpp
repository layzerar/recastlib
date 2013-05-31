/*
 * @summary: recast detour
 * @date: 2013-03-17
 * @author: zl
 */

#include "config.h"
#include "detour.h"
#include "common.h"
#include "convertor.h"


/*
 * rename overloading functions
 */
dtResult (dtNavMeshQueryWraper::*dtNavMeshQueryWraper_findStraightPath4)(
	dtVec3 startPos, dtVec3 endPos, const dtPolyRefList& path,
	const int maxStraightPath) const = &dtNavMeshQueryWraper::findStraightPath;
dtResult (dtNavMeshQueryWraper::*dtNavMeshQueryWraper_findStraightPath5)(
	dtVec3 startPos, dtVec3 endPos, const dtPolyRefList& path,
	const int maxStraightPath, const int options) const = &dtNavMeshQueryWraper::findStraightPath;


void export_detour()
{
	using namespace boost::python;

	enum_<dtStatus>("dtStatus")
		.value("DT_FAILURE", DT_FAILURE)
		.value("DT_SUCCESS", DT_SUCCESS)
		.value("DT_IN_PROGRESS", DT_IN_PROGRESS)
		.value("DT_WRONG_MAGIC", DT_WRONG_MAGIC)
		.value("DT_WRONG_VERSION", DT_WRONG_VERSION)
		.value("DT_OUT_OF_MEMORY", DT_OUT_OF_MEMORY)
		.value("DT_INVALID_PARAM", DT_INVALID_PARAM)
		.value("DT_BUFFER_TOO_SMALL", DT_BUFFER_TOO_SMALL)
		.value("DT_OUT_OF_NODES", DT_OUT_OF_NODES)
		.value("DT_PARTIAL_RESULT", DT_PARTIAL_RESULT)
		.value("DT_OUT_OF_NODES", DT_OUT_OF_NODES)
		.value("DT_PARTIAL_RESULT", DT_PARTIAL_RESULT)
		.value("DT_STATUS_DETAIL_MASK", DT_STATUS_DETAIL_MASK)
	;

	enum_<dtStraightPathFlags>("dtStraightPathFlags")
		.value("DT_STRAIGHTPATH_START", DT_STRAIGHTPATH_START)
		.value("DT_STRAIGHTPATH_END", DT_STRAIGHTPATH_END)
		.value("DT_STRAIGHTPATH_OFFMESH_CONNECTION", DT_STRAIGHTPATH_OFFMESH_CONNECTION)
	;

	enum_<dtStraightPathOptions>("dtStraightPathOptions")
		.value("DT_STRAIGHTPATH_AREA_CROSSINGS", DT_STRAIGHTPATH_AREA_CROSSINGS)
		.value("DT_STRAIGHTPATH_ALL_CROSSINGS", DT_STRAIGHTPATH_ALL_CROSSINGS)
	;


	class_<dtNavMesh, boost::noncopyable, dtNavMesh*>("dtNavMesh", no_init);

	class_<dtQueryFilterWraper, boost::noncopyable, dtQueryFilter*>("dtQueryFilter")
		.def("passFilter", &dtQueryFilter::passFilter, &dtQueryFilterWraper::passFilter)
		.def("getCost", &dtQueryFilter::getCost, &dtQueryFilterWraper::getCost)
		.def("getAreaCost", &dtQueryFilter::getAreaCost, &dtQueryFilterWraper::getAreaCost)
		.def("setAreaCost", &dtQueryFilter::setAreaCost, &dtQueryFilterWraper::setAreaCost)
		.def("getIncludeFlags", &dtQueryFilterWraper::getIncludeFlags)
		.def("setIncludeFlags", &dtQueryFilterWraper::setIncludeFlags)
		.def("getExcludeFlags", &dtQueryFilterWraper::getExcludeFlags)
		.def("setExcludeFlags", &dtQueryFilterWraper::setExcludeFlags)
	;

	class_<dtNavMeshQueryWraper, boost::noncopyable>("dtNavMeshQuery")
		.def("init", &dtNavMeshQueryWraper::init, with_custodian_and_ward<1, 2>())
		.def("findPath", &dtNavMeshQueryWraper::findPath)
		.def("findStraightPath", dtNavMeshQueryWraper_findStraightPath4)
		.def("findStraightPath", dtNavMeshQueryWraper_findStraightPath5)
		.def("initSlicedFindPath", &dtNavMeshQueryWraper::initSlicedFindPath)
		.def("updateSlicedFindPath", &dtNavMeshQueryWraper::updateSlicedFindPath)
		.def("finalizeSlicedFindPath", &dtNavMeshQueryWraper::finalizeSlicedFindPath)
		.def("finalizeSlicedFindPathPartial", &dtNavMeshQueryWraper::finalizeSlicedFindPathPartial)
		.def("findPolysAroundCircle", &dtNavMeshQueryWraper::findPolysAroundCircle)
		.def("findPolysAroundShape", &dtNavMeshQueryWraper::findPolysAroundShape)
		.def("findNearestPoly", &dtNavMeshQueryWraper::findNearestPoly)
		.def("queryPolygons", &dtNavMeshQueryWraper::queryPolygons)
		.def("findLocalNeighbourhood", &dtNavMeshQueryWraper::findLocalNeighbourhood)
		.def("moveAlongSurface", &dtNavMeshQueryWraper::moveAlongSurface)
		.def("raycast", &dtNavMeshQueryWraper::raycast)
		.def("findDistanceToWall", &dtNavMeshQueryWraper::findDistanceToWall)
		.def("getPolyWallSegments", &dtNavMeshQueryWraper::getPolyWallSegments)
		.def("findRandomPoint", &dtNavMeshQueryWraper::findRandomPoint)
		.def("findRandomPointAroundCircle", &dtNavMeshQueryWraper::findRandomPointAroundCircle)
		.def("closestPointOnPoly", &dtNavMeshQueryWraper::closestPointOnPoly)
		.def("closestPointOnPolyBoundary", &dtNavMeshQueryWraper::closestPointOnPolyBoundary)
		.def("getPolyHeight", &dtNavMeshQueryWraper::getPolyHeight)
		.def("isValidPolyRef", &dtNavMeshQueryWraper::isValidPolyRef)
		.def("isInClosedList", &dtNavMeshQueryWraper::isInClosedList)
		.def("getAttachedNavMesh", &dtNavMeshQueryWraper::getAttachedNavMesh, return_internal_reference<>())
		//.def("getNodePool", &dtNavMeshQueryWraper::getNodePool, return_internal_reference<>())
	;


	def("dtStatusFailed", &dtStatusFailed);
	def("dtStatusSucceed", &dtStatusSucceed);
	def("dtStatusDetail", &dtStatusDetail);
	def("dtStatusInProgress", &dtStatusInProgress);


	vector_from_seq_converter<float>();
	to_python_converter<FloatList, vector_to_list_converter<float> >();
	vector_from_seq_converter<unsigned char>();
	to_python_converter<ByteList, vector_to_list_converter<unsigned char> >();
	vector_from_seq_converter<dtPolyRef>();
	to_python_converter<dtPolyRefList, vector_to_list_converter<dtPolyRef> >();
	to_python_converter<dtResult, pair_to_tuple_converter<dtStatus, dict> >();
	to_python_converter<dtResultI, pair_to_tuple_converter<dtStatus, int> >();
	to_python_converter<dtResultF, pair_to_tuple_converter<dtStatus, float> >();
	to_python_converter<dtResultV, pair_to_tuple_converter<dtStatus, dtVec3> >();

}

