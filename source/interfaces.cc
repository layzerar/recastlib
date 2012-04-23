#include "detour.h"
#include "NavMeshQuery.h"
#include "NavMeshLoader.h"

#include "boost.h"


BOOST_PYTHON_MODULE(_detour)
{
	using namespace boost::python;

	enum_<dtStatus>("dt")
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

	def("dtStatusFailed", dtStatusFailed);
	def("dtStatusSucceed", dtStatusSucceed);
	def("dtStatusDetail", dtStatusDetail);
	def("dtStatusInProgress", dtStatusInProgress);

	class_<dtNavMesh>("_NavMesh", no_init);

	class_<NavMeshQuery>("_NavMeshQuery", no_init)
		.def("findPath", &NavMeshQuery::findPath)
		.def("findStraightPath", &NavMeshQuery::findStraightPath)
		.def("initSlicedFindPath", &NavMeshQuery::initSlicedFindPath)
		.def("updateSlicedFindPath", &NavMeshQuery::updateSlicedFindPath)
		.def("finalizeSlicedFindPath", &NavMeshQuery::finalizeSlicedFindPath)
		.def("finalizeSlicedFindPathPartial", &NavMeshQuery::finalizeSlicedFindPathPartial)
		.def("findPolysAroundCircle", &NavMeshQuery::findPolysAroundCircle)
		.def("findPolysAroundShape", &NavMeshQuery::findPolysAroundShape)
		.def("findNearestPoly", &NavMeshQuery::findNearestPoly)
		.def("queryPolygons", &NavMeshQuery::queryPolygons)
		.def("findLocalNeighbourhood", &NavMeshQuery::findLocalNeighbourhood)
		.def("moveAlongSurface", &NavMeshQuery::moveAlongSurface)
		.def("raycast", &NavMeshQuery::raycast)
		.def("findDistanceToWall", &NavMeshQuery::findDistanceToWall)
		.def("getPolyWallSegments", &NavMeshQuery::getPolyWallSegments)
		.def("findRandomPoint", &NavMeshQuery::findRandomPoint)
		.def("findRandomPointAroundCircle", &NavMeshQuery::findRandomPointAroundCircle)
		.def("closestPointOnPoly", &NavMeshQuery::closestPointOnPoly)
		.def("closestPointOnPolyBoundary", &NavMeshQuery::closestPointOnPolyBoundary)
		.def("getPolyHeight", &NavMeshQuery::getPolyHeight)
		.def("isValidPolyRef", &NavMeshQuery::isValidPolyRef)
		.def("isInClosedList", &NavMeshQuery::isInClosedList)
		.def("getAreaCost", &NavMeshQuery::getAreaCost)
		.def("setAreaCost", &NavMeshQuery::setAreaCost)
		.def("getIncludeFlags", &NavMeshQuery::getIncludeFlags)
		.def("setIncludeFlags", &NavMeshQuery::setIncludeFlags)
		.def("getExcludeFlags", &NavMeshQuery::getExcludeFlags)
		.def("setExcludeFlags", &NavMeshQuery::setExcludeFlags)
	;

	def("dtNavMesh", loadNavMesh, return_value_policy<manage_new_object>());
	def("dtNavMeshQuery", loadNavMeshQuery, return_value_policy<manage_new_object>());

	//class_<dtQueryFilter>("dtQueryFilter")
	//	.def("getAreaCost", &dtQueryFilter::getAreaCost)
	//	.def("setAreaCost", &dtQueryFilter::setAreaCost)
	//	.def("getIncludeFlags", &dtQueryFilter::getIncludeFlags)
	//	.def("setIncludeFlags", &dtQueryFilter::setIncludeFlags)
	//	.def("getExcludeFlags", &dtQueryFilter::getExcludeFlags)
	//	.def("setExcludeFlags", &dtQueryFilter::setExcludeFlags)
	//;

}
