/*
 * @summary: detour navmesh query
 * @date: 2013-03-26
 * @author: zl
 */

#ifndef QUERY_H_
#define QUERY_H_

#include "config.h"
#include "dtmath.h"
#include <vector>
#include <utility>

using boost::python::dict;
using boost::python::object;
using boost::python::wrapper;

typedef std::vector<float> FloatList;
typedef std::vector<unsigned char> ByteList;
typedef std::vector<dtPolyRef> dtPolyRefList;
typedef std::pair<dtStatus, dict> dtResult;
typedef std::pair<dtStatus, int> dtResultI;
typedef std::pair<dtStatus, float> dtResultF;
typedef std::pair<dtStatus, dtVec3> dtResultV;


/*
 * Wraper of dtQueryFilter
 *
 */
struct dtQueryFilterWraper: dtQueryFilter, wrapper<dtQueryFilter>
{
public:
	dtQueryFilterWraper();
	virtual ~dtQueryFilterWraper();

	virtual bool passFilter(const dtPolyRef ref,
			const dtMeshTile* tile,
			const dtPoly* poly) const;

	virtual float getCost(const float* pa, const float* pb,
			const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
			const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
			const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;

	virtual float getAreaCost(const int i) const;
	virtual void setAreaCost(const int i, const float cost);
};


/*
 * Wrapper of dtNavMeshQuery
 *
 */
class dtNavMeshQueryWraper {
public:
	dtNavMeshQueryWraper();
	~dtNavMeshQueryWraper();

	dtStatus init(const dtNavMesh* nav, const int maxNodes) const;

	/// Standard Pathfinding Functions

	dtResult findPath(dtPolyRef startRef, dtPolyRef endRef,
			dtVec3 startPos, dtVec3 endPos, const dtQueryFilter* filter,
			const int maxPath) const;

	dtResult findStraightPath(dtVec3 startPos, dtVec3 endPos,
		const dtPolyRefList& path, const int maxStraightPath) const;

	dtResult findStraightPath(dtVec3 startPos, dtVec3 endPos,
			const dtPolyRefList& path, const int maxStraightPath,
			const int options) const;

	/// Sliced Pathfinding Functions
	/// Common use case:
	///	-# Call initSlicedFindPath() to initialize the sliced path query.
	///	-# Call updateSlicedFindPath() until it returns complete.
	///	-# Call finalizeSlicedFindPath() to get the path.

	dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
			dtVec3 startPos, dtVec3 endPos, const dtQueryFilter* filter);

	dtResultI updateSlicedFindPath(const int maxIter);

	dtResult finalizeSlicedFindPath(const int maxPath);

	dtResult finalizeSlicedFindPathPartial(const dtPolyRefList& existing,
			const int maxPath);

	/// Dijkstra Search Functions

	dtResult findPolysAroundCircle(dtPolyRef startRef, dtVec3 centerPos,
			const float radius, const dtQueryFilter* filter,
			const int maxResult) const;

	dtResult findPolysAroundShape(dtPolyRef startRef, const dtVec3List& verts,
			const dtQueryFilter* filter, const int maxResult) const;

	/// Local Query Functions

	dtResult findNearestPoly(dtVec3 center, dtVec3 extents,
			const dtQueryFilter* filter) const;

	dtResult queryPolygons(dtVec3 center, dtVec3 extents,
			const dtQueryFilter* filter, const int maxPolys) const;

	dtResult findLocalNeighbourhood(dtPolyRef startRef, dtVec3 centerPos,
			const float radius, const dtQueryFilter* filter, const int maxResult) const;

	dtResult moveAlongSurface(dtPolyRef startRef, dtVec3 startPos,
			dtVec3 endPos, const dtQueryFilter* filter, const int maxVisitedSize) const;

	dtResult raycast(dtPolyRef startRef, dtVec3 startPos, dtVec3 endPos,
			const dtQueryFilter* filter, const int maxPath) const;

	dtResult findDistanceToWall(dtPolyRef startRef, dtVec3 centerPos,
			const float maxRadius, const dtQueryFilter* filter) const;

	dtResult getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
			const int maxSegments) const;

	dtResult findRandomPoint(const dtQueryFilter* filter) const;

	dtResult findRandomPointAroundCircle(dtPolyRef startRef, dtVec3 centerPos,
			const float maxRadius, const dtQueryFilter* filter) const;

	dtResultV closestPointOnPoly(dtPolyRef ref, dtVec3 pos) const;

	dtResultV closestPointOnPolyBoundary(dtPolyRef ref, dtVec3 pos) const;

	dtResultF getPolyHeight(dtPolyRef ref, dtVec3 pos) const;

	/// Miscellaneous Functions

	bool isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const;

	bool isInClosedList(dtPolyRef ref) const;

	const dtNavMesh* getAttachedNavMesh() const;

	dtNodePool* getNodePool() const;

private:
	dtNavMeshQuery* navq_;
};


#endif /* QUERY_H_ */
