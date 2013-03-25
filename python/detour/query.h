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

using boost::python::dict;
using boost::python::object;
using boost::python::wrapper;

typedef std::vector<float> FloatList;
typedef std::vector<unsigned char> ByteList;
typedef std::vector<dtPolyRef> dtPolyRefList;


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

	dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef, dtVec3 startPos,
			dtVec3 endPos, const dtQueryFilter* filter, dict out,
			const int maxPath) const;

	dtStatus findStraightPath(dtVec3 startPos, dtVec3 endPos, dtPolyRefList path, dict out,
			const int maxStraightPath, const int options=0) const;

	/// Sliced Pathfinding Functions
	/// Common use case:
	///	-# Call initSlicedFindPath() to initialize the sliced path query.
	///	-# Call updateSlicedFindPath() until it returns complete.
	///	-# Call finalizeSlicedFindPath() to get the path.

	dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
			dtVec3 startPos, dtVec3 endPos, const dtQueryFilter* filter);

	dtStatus updateSlicedFindPath(const int maxIter, dict out);

	dtStatus finalizeSlicedFindPath(dict out, const int maxPath);

	dtStatus finalizeSlicedFindPathPartial(dtPolyRefList existing, dict out, const int maxPath);

	/// Dijkstra Search Functions

	dtStatus findPolysAroundCircle(dtPolyRef startRef, dtVec3 centerPos,
			const float radius, const dtQueryFilter* filter, dict out,
			const int maxResult) const;

	dtStatus findPolysAroundShape(dtPolyRef startRef, dtVec3List verts,
			const dtQueryFilter* filter, dict out, const int maxResult) const;

	/// Local Query Functions

	dtStatus findNearestPoly(dtVec3 center, dtVec3 extents,
			const dtQueryFilter* filter, dict out) const;

	dtStatus queryPolygons(dtVec3 center, dtVec3 extents,
			const dtQueryFilter* filter, dict out, const int maxPolys) const;

	dtStatus findLocalNeighbourhood(dtPolyRef startRef, dtVec3 centerPos,
			const float radius, const dtQueryFilter* filter, dict out,
			const int maxResult) const;

	dtStatus moveAlongSurface(dtPolyRef startRef, dtVec3 startPos,
			dtVec3 endPos, const dtQueryFilter* filter, dict out,
			const int maxVisitedSize) const;

	dtStatus raycast(dtPolyRef startRef, dtVec3 startPos, dtVec3 endPos,
			const dtQueryFilter* filter, dict out, const int maxPath) const;

	dtStatus findDistanceToWall(dtPolyRef startRef, dtVec3 centerPos,
			const float maxRadius, const dtQueryFilter* filter, dict out) const;

	dtStatus getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
			dict out, const int maxSegments) const;

	dtStatus findRandomPoint(const dtQueryFilter* filter, dict out) const;

	dtStatus findRandomPointAroundCircle(dtPolyRef startRef, dtVec3 centerPos,
			const float maxRadius, const dtQueryFilter* filter, dict out) const;

	dtStatus closestPointOnPoly(dtPolyRef ref, dtVec3 pos, dict out) const;

	dtStatus closestPointOnPolyBoundary(dtPolyRef ref, dtVec3 pos,
			dict out) const;

	dtStatus getPolyHeight(dtPolyRef ref, dtVec3 pos, dict out) const;

	/// Miscellaneous Functions

	bool isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const;

	bool isInClosedList(dtPolyRef ref) const;

	const dtNavMesh* getAttachedNavMesh() const;

	dtNodePool* getNodePool() const;

private:
	dtNavMeshQuery* navq_;
};


#endif /* QUERY_H_ */
