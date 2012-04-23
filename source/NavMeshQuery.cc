#include "boost.h"
#include "NavMeshQuery.h"


using namespace boost::python;


inline void extractVector3(float fpos[3], object pos)
{
	typedef extract<float> extract_type;
	fpos[0] = extract_type(pos[0]);
	fpos[1] = extract_type(pos[1]);
	fpos[2] = extract_type(pos[2]);
}

float frand_01()
{
	using namespace boost;
	mt19937 rng(43);
	static uniform_01<mt19937> zeroone(rng);
	return (float)zeroone();
}

dtQueryFilter* dtAllocQueryFilter()
{
	void* mem = dtAlloc(sizeof(dtQueryFilter), DT_ALLOC_PERM);
	if (!mem) 
		return NULL;
	return new(mem) dtQueryFilter;
}

void dtFreeQueryFilter(dtQueryFilter* qf)
{
	if (!qf)
		return;
	qf->~dtQueryFilter();
	dtFree(qf);
}

NavMeshQuery::NavMeshQuery(dtQueryFilter* filter, 
	dtNavMeshQuery* navq,
	object type) : m_filter(filter), m_navq(navq), m_type(type)
{
}

NavMeshQuery::~NavMeshQuery()
{
	dtFreeQueryFilter(m_filter);
	dtFreeNavMeshQuery(m_navq);
}

tuple NavMeshQuery::findPath(dtPolyRef startRef, 
	dtPolyRef endRef,
	object startPos,
	object endPos,
	const int maxPath) const
{
	list path;
	dtStatus status;
	dtPolyRef *pPath;
	float pStartPos[3], pEndPos[3];

	extractVector3(pEndPos, endPos);
	extractVector3(pStartPos, startPos);

	pPath = NULL;
	if (maxPath > 0)
	{
		pPath = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*maxPath, DT_ALLOC_TEMP);
		if (!pPath)
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
	}

	if (pPath)
	{
		int pathCount = 0;

		status = m_navq->findPath(startRef, endRef, 
			pStartPos, pEndPos, 
			m_filter, pPath, &pathCount, maxPath);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr = pPath;
			dtPolyRef const *endPtr = pPath + pathCount;

			try
			{
				while (iPtr != endPtr)
					path.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pPath);
				throw;
			}
		}

		dtFree(pPath);
	}

	return make_tuple(status, path);
}

tuple NavMeshQuery::findStraightPath(object startPos,
	object endPos, 
	list path, 
	const int maxStraightPath) const
{
	int pathSize;
	dtStatus status;
	unsigned char* pStraightPathFlags;
	dtPolyRef *pPath , *pStraightPathRefs;
	float pStartPos[3], pEndPos[3], *pStraightPath;
	list straightPath, straightPathFlags, straightPathRefs;

	pathSize = len(path);
	extractVector3(pEndPos, endPos);
	extractVector3(pStartPos, startPos);

	pPath = NULL;
	pStraightPathRefs = NULL;
	pStraightPathFlags = NULL;
	pStraightPath = NULL;

	if (pathSize > 0 && maxStraightPath > 0)
	{
		pStraightPath = (float*)dtAlloc(
			sizeof(float)*3*maxStraightPath, DT_ALLOC_TEMP);
		pStraightPathFlags = (unsigned char*)dtAlloc(
			sizeof(unsigned char)*maxStraightPath, DT_ALLOC_TEMP);
		pStraightPathRefs = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxStraightPath, DT_ALLOC_TEMP);
		pPath = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*pathSize, DT_ALLOC_TEMP);
		if (!pStraightPath || !straightPathFlags || !straightPathRefs || !pPath)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
			dtFree(pPath);
			dtFree(pStraightPathRefs);
			dtFree(pStraightPathFlags);
			dtFree(pStraightPath);
			pPath = NULL;
			pStraightPathRefs = NULL;
			pStraightPathFlags = NULL;
			pStraightPath = NULL;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pStraightPath && straightPathFlags && straightPathRefs && pPath)
	{
		try
		{
			int iCount = 0;

			while (iCount < pathSize)
			{
				pPath[iCount] = extract<dtPolyRef>(path[iCount]);
				++iCount;
			}
			int straightPathCount = 0;

			status = m_navq->findStraightPath(pStartPos, pEndPos,
				pPath, pathSize,
				pStraightPath, pStraightPathFlags, pStraightPathRefs, 
				&straightPathCount, maxStraightPath);
			if (dtStatusSucceed(status))
			{
				float *pos;
				int iSPC = 0;
				
				while (iSPC < straightPathCount)
				{
					pos = iSPC * 3 + pStraightPath;
					straightPath.append(m_type(pos[0], pos[1], pos[2]));
					straightPathFlags.append(*(pStraightPathFlags + iSPC));
					straightPathRefs.append(*(pStraightPathRefs + iSPC));
				}
			}
		}
		catch (error_already_set const &)
		{
			dtFree(pPath);
			dtFree(pStraightPathRefs);
			dtFree(pStraightPathFlags);
			dtFree(pStraightPath);
			throw;
		}

		dtFree(pPath);
		dtFree(pStraightPathRefs);
		dtFree(pStraightPathFlags);
		dtFree(pStraightPath);
	}
	
	return make_tuple(status, make_tuple(straightPath, 
		straightPathFlags, 
		straightPathRefs));
}

dtStatus NavMeshQuery::initSlicedFindPath(dtPolyRef startRef,
	dtPolyRef endRef,
	object startPos,
	object endPos)
{
	float pStartPos[3], pEndPos[3];

	extractVector3(pEndPos, endPos);
	extractVector3(pStartPos, startPos);

	return m_navq->initSlicedFindPath(startRef, endRef, 
		pStartPos, pEndPos, m_filter);
}

tuple NavMeshQuery::updateSlicedFindPath(const int maxIter)
{
	dtStatus status;
	int doneIters = 0;

	status = m_navq->updateSlicedFindPath(maxIter, &doneIters);
	return make_tuple(status, doneIters);
}

tuple NavMeshQuery::finalizeSlicedFindPath(const int maxPath)
{
	list path;
	dtStatus status;
	dtPolyRef *pPath = NULL;

	if (maxPath > 0)
	{
		pPath = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*maxPath, DT_ALLOC_TEMP);
		if (!pPath)
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pPath)
	{
		int pathCount = 0;

		status = m_navq->finalizeSlicedFindPath(pPath, &pathCount, maxPath);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr = pPath;
			dtPolyRef const *endPtr = pPath + pathCount;

			try
			{
				while (iPtr != endPtr)
					path.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pPath);
				throw;
			}
		}

		dtFree(pPath);
	}

	return make_tuple(status, path);
}

tuple NavMeshQuery::finalizeSlicedFindPathPartial(list existing, 
	const int maxPath)
{
	list path;
	dtStatus status;
	int existingSize;
	dtPolyRef *pPath, *pExisting;

	pPath = NULL;
	pExisting = NULL;
	existingSize = len(existing);
	if (existingSize > 0 && maxPath > 0)
	{
		pPath = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxPath, DT_ALLOC_TEMP);
		pExisting = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*existingSize, DT_ALLOC_TEMP);
		if (!pPath || !pExisting)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
			dtFree(pPath);
			dtFree(pExisting);
			pPath = NULL;
			pExisting = NULL;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pPath)
	{
		int pathCount = 0;

		status = m_navq->finalizeSlicedFindPathPartial(pExisting, existingSize,
			pPath, &pathCount, maxPath);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr = pPath;
			dtPolyRef const *endPtr = pPath + pathCount;

			try
			{
				while (iPtr != endPtr)
					path.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pPath);
				throw;
			}
		}

		dtFree(pPath);
	}

	return make_tuple(status, path);
}

tuple NavMeshQuery::findPolysAroundCircle(dtPolyRef startRef, 
	object centerPos, 
	const float radius, 
	const int maxResult) const
{
	dtStatus status;
	list resultRef, resultParent;
	float pCenterPos[3], resultCost;
	dtPolyRef *pResultRef, *pResultParent;

	extractVector3(pCenterPos, centerPos);

	pResultRef = NULL;
	pResultParent = NULL;
	if (maxResult > 0 && radius > 0.0)
	{
		pResultRef = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxResult, DT_ALLOC_TEMP);
		pResultParent = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxResult, DT_ALLOC_TEMP);
		if (!pResultRef || !pResultParent)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
			dtFree(pResultRef);
			dtFree(pResultParent);
			pResultRef = NULL;
			pResultParent = NULL;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pResultRef && pResultParent)
	{
		int resultCount = 0;

		status = m_navq->findPolysAroundCircle(startRef, pCenterPos, radius, 
			m_filter, pResultRef, pResultParent, 
			&resultCost, &resultCount, maxResult);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr;
			dtPolyRef const *endPtr;

			try
			{
				iPtr = pResultRef;
				endPtr = pResultRef + resultCount;
				while (iPtr != endPtr)
					resultRef.append(*(iPtr++));

				iPtr = pResultParent;
				endPtr = pResultParent + resultCount;
				while (iPtr != endPtr)
					resultParent.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pResultRef);
				dtFree(pResultParent);
				throw;
			}
		}

		dtFree(pResultRef);
		dtFree(pResultParent);
	}

	return make_tuple(status, make_tuple(resultRef, resultParent, resultCost));
}

tuple NavMeshQuery::findPolysAroundShape(dtPolyRef startRef, 
	list verts, 
	const int maxResult) const
{
	int nVerts;
	dtStatus status;
	float *pVerts, resultCost;
	list resultRef, resultParent;
	dtPolyRef *pResultRef, *pResultParent;

	nVerts = len(verts);

	pVerts = NULL;
	pResultRef = NULL;
	pResultParent = NULL;
	if (nVerts > 0 && maxResult > 0)
	{
		pVerts = (float*)dtAlloc(
			sizeof(float)*3*nVerts, DT_ALLOC_TEMP);
		pResultRef = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxResult, DT_ALLOC_TEMP);
		pResultParent = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxResult, DT_ALLOC_TEMP);
		if (!pResultRef || !pResultParent)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
			dtFree(pVerts);
			dtFree(pResultRef);
			dtFree(pResultParent);
			pVerts = NULL;
			pResultRef = NULL;
			pResultParent = NULL;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pVerts && pResultRef && pResultParent)
	{
		int resultCount = 0;

		status = m_navq->findPolysAroundShape(startRef, pVerts, nVerts, 
			m_filter, pResultRef, pResultParent, 
			&resultCost, &resultCount, maxResult);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr;
			dtPolyRef const *endPtr;

			try
			{
				iPtr = pResultRef;
				endPtr = pResultRef + resultCount;
				while (iPtr != endPtr)
					resultRef.append(*(iPtr++));

				iPtr = pResultParent;
				endPtr = pResultParent + resultCount;
				while (iPtr != endPtr)
					resultParent.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pVerts);
				dtFree(pResultRef);
				dtFree(pResultParent);
				throw;
			}
		}

		dtFree(pVerts);
		dtFree(pResultRef);
		dtFree(pResultParent);

	}

	return make_tuple(status, make_tuple(resultRef, resultParent, resultCost));
}

/* (dtStatus, (nearestRef, nearestPt)) */
tuple NavMeshQuery::findNearestPoly(object center, 
	object extents) const
{
	dtStatus status;
	dtPolyRef nearestRef;
	float pCenter[3], pExtents[3], pNearestPt[3];

	extractVector3(pCenter, center);
	extractVector3(pExtents, extents);

	nearestRef = 0;
	pNearestPt[0] = 0.0f;
	pNearestPt[1] = 0.0f;
	pNearestPt[2] = 0.0f;
	status = m_navq->findNearestPoly(pCenter, pExtents, 
		m_filter, &nearestRef, pNearestPt);

	return make_tuple(status, make_tuple(nearestRef, 
		m_type(pNearestPt[0], pNearestPt[1], pNearestPt[2])));
}

/* (dtStatus, polys) */
tuple NavMeshQuery::queryPolygons(object center, 
	object extents, 
	const int maxPolys) const
{
	list polys;
	dtStatus status;
	dtPolyRef *pPolys;
	float pCenter[3], pExtents[3];

	extractVector3(pCenter, center);
	extractVector3(pExtents, extents);

	pPolys = NULL;
	if (maxPolys > 0)
	{
		pPolys = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxPolys, DT_ALLOC_TEMP);
		if (!pPolys)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pPolys)
	{
		int polyCount = 0;

		status = m_navq->queryPolygons(pCenter, pExtents, 
				m_filter, pPolys, &polyCount, maxPolys);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr;
			dtPolyRef const *endPtr;

			try
			{
				iPtr = pPolys;
				endPtr = pPolys + polyCount;
				while (iPtr != endPtr)
					polys.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pPolys);
				throw;
			}
		}

		dtFree(pPolys);
	}

	return make_tuple(status, polys);
}

tuple NavMeshQuery::findLocalNeighbourhood(dtPolyRef startRef, 
	object centerPos, 
	const float radius, 
	const int maxResult) const
{
	dtStatus status;
	float pCenterPos[3];
	list resultRef, resultParent;
	dtPolyRef *pResultRef, *pResultParent;

	extractVector3(pCenterPos, centerPos);

	pResultRef = NULL;
	pResultParent = NULL;
	if (maxResult > 0 && radius > 0.0)
	{
		pResultRef = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxResult, DT_ALLOC_TEMP);
		pResultParent = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxResult, DT_ALLOC_TEMP);
		if (!pResultRef || !pResultParent)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
			dtFree(pResultRef);
			dtFree(pResultParent);
			pResultRef = NULL;
			pResultParent = NULL;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pResultRef && pResultParent)
	{
		int resultCount = 0;

		status = m_navq->findLocalNeighbourhood(startRef, pCenterPos, radius,
			m_filter, pResultRef, pResultParent, &resultCount, maxResult);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr;
			dtPolyRef const *endPtr;

			try
			{
				iPtr = pResultRef;
				endPtr = pResultRef + resultCount;
				while (iPtr != endPtr)
					resultRef.append(*(iPtr++));

				iPtr = pResultParent;
				endPtr = pResultParent + resultCount;
				while (iPtr != endPtr)
					resultParent.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pResultRef);
				dtFree(pResultParent);
				throw;
			}
		}

		dtFree(pResultRef);
		dtFree(pResultParent);
	}

	return make_tuple(status, make_tuple(resultRef, resultParent));
}

/* (dtStatus, (resultPos, visited)) */
tuple NavMeshQuery::moveAlongSurface(dtPolyRef startRef, 
	object startPos, 
	object endPos, 
	const int maxVisitedSize) const
{
	list visited;
	dtStatus status;
	dtPolyRef *pVisited;
	float pStartPos[3], pEndPos[3], pResultPos[3];

	extractVector3(pEndPos, endPos);
	extractVector3(pStartPos, startPos);

	pVisited = NULL;
	pResultPos[0] = 0.0f;
	pResultPos[1] = 0.0f;
	pResultPos[2] = 0.0f;
	if (maxVisitedSize > 0)
	{
		pVisited = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxVisitedSize, DT_ALLOC_TEMP);
		if (!pVisited)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pVisited)
	{
		int visitedCount = 0;

		status = m_navq->moveAlongSurface(startRef, pStartPos, pEndPos, 
			m_filter, pResultPos, pVisited, &visitedCount, maxVisitedSize);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr;
			dtPolyRef const *endPtr;

			try
			{
				iPtr = pVisited;
				endPtr = pVisited + visitedCount;
				while (iPtr != endPtr)
					visited.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pVisited);
				throw;
			}
		}

		dtFree(pVisited);
	}

	return make_tuple(status, make_tuple(
		m_type(pResultPos[0], pResultPos[1], pResultPos[2]), visited));
}

/* (dtStatus, (t, hitNormal, path)) */
tuple NavMeshQuery::raycast(dtPolyRef startRef, 
	object startPos, 
	object endPos, 
	const int maxPath) const
{
	list path;
	dtStatus status;
	dtPolyRef *pPath;
	float pStartPos[3], pEndPos[3], pHitNormal[3], t;

	extractVector3(pEndPos, endPos);
	extractVector3(pStartPos, startPos);

	pPath = NULL;
	t = 0.0f;
	pHitNormal[0] = 0.0f;
	pHitNormal[1] = 0.0f;
	pHitNormal[2] = 0.0f;
	if (maxPath > 0)
	{
		pPath = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxPath, DT_ALLOC_TEMP);
		if (!pPath)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pPath)
	{
		int pathCount = 0;

		status = m_navq->raycast(startRef, pStartPos, pEndPos,
			m_filter, &t, pHitNormal, pPath, &pathCount, maxPath);
		if (dtStatusSucceed(status))
		{
			dtPolyRef *iPtr;
			dtPolyRef const *endPtr;

			try
			{
				iPtr = pPath;
				endPtr = pPath + pathCount;
				while (iPtr != endPtr)
					path.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pPath);
				throw;
			}
		}

		dtFree(pPath);
	}

	return make_tuple(status, make_tuple(t, 
		m_type(pHitNormal[0], pHitNormal[1], pHitNormal[2]), path));
}

/* (dtStatus, (hitDist, hitPos, hitNormal)) */
tuple NavMeshQuery::findDistanceToWall(dtPolyRef startRef, 
	object centerPos, 
	const float maxRadius) const
{
	dtStatus status;
	float pCenterPos[3], pHitPos[3], pHitNormal[3], hitDist;

	extractVector3(pCenterPos, centerPos);

	pHitPos[0] = 0.0f;
	pHitPos[1] = 0.0f;
	pHitPos[2] = 0.0f;
	pHitNormal[0] = 0.0f;
	pHitNormal[1] = 0.0f;
	pHitNormal[2] = 0.0f;

	if (maxRadius > 0.0)
	{
		status = m_navq->findDistanceToWall(startRef, pCenterPos, maxRadius, 
			m_filter, &hitDist, pHitPos, pHitNormal);
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	return make_tuple(status, make_tuple(hitDist, 
		m_type(pHitPos[0], pHitPos[1], pHitPos[2]), 
		m_type(pHitNormal[0], pHitNormal[1], pHitNormal[2])));
}

/* (dtStatus, (segmentVerts, segmentRefs)) */
tuple NavMeshQuery::getPolyWallSegments(dtPolyRef startRef, 
	const int maxSegments) const
{
	dtStatus status;
	float *pSegmentVerts;
	dtPolyRef *pSegmentRefs;
	list segmentVerts, segmentRefs;

	pSegmentVerts = NULL;
	pSegmentRefs = NULL;
	if (maxSegments > 0)
	{
		pSegmentVerts = (float*)dtAlloc(
			sizeof(float)*6*maxSegments, DT_ALLOC_TEMP);
		pSegmentRefs = (dtPolyRef*)dtAlloc(
			sizeof(dtPolyRef)*maxSegments, DT_ALLOC_TEMP);
		if (!pSegmentVerts || !pSegmentRefs)
		{
			status = DT_FAILURE | DT_OUT_OF_MEMORY;
			dtFree(pSegmentVerts);
			dtFree(pSegmentRefs);
			pSegmentVerts = NULL;
			pSegmentRefs = NULL;
		}
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	if (pSegmentVerts && pSegmentRefs)
	{
		int segmentCount = 0;

		status = m_navq->getPolyWallSegments(startRef, 
			m_filter, pSegmentVerts, pSegmentRefs, &segmentCount, maxSegments);
		if (dtStatusSucceed(status))
		{
			float *iVPtr;
			float const *endVPtr;
			dtPolyRef *iPtr;
			dtPolyRef const *endPtr;

			try
			{
				iVPtr = pSegmentVerts;
				endVPtr = pSegmentVerts + segmentCount;
				while (iVPtr != endVPtr)
				{
					segmentVerts.append(
						make_tuple(
							m_type(iVPtr[0], iVPtr[1], iVPtr[2]), 
							m_type(iVPtr[3], iVPtr[4], iVPtr[5])
						)
					);
					iVPtr += 6;
				}

				iPtr = pSegmentRefs;
				endPtr = pSegmentRefs + segmentCount;
				while (iPtr != endPtr)
					segmentRefs.append(*(iPtr++));
			}
			catch (error_already_set const &)
			{
				dtFree(pSegmentVerts);
				dtFree(pSegmentRefs);
				throw;
			}
		}

		dtFree(pSegmentVerts);
		dtFree(pSegmentRefs);
	}

	return make_tuple(status, make_tuple(segmentVerts, segmentRefs));
}

/* (dtStatus, (randomRef, randomPt)) */
tuple NavMeshQuery::findRandomPoint() const
{
	dtStatus status;
	dtPolyRef randomRef;
	float randomPt[3];

	randomRef = 0;
	randomPt[0] = 0.0f;
	randomPt[1] = 0.0f;
	randomPt[2] = 0.0f;

	status = m_navq->findRandomPoint(m_filter, 
		frand_01, &randomRef, randomPt);

	return make_tuple(status, make_tuple(randomRef, 
		m_type(randomPt[0], randomPt[1], randomPt[2])));
}

/* (dtStatus, (randomRef, randomPt)) */
tuple NavMeshQuery::findRandomPointAroundCircle(dtPolyRef startRef, 
	object centerPos, 
	const float maxRadius) const
{
	dtStatus status;
	dtPolyRef randomRef;
	float pCenterPos[3], randomPt[3];

	extractVector3(pCenterPos, centerPos);

	randomRef = 0;
	randomPt[0] = 0.0f;
	randomPt[1] = 0.0f;
	randomPt[2] = 0.0f;
	if (maxRadius > 0.0f)
	{
		status = m_navq->findRandomPointAroundCircle(startRef, 
			pCenterPos, maxRadius, m_filter, frand_01, &randomRef, randomPt);
	}
	else
	{
		status = DT_FAILURE | DT_INVALID_PARAM;
	}

	return make_tuple(status, make_tuple(randomRef, 
		m_type(randomPt[0], randomPt[1], randomPt[2])));
}

/* (dtStatus, closest) */
tuple NavMeshQuery::closestPointOnPoly(dtPolyRef startRef, 
	object pos) const
{
	dtStatus status;
	float pPos[3], pClosest[3];

	extractVector3(pPos, pos);

	pClosest[0] = 0.0f;
	pClosest[1] = 0.0f;
	pClosest[2] = 0.0f;

	status = m_navq->closestPointOnPoly(startRef, pPos, pClosest);
	return make_tuple(status, m_type(pClosest[0], pClosest[1], pClosest[2]));
}

/* (dtStatus, closest) */
tuple NavMeshQuery::closestPointOnPolyBoundary(dtPolyRef startRef, 
	object pos) const
{
	dtStatus status;
	float pPos[3], pClosest[3];

	extractVector3(pPos, pos);

	pClosest[0] = 0.0f;
	pClosest[1] = 0.0f;
	pClosest[2] = 0.0f;

	status = m_navq->closestPointOnPolyBoundary(startRef, pPos, pClosest);
	return make_tuple(status, m_type(pClosest[0], pClosest[1], pClosest[2]));
}

/* (dtStatus, height) */
tuple NavMeshQuery::getPolyHeight(dtPolyRef startRef, 
	object pos) const
{
	dtStatus status;
	float pPos[3], height;

	extractVector3(pPos, pos);

	height = 0.0f;

	status = m_navq->getPolyHeight(startRef, pPos, &height);
	return make_tuple(status, height);
}

