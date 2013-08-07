/*
 * @summary: recast tilemesh common
 * @date: 2013-08-08
 * @author: zl
 */

#ifndef TILEMESH_H_
#define TILEMESH_H_

#include "config.h"
#include <fstream>

using std::ifstream;


 /*
 * Loader for SampleTileMesh.
 * See "RecastDemo/Source/Sample_TileMesh.cpp" for more detail.
 *
 */

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

inline void dtSwapEndian(NavMeshSetHeader* h)
{
	dtNavMeshParams* p = &h->params;

	dtSwapEndian(&h->magic);
	dtSwapEndian(&h->version);
	dtSwapEndian(&h->numTiles);

	dtSwapEndian(&p->orig[0]);
	dtSwapEndian(&p->orig[1]);
	dtSwapEndian(&p->orig[2]);
	dtSwapEndian(&p->tileWidth);
	dtSwapEndian(&p->tileHeight);
	dtSwapEndian(&p->maxTiles);
	dtSwapEndian(&p->maxPolys);
}

inline void dtSwapEndian(NavMeshTileHeader* h)
{
	dtSwapEndian(&h->tileRef);
	dtSwapEndian(&h->dataSize);
}

int dtLoadSetHeader(ifstream& fp, NavMeshSetHeader& h);
int dtLoadTileData(ifstream& fp, dtNavMesh* m, unsigned char* data, NavMeshTileHeader& tileh, int swapped);
int dtLoadAllTiles(ifstream& fp, dtNavMesh* m, NavMeshSetHeader& h, int swapped);


#endif // TILEMESH_H_


