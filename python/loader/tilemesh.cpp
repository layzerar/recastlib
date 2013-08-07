/*
 * @summary: recast tilemesh common
 * @date: 2013-08-08
 * @author: zl
 */

#include "config.h"
#include "tilemesh.h"
#include <fstream>

using std::ios_base;
using std::ifstream;

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;


int dtLoadSetHeader(ifstream& fp, NavMeshSetHeader& h)
{
	// read header
	fp.read((char*)&h, sizeof(NavMeshSetHeader));
	if (!fp)
		return -11;

	// check the endian and version of header
	int swapped = 0;
	if (h.magic != NAVMESHSET_MAGIC)
	{
		swapped = 1;
		dtSwapEndian(&h);
		if (h.magic != NAVMESHSET_MAGIC)
			return -12;
	}
	if (h.version != NAVMESHSET_VERSION)
		return -13;

	return swapped;
}

int dtLoadTileData(ifstream& fp, dtNavMesh* m, unsigned char* data, NavMeshTileHeader& tileh, int swapped)
{
	fp.read((char*)data, tileh.dataSize);
	if (!fp)
		return -31;

	if (swapped)
	{
		if (!dtNavMeshHeaderSwapEndian(data, tileh.dataSize))
			return -32;
		if (!dtNavMeshDataSwapEndian(data, tileh.dataSize))
			return -33;
	}

	dtStatus status = m->addTile(data, tileh.dataSize, DT_TILE_FREE_DATA, tileh.tileRef, 0);
	if (dtStatusFailed(status))
		return -34;

	return 0;
}

int dtLoadAllTiles(ifstream& fp, dtNavMesh* m, NavMeshSetHeader& h, int swapped)
{
	// initialize mesh data
	dtStatus status = m->init(&h.params);
	if (dtStatusFailed(status))
		return -21;

	// read tiles.
	for (int i = 0; i < h.numTiles; ++i)
	{
		NavMeshTileHeader tileh;

		fp.read((char*)&tileh, sizeof(NavMeshTileHeader));
		if (!fp)
			return -22;

		if (swapped)
		{
			dtSwapEndian(&tileh);
		}
		if (tileh.dataSize <= 0 || !tileh.tileRef)
			return -23;

		unsigned char* data = (unsigned char*)dtAlloc(tileh.dataSize, DT_ALLOC_PERM);
		if (!data)
			return -24;

		int ret = dtLoadTileData(fp, m, data, tileh, swapped);
		if (ret < 0)
		{
			dtFree(data);
			return ret;
		}
	}

	return 0;
}

