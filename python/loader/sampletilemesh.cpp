/*
 * @summary: recast sampletilemesh loader
 * @date: 2013-03-17
 * @author: zl
 */


#include "config.h"
#include <fstream>

using std::ios_base;
using std::ifstream;


/*
 * Loader for SampleTileMesh.
 * See "RecastDemo/Source/Sample_TileMesh.cpp" for more detail.
 *
 */

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

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

static int dtLoadSetHeader(ifstream& fp, NavMeshSetHeader& h)
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

static int dtLoadTileData(ifstream& fp, dtNavMesh* m, unsigned char* data, NavMeshTileHeader& tileh, int swapped)
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

static int dtLoadAllTiles(ifstream& fp, dtNavMesh* m, NavMeshSetHeader& h, int swapped)
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

dtNavMesh* dtLoadSampleTileMesh(const char* path)
{
	ifstream fp(path, ios_base::in | ios_base::binary);

	int swapped = 0;
	NavMeshSetHeader h;

	swapped = dtLoadSetHeader(fp, h);
	if (swapped < 0)
		return NULL;

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
		return NULL;

	int ret = dtLoadAllTiles(fp, mesh, h, swapped);
	if (ret < 0)
	{
		dtFreeNavMesh(mesh);
		return NULL;
	}

	return mesh;
}

