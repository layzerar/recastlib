#include "NavMeshQuery.h"
#include "NavMeshLoader.h"


#include <new.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdexcept>
#include <fstream>

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_MAGIC_REV = 'T'<<24 | 'E'<<16 | 'S'<<8 | 'M'; //'TESM';
static const int NAVMESHSET_VERSION = 0x00000001;
static const int NAVMESHSET_VERSION_REV = 0x01000000;

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

static bool
dtNavMeshSetHeaderSwapEndian(unsigned char* data, const int dataSize)
{
	NavMeshSetHeader* header = (NavMeshSetHeader*)data;

	dtSwapEndian(&header->magic);
	dtSwapEndian(&header->version);

	if ((header->magic != NAVMESHSET_MAGIC
			|| header->version != NAVMESHSET_VERSION)
		&& (header->magic != NAVMESHSET_MAGIC_REV
			|| header->version != NAVMESHSET_VERSION_REV))
		{
			return false;
		}
	dtNavMeshParams* params = &header->params;

	dtSwapEndian(&header->numTiles);

	dtSwapEndian(&params->orig[0]);
	dtSwapEndian(&params->orig[1]);
	dtSwapEndian(&params->orig[2]);
	dtSwapEndian(&params->tileWidth);
	dtSwapEndian(&params->tileHeight);
	dtSwapEndian(&params->maxTiles);
	dtSwapEndian(&params->maxPolys);
	return true;
}

static bool
dtNavMeshTileHeaderSwapEndian(unsigned char* data, const int dataSize)
{
	NavMeshTileHeader* header = (NavMeshTileHeader*)data;

	dtSwapEndian(&header->tileRef);
	dtSwapEndian(&header->dataSize);
	return true;
}

dtNavMesh* loadNavMesh(const char* path)
{
	using std::ios_base;
	using std::ifstream;

	dtNavMesh* navMesh;
	ifstream fp(path, ios_base::in | ios_base::binary);

	if (!fp.is_open())
	{
		throw io_err("file not exists");
	}

	// Read header.
	bool endianess = false;
	size_t dataSize = 0;
	NavMeshSetHeader header;

	dataSize = sizeof(NavMeshSetHeader);
	fp.read((char*)&header, dataSize);
	if (!fp.good() || fp.gcount() != dataSize)
		throw data_err("larger mesh-data expected");

	if (header.magic != NAVMESHSET_MAGIC)
	{
		if (!dtNavMeshSetHeaderSwapEndian((unsigned char*)&header, dataSize))
			throw data_err("format of mesh-data unexpected");

		endianess = true;
	}
	if (header.version != NAVMESHSET_VERSION)
		throw data_err("format of mesh-data unexpected");

	navMesh = dtAllocNavMesh();
	if (!navMesh)
		throw bad_alloc();

	dtStatus status = navMesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		dtFreeNavMesh(navMesh);
		throw exception("unknown", status);
	}

	// Read tiles.
	dataSize = sizeof(NavMeshTileHeader);
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;

		fp.read((char*)&tileHeader, dataSize);
		if (!fp.good() || fp.gcount() != dataSize)
		{
			dtFreeNavMesh(navMesh);
			throw data_err("larger mesh-data expected");
		}
		if (endianess && !dtNavMeshTileHeaderSwapEndian(
				(unsigned char*)&tileHeader,
				dataSize))
		{
			dtFreeNavMesh(navMesh);
			throw data_err("format of mesh-data unexpected");
		}
		if (tileHeader.dataSize < 0 || !tileHeader.tileRef)
		{
			dtFreeNavMesh(navMesh);
			throw data_err("format of mesh-data unexpected");
		}

		unsigned char* data = (unsigned char*)dtAlloc(
				tileHeader.dataSize,
				DT_ALLOC_PERM);
		if (!data)
		{
			dtFreeNavMesh(navMesh);
			throw bad_alloc();
		}

		fp.read((char*)data, tileHeader.dataSize);
		if (!fp.good() || fp.gcount() != tileHeader.dataSize)
		{
			dtFree(data);
			dtFreeNavMesh(navMesh);
			throw data_err("larger mesh-data expected");
		}

		if (endianess && !(dtNavMeshHeaderSwapEndian(data, tileHeader.dataSize)
				&& dtNavMeshDataSwapEndian(data, tileHeader.dataSize))
			)
		{
			dtFree(data);
			dtFreeNavMesh(navMesh);
			throw data_err("format of mesh-data unexpected");
		}

		status = navMesh->addTile(data, 
				tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
		if (dtStatusFailed(status))
		{
			dtFree(data);
			dtFreeNavMesh(navMesh);
			throw exception("unknown", status);
		}
	}

	return navMesh;
}

NavMeshQuery* loadNavMeshQuery(dtNavMesh* navMesh, object type)
{
	dtQueryFilter* filter;
	dtNavMeshQuery* navq;
	NavMeshQuery* wnavq;

	type(0.0f, 0.0f, 0.0f); // test the constructor of type

	filter = dtAllocQueryFilter();
	if (!filter)
	{
		throw bad_alloc();
	}

	navq = dtAllocNavMeshQuery();
	if (!navq)
	{
		dtFreeQueryFilter(filter);
		throw bad_alloc();
	}

	dtStatus status = navq->init(navMesh, 2048); // TODO: maxNodes ?
	if (dtStatusFailed(status))
	{
		dtFreeQueryFilter(filter);
		dtFreeNavMeshQuery(navq);
		throw exception("unknown", status);
	}

	wnavq = new NavMeshQuery(filter, navq, type);
	if (!wnavq)
	{
		dtFreeQueryFilter(filter);
		dtFreeNavMeshQuery(navq);
		throw bad_alloc();
	}

	return wnavq;
}

