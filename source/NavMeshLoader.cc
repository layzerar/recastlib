#include "NavMeshQuery.h"
#include "NavMeshLoader.h"


#include <new.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdexcept>


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
	dtNavMesh* navMesh;

	if (!path)
		throw invalid_argument("path invalid");

	navMesh = dtAllocNavMesh();
	if (!navMesh)
		throw bad_alloc();

	FILE* fp = fopen(path, "rb");
	if (!fp)
		throw io_err("file not exists");

	// Read header.
	bool endianess = false;
	size_t dataSize = 0;
	NavMeshSetHeader header;

	dataSize = sizeof(NavMeshSetHeader);
	if (fread(&header, dataSize, 1, fp) != dataSize)
	{
		fclose(fp);
		throw data_err("larger mesh-data expected");
	}

	if (header.magic != NAVMESHSET_MAGIC)
	{
		if (!dtNavMeshSetHeaderSwapEndian((unsigned char*)&header, dataSize))
		{
			fclose(fp);
			throw data_err("format of mesh-data unexpected");
		}
		endianess = true;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		throw data_err("format of mesh-data unexpected");
	}

	dtStatus status = navMesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		throw exception("unknown", status);
	}

	// Read tiles.
	dataSize = sizeof(NavMeshTileHeader);
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;

		if (fread(&tileHeader, dataSize, 1, fp) != dataSize)
		{
			fclose(fp);
			throw data_err("larger mesh-data expected");
		}
		if (endianess && !dtNavMeshTileHeaderSwapEndian(
				(unsigned char*)&tileHeader,
				dataSize))
		{
			fclose(fp);
			throw data_err("format of mesh-data unexpected");
		}
		if (tileHeader.dataSize < 0 || !tileHeader.tileRef)
		{
			fclose(fp);
			throw data_err("format of mesh-data unexpected");
		}

		unsigned char* data = (unsigned char*)dtAlloc(
				tileHeader.dataSize,
				DT_ALLOC_PERM);
		if (!data)
		{
			fclose(fp);
			throw bad_alloc();
		}
		memset(data, 0, tileHeader.dataSize);

		if (fread(data, tileHeader.dataSize, 1, fp) != tileHeader.dataSize)
		{
			fclose(fp);
			throw data_err("larger mesh-data expected");
		}

		if (endianess && !(dtNavMeshHeaderSwapEndian(data, tileHeader.dataSize)
				&& dtNavMeshDataSwapEndian(data, tileHeader.dataSize))
			)
		{
			fclose(fp);
			throw data_err("larger mesh-data expected");
		}

		status = navMesh->addTile(data,
				tileHeader.dataSize,
				DT_TILE_FREE_DATA,
				tileHeader.tileRef,
				0);
		if (dtStatusFailed(status))
		{
			fclose(fp);
			throw exception("unknown", status);
		}
	}

	fclose(fp);
	return navMesh;
}

NavMeshQuery* loadNavMeshQuery(dtNavMesh* navMesh, object type)
{
	dtQueryFilter* filter;
	dtNavMeshQuery* navq;
	NavMeshQuery* wnavq;

	type(0.0f, 0.0f, 0.0f);

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

	wnavq = new NavMeshQuery(filter, navq, type);
	if (!wnavq)
	{
		dtFreeQueryFilter(filter);
		dtFreeNavMeshQuery(navq);
		throw bad_alloc();
	}

	return wnavq;
}



