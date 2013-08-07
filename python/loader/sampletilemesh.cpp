/*
 * @summary: recast sampletilemesh loader
 * @date: 2013-03-17
 * @author: zl
 */

#include "config.h"
#include "tilemesh.h"
#include "sampletilemesh.h"
#include <fstream>

using std::ios_base;
using std::ifstream;


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

