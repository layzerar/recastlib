/*
 * @summary: recast loader for unity3d NavMesh.asset
 * @date: 2013-08-08
 * @author: zl
 */

#include "config.h"
#include "tilemesh.h"
#include "unitytilemesh.h"
#include <string>
#include <fstream>

using std::ios_base;
using std::ifstream;
using std::string;
using std::getline;

static const string entry1("TESM\x01\0\0\0");
static const string entry2("MSET\0\0\0\x01");


static int dtFindNavMeshEntry(ifstream& fp)
{
	int pos;
	string line;
	string::size_type off;

	while (fp)
	{
		pos = (int)fp.tellg();
		if (pos < 0)
			break;
		getline(fp, line, '\n');

		off = line.find(entry1);
		if (off == string::npos)
			continue;
		off = line.find(entry2);
		if (off == string::npos)
			continue;

		pos += (int)off;
		if (pos < 0)
			break;
		return pos;
	}

	return -1;
}


dtNavMesh* dtLoadUnityTileMesh(const char* path)
{
	ifstream fp(path, ios_base::in | ios_base::binary);

	int entrypos = dtFindNavMeshEntry(fp);
	if (entrypos < 0)
		return NULL;
	fp.seekg(entrypos);

	NavMeshSetHeader h;
	int swapped = dtLoadSetHeader(fp, h);
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

