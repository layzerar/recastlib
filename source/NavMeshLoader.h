#ifndef NAVMESHLOADER_H_
#define NAVMESHLOADER_H_

#include "detour.h"
#include "NavMeshQuery.h"

#include <string>
#include <stdexcept>


using std::string;
using std::exception;
using std::overflow_error;
using std::bad_alloc;
using std::invalid_argument;


class io_err : public exception
{
public:
	explicit io_err(const string& _Message)
		: exception(_Message.c_str())
	{	// construct from message string
	}

	explicit io_err(const char *_Message)
		: exception(_Message)
	{	// construct from message string
	}
};

class data_err : public exception
{
public:
	explicit data_err(const string& _Message)
		: exception(_Message.c_str())
	{	// construct from message string
	}

	explicit data_err(const char *_Message)
		: exception(_Message)
	{	// construct from message string
	}
};


extern dtNavMesh* loadNavMesh(const char* path);
extern NavMeshQuery* loadNavMeshQuery(dtNavMesh* navMesh, object type);


#endif /* NAVMESHLOADER_H_ */
