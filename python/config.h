/*
 * @summary: recast config
 * @date: 2013-03-17
 * @author: zl
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// boost includes
#define BOOST_PYTHON_STATIC_LIB
#include <boost/python.hpp>
#include <boost/random.hpp>
#include <boost/foreach.hpp>
#ifndef FOREACH
#define FOREACH BOOST_FOREACH
#endif

// detour includes
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNode.h"
#include "DetourStatus.h"

#endif /* CONFIG_H_ */
