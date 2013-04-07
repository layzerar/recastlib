cmake_minimum_required (VERSION 2.6)
project (RecastLib)
set (RecastLib_VERSION_MAJOR 0)
set (RecastLib_VERSION_MINOR 1)

find_path (Python_INCLUDE_DIR NAMES Python.h)
find_path (Python_LIBRARY_DIR NAMES python*.lib)
find_path (Boost_INCLUDE_DIR NAMES boost/python.hpp)
find_path (Boost_LIBRARY_DIR NAMES libboost_python*.lib)
set (Python_INCLUDE_DIRS ${Python_INCLUDE_DIR})
set (Python_LIBRARY_DIRS ${Python_LIBRARY_DIR})
set (Boost_INCLUDE_DIRS ${Boost_INCLUDE_DIR})
set (Boost_LIBRARY_DIRS ${Boost_LIBRARY_DIR})

include_directories (${Boost_INCLUDE_DIRS})
include_directories (${Python_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/Recast/DebugUtils/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/Detour/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/DetourCrowd/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/DetourTileCache/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/Recast/Include")
include_directories ("${PROJECT_SOURCE_DIR}/python")

link_directories (${Python_LIBRARY_DIRS})
link_directories (${Boost_LIBRARY_DIRS})

add_library (_recast SHARED
${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourAlloc.cpp
${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourCommon.cpp
${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNavMesh.cpp
${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNavMeshBuilder.cpp
${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNavMeshQuery.cpp
${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNode.cpp
${PROJECT_SOURCE_DIR}/python/detour/query.cpp
${PROJECT_SOURCE_DIR}/python/loader/sampletilemesh.cpp
${PROJECT_SOURCE_DIR}/python/detour.cpp
${PROJECT_SOURCE_DIR}/python/dtmath.cpp
${PROJECT_SOURCE_DIR}/python/loader.cpp
${PROJECT_SOURCE_DIR}/python/export.cpp
)

