#coding=utf-8
u"""
 @summary: 
 @date: 2012-4-24
 @author: zl
"""

import detour as d
from vector import Vector3

x = d.dtNavMesh("nav_test.nv")
q = d.dtNavMeshQuery(x, Vector3)

polyPickExt = Vector3(2.0, 4.0, 2.0)

startPos = Vector3(6.054083, -2.365402, 3.330421)
endPos = Vector3(19.289761, -2.368813, -6.954918)

status, (startRef, _startPt) = q.findNearestPoly(startPos, polyPickExt)
if d.dtStatusFailed(status):
    raise Exception("error code: %d", status)
status, (endRef, _endPt) = q.findNearestPoly(endPos, polyPickExt)
if d.dtStatusFailed(status):
    raise Exception("error code: %d", status)
status, pathRefs = q.findPath(startRef, endRef, startPos, endPos, 32)
if d.dtStatusFailed(status):
    raise Exception("error code: %d", status)

status, fixEndPos = q.closestPointOnPoly(pathRefs[-1], endPos)
if d.dtStatusFailed(status):
    raise Exception("error code: %d", status)

staus, (straightPath, straightPathFlags, straightPathRefs) = \
q.findStraightPath(startPos, fixEndPos, pathRefs, 32)
if d.dtStatusFailed(status):
    raise Exception("error code: %d", status)
print "start pos: ", startPos
print "start pos (in poly): ", _startPt
print "end pos: ", endPos
print "end pos (in poly): ", _endPt
print "end pos (fixed): ", fixEndPos
print "the straight path:"
print hex(status)
print straightPath
print straightPathFlags
print straightPathRefs
