#coding=utf-8
u"""
 @summary: 
 @date: 2012-4-24
 @author: zl
"""

import recast as dt


def test_sample_tile_mesh():
    print "Create NavMesh Object From File"
    nav_mesh = dt.dtLoadSampleTileMesh("nav_test.nv")
    filter = dt.dtQueryFilter()
    query = dt.dtNavMeshQuery()

    print "Init NavMesh Object"
    status = query.init(nav_mesh, 2048)
    if dt.dtStatusFailed(status):
        return -1, status

    print "Fix The Input Data"
    polyPickExt = dt.dtVec3(2.0, 4.0, 2.0)
    startPos = dt.dtVec3(6.054083, -2.365402, 3.330421)
    endPos = dt.dtVec3(19.289761, -2.368813, -6.954918)

    status, out = query.findNearestPoly(startPos, polyPickExt, filter)
    if dt.dtStatusFailed(status):
        return -2, status
    startRef = out["nearestRef"]
    _startPt = out["nearestPt"]

    status, out = query.findNearestPoly(endPos, polyPickExt, filter)
    if dt.dtStatusFailed(status):
        return -3, status
    endRef = out["nearestRef"]
    _endPt = out["nearestPt"]

    print "Get Path Reference List"
    status, out = query.findPath(startRef, endRef, startPos, endPos, filter, 32)
    if dt.dtStatusFailed(status):
        return -4, status
    pathRefs = out["path"]

    status, fixEndPos = query.closestPointOnPoly(pathRefs[-1], endPos)
    if dt.dtStatusFailed(status):
        return -5, status

    print "Get Path Point List"
    status, out = query.findStraightPath(startPos, fixEndPos, pathRefs, 32, 0)
    if dt.dtStatusFailed(status):
        return -6, status
    straightPath = out["straightPath"]
    straightPathFlags = out["straightPathFlags"]
    straightPathRefs = out["straightPathRefs"]

    print "Print Search Result"
    print "The input data:"
    print "\tstart pos:", startPos
    print "\tend pos: ", endPos

    print "The fixed input data:"
    print "\tstart point(in poly):", _startPt
    print "\tend pos(in poly):", _endPt
    print "\tend pos(fixed):", fixEndPos

    print "The final output path:"
    print "\tstraight path:", straightPath
    print "\tstraight path flags:", straightPathFlags
    print "\tstraight path refs:", straightPathRefs
    return 0, 0


def main():
    rv, st = test_sample_tile_mesh()
    print "rv=%d, st=%d" % (rv, st)


if __name__ == '__main__':
    main()
    
    
    
