// Copyright (c) 2007-2015 by Space-Time Engineering, LLC ("STE").
// All Rights Reserved.
//
// This source code is a part of Scenargie Software ("Software") and is
// subject to STE Software License Agreement. The information contained
// herein is considered a trade secret of STE, and may not be used as
// the basis for any other software, hardware, product or service.
//
// Refer to license.txt for more specific directives.

#ifndef SCENSIM_TERRAIN_H
#define SCENSIM_TERRAIN_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <set>

#include <assert.h>

using std::vector;
using std::set;
using std::string;

namespace ScenSim {

class TerrainDatabase {
public:
    TerrainDatabase(
    double initDeltaX = 50, double initDeltaY = 50,
    int initNumPointsX = 1600, int initNumPointsY = 1600,
    double initOffsetX = 0, double initOffsetY = 0,
    double initlbx = 0, double initlby = 0,
    double initrtx = 0, double initrty = 0);

    ~TerrainDatabase(){}

    void LoadTerrainDataFromFile(const string& fileName);
    double GetPositionHeightXY(const double &x, const double &y);
    bool HasTerrainDatabase() { return hasTerrainDatabase; }
private:
    bool hasTerrainDatabase;
    vector<vector<double> > height;
    set<string> loadedFileNames;
    double deltaX, deltaY;
    int numPointsX, numPointsY;
    double offsetX, offsetY;
    double leftBottomX, leftBottomY;
    double rightTopX, rightTopY;
    double leftBottomLong, leftBottomLat;
    double rightTopLong, rightTopLat;
    double maxHeight, minHeight;
};



inline
TerrainDatabase::TerrainDatabase(
    double initDeltaX, double initDeltaY,
    int initNumPointsX, int initNumPointsY,
    double initOffsetX, double initOffsetY,
    double initlbx, double initlby,
    double initrtx, double initrty):
                deltaX(initDeltaX), deltaY(initDeltaY), 
                numPointsX(initNumPointsX), numPointsY(initNumPointsY),
                offsetX(initOffsetX), offsetY(initOffsetY),
                leftBottomX(initlbx),leftBottomY(initlby),
                rightTopX(initrtx), rightTopY(initrty)
{
    height.resize(numPointsX, vector<double>(numPointsY));
    for(int i = 0; i < numPointsX; i++) {
        for(int j = 0; j < numPointsY; j++) {
            height[i][j] = 0;
        }
    }
}


inline
double TerrainDatabase::GetPositionHeightXY(const double &x, const double &y)
{
    assert((*this).hasTerrainDatabase);
    int xid, yid;
    xid = static_cast<int>(((x - offsetX) - deltaX / 2.0) / deltaX);
    yid = static_cast<int>(((y - offsetY) - deltaY / 2.0) / deltaY);
    return height[xid][yid];
}

}
#endif //SCENSIM_TERRAIN_H
