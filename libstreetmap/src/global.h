/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/file.h to edit this template
 */

/* 
 * File:   global.h
 * Author: wang4543
 *
 * Created on March 19, 2025, 10:37 p.m.
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <iostream>
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "math.h"
#include <vector>
#include <unordered_set>
#include <utility>
#include <unordered_map>
#include <string>
#include <cctype>
#include <map>
#include <cmath>
#include <queue>
#include <list>


extern double max_speed;
extern std::vector<std::vector<std::pair<IntersectionIdx, StreetSegmentIdx>>> outgoingInfo;
extern std::vector<LatLon> intersection_positions;
extern std::vector<double> segment_travel_time;

#endif /* GLOBAL_H */

