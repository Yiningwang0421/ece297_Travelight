/* 
 * Copyright 2025 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated 
 * documentation files (the "Software") in course work at the University 
 * of Toronto, or for personal use. Other uses are prohibited, in 
 * particular the distribution of the Software either publicly or to third 
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "math.h"

// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection 
// data that is higher-level than the raw OSM data). 
// This file name will always end in ".streets.bin" and you 
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1 
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the 
// name of the ".osm.bin" file that matches your map -- just change 
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.

//global variables for function usage
std::vector<std::vector<StreetSegmentIdx>> intersection_street_segments; 
std::vector<std::vector<IntersectionIdx>> adjacent_street_segments;
bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = loadStreetsDatabaseBIN(map_streets_database_filename); //Indicates whether the map has loaded 
                                  //successfully
    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

    //
    // Load your map related data structures here.
    //
    // findStreetSegmentsOfIntersection()
    intersection_street_segments.resize(getNumIntersections());
    adjacent_street_segments.resize(getNumIntersections());

    for(IntersectionIdx intersection_id = 0; intersection_id < getNumIntersections(); intersection_id++){
        int segments = getNumIntersectionStreetSegment(intersection_id);

        intersection_street_segments[intersection_id].reserve(segments);
        for(int i = 0; i < segments; i++){
            StreetSegmentIdx ss_id = getIntersectionStreetSegment(intersection_id, i);
            intersection_street_segments[intersection_id].push_back(ss_id);
            StreetSegmentInfo ss_info = getStreetSegmentInfo(ss_id); // get each street's info
            IntersectionIdx adjacent = 0;
            if(ss_info.from == intersection_id){
                adjacent = ss_info.to;
            }
            else if(ss_info.to == intersection_id && !(ss_info.oneWay)){
                adjacent = ss_info.from;
            }
            if(adjacent != 0 && std::find(adjacent_street_segments[intersection_id].begin(), 
                                            adjacent_street_segments[intersection_id].end(),
                                            adjacent) ==  adjacent_street_segments[intersection_id].end()){
                adjacent_street_segments[intersection_id].push_back(adjacent);
            }
        }
    }

    // findAdjacentIntersections



    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    intersection_street_segments.clear();
    closeStreetDatabase();
}

// Returns the distance between two (latitude, longitude) coordinates in meters.
// Speed Requirement --> moderate
double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2){
    return 0.0;
}

//Returns the length of the given street segment in meters
//Speed Requirement --> moderate
double findStreetSegmentLength(StreetSegmentIdx street_segment_id){
    return 0.0;
}

//Returns the travel time to drive a street segment in seconds
//(time = distance / speed_limit)
//Speed Requirement --> High
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    return 0.0;
}

//helper function for finding non straight street
LatLon getClosestSegment(StreetSegmentIdx segmentID, IntersectionIdx intersection){
    StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segmentID);
    if(segmentInfo.numCurvePoints > 0){ // curved street
        return getStreetSegmentCurvePoint(segmentID, segmentInfo.numCurvePoints - 1);
    }

    if(segmentInfo.to == intersection){
        return getIntersectionPosition(segmentInfo.from);
    }
    else{
        return getIntersectionPosition(segmentInfo.to);
    }

}
double findStreetSegmentTurnAngle(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_id){
    StreetSegmentInfo src_info = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo dst_info = getStreetSegmentInfo(dst_street_id);
    IntersectionIdx intersection = -1;
    // checking intersection category
    if(src_info.to == dst_info.from){
        intersection = src_info.to;
    }
    else if(src_info.from == dst_info.to){
        intersection = src_info.from;
    }
    else if(src_info.to == dst_info.to){
        intersection = src_info.to;
    }
    else if(src_info.from == dst_info.from){
        intersection = src_info.from;
    }
    else{
        return NO_ANGLE;
    }

    //finding each street segment direction
    LatLon intersectionPos = getIntersectionPosition(intersection);
    LatLon srcPT = getClosestSegment(src_street_segment_id, intersection);
    LatLon dstPT = getClosestSegment(dst_street_id,  intersection);
    //finding directional vector for each streetsegment
    double a = findDistanceBetweenTwoPoints(intersectionPos, srcPT);
    double b = findDistanceBetweenTwoPoints(intersectionPos, dstPT);
    double c = findDistanceBetweenTwoPoints(srcPT, dstPT);
    double turnAngle = acos((a*a + b*b - c*c) / (2*a*b));
    return turnAngle;
}

double findStreetLength(StreetIdx street_id){
    return 0.0;
}

double findFeatureArea(FeatureIdx feature_id){
    return 0.0;
}

double findWayLength(OSMID way_id){
    return 0.0;
}

LatLonBounds findStreetBoundingBox(StreetIdx street_id){
    return LatLonBounds();
}

POIIdx findClosestPOI(LatLon my_position, std::string poi_type){
    return POIIdx();
}

std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id){
    return adjacent_street_segments[intersection_id];
}

IntersectionIdx findClosestIntersection(LatLon my_position){
    return IntersectionIdx(-1);
}

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection (IntersectionIdx intersection_id) {
    return intersection_street_segments[intersection_id];
}

// Returns all intersections along the given street.
// There should be no duplicate intersections in the returned vector.
// Speed Requirement --> high
std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
    return std::vector<IntersectionIdx>();
}

// Return all intersection ids at which the two given streets intersect.
// This function will typically return one intersection id for streets that
// intersect and a length 0 vector for streets that do not. For unusual curved
// streets it is possible to have more than one intersection at which two
// streets cross.
// There should be no duplicate intersections in the returned vector.
// Speed Requirement --> high
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids){
    return std::vector<IntersectionIdx>();
}

// Returns all street ids corresponding to street names that start with the
// given prefix.
// The function should be case-insensitive to the street prefix.
// The function should ignore spaces.
// For example, both "bloor " and "BloOrst" are prefixes to
// "Bloor Street East".
// If no street names match the given prefix, this routine returns an empty
// (length 0) vector.
// You can choose what to return if the street prefix passed in is an empty
// (length 0) string, but your program must not crash if street_prefix is a
// length 0 string.
// Speed Requirement --> high
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix){
    return std::vector<StreetIdx>();
}

std::string getOSMNodeTagValue(OSMID osm_id, std::string key){
    return std::string();
}
