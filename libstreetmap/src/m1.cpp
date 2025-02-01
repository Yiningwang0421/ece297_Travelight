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
#include <vector>
#include <unordered_set>

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


// Global nested vector: Index is streetId, value is a vector of segment IDs
std::vector<std::vector<StreetSegmentIdx>> streetSegmentVector;
std::vector<std::pair<double, double>> segmentData;  //  First = length, Second = speed limit
std::vector<std::pair<LatLon, LatLon>> segmentLatLon;  // Stores (from, to) LatLon for each segment
std::vector<std::pair<double, double>> streetLat;  // Stores (min_lat, max_lat) for each street
std::vector<std::pair<double, double>> streetLon;  // Stores (min_lon, max_lon) for each street


void preprocessStreetSegments(); 



//global variables for function usage
std::vector<std::vector<StreetSegmentIdx>> intersection_street_segments; 
std::vector<std::vector<IntersectionIdx>> adjacent_street_segments;
bool loadMap(std::string map_streets_database_filename) {


    bool load_successful = loadStreetsDatabaseBIN(map_streets_database_filename); //Indicates whether the map has loaded 
                                  //successfully
    std::cout << "loadMap: " << map_streets_database_filename << std::endl;
    if(load_successful == false){
        return false;
    }
    //
    // Load your map related data structures here.
    //
    // findStreetSegmentsOfIntersection()
    intersection_street_segments.resize(getNumIntersections());
    adjacent_street_segments.resize(getNumIntersections());

    for(IntersectionIdx intersection_id = 0; intersection_id < getNumIntersections(); intersection_id++){
        int segments = getNumIntersectionStreetSegment(intersection_id);

        for(int i = 0; i < segments; i++){
            StreetSegmentIdx ss_id = getIntersectionStreetSegment(intersection_id, i); //finding  the streetsegment intersection
            intersection_street_segments[intersection_id].push_back(ss_id);
            StreetSegmentInfo ss_info = getStreetSegmentInfo(ss_id); // get each street's info
            IntersectionIdx adjacent = 0;
            bool uniqueAdjSegment = false;
            //finding the adjacent point for forming a vector
            if(ss_info.from == intersection_id){
                adjacent = ss_info.to;
            }
            else if(ss_info.to == intersection_id && !(ss_info.oneWay)){
                adjacent = ss_info.from;
            }
            else if(ss_info.to == intersection_id && ss_info.from == intersection_id){ //corner case for cul-de-sacs
                adjacent = ss_info.to;
            }
            //no  duplicate intersection
            if(std::find(adjacent_street_segments[intersection_id].begin(), adjacent_street_segments[intersection_id].end(), adjacent) == adjacent_street_segments[intersection_id].end()){
                uniqueAdjSegment = true;
            }
            if(adjacent != 0 && uniqueAdjSegment == true){
                adjacent_street_segments[intersection_id].push_back(adjacent);
            }
        }
    }



    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    preprocessStreetSegments(); 

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    intersection_street_segments.clear();
    closeStreetDatabase();
    streetSegmentVector.clear();
    segmentData.clear();
}

// Returns the distance between two (latitude, longitude) coordinates in meters.
// Speed Requirement --> moderate
double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2){

    
    // Convert latitude and longitude from degrees to radians
    double lat1 = point_1.latitude() * kDegreeToRadian;
    double lon1 = point_1.longitude() * kDegreeToRadian;
    double lat2 = point_2.latitude() * kDegreeToRadian;
    double lon2 = point_2.longitude() * kDegreeToRadian;

    // Compute the average latitude
    double lat_avg = (lat1 + lat2) / 2.0;

    // Compute x and y distances
    double x = kEarthRadiusInMeters * (lon2 - lon1) * std::cos(lat_avg);
    double y = kEarthRadiusInMeters * (lat2 - lat1);

    // Compute the distance using Pythagoras' theorem
    return std::sqrt(x * x + y * y);
}



//Returns the length of the given street segment in meters
//Speed Requirement --> moderate
double findStreetSegmentLength(StreetSegmentIdx street_segment_id){
    
    StreetSegmentInfo streetSegment = getStreetSegmentInfo(street_segment_id);


    // Get the Start Point
    LatLon startPoint = getIntersectionPosition(streetSegment.from);

    double totalLength = 0.0;
    

    // Iterate through curve points
    for (int i = 0; i < streetSegment.numCurvePoints; i++) {
        LatLon curvePoint = getStreetSegmentCurvePoint(street_segment_id, i);
        totalLength = totalLength + findDistanceBetweenTwoPoints(startPoint, curvePoint);
        startPoint = curvePoint;  
    }

    // Add final segment (last curve point → to intersection)
    LatLon endPoint = getIntersectionPosition(streetSegment.to);
    totalLength = totalLength + findDistanceBetweenTwoPoints(startPoint, endPoint);

    return totalLength;  

}

//Returns the travel time to drive a street segment in seconds
//(time = distance / speed_limit)
//Speed Requirement --> High
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id){
    double segmentLength = segmentData[street_segment_id].first;  
    double speedLimit = segmentData[street_segment_id].second;  
    return (speedLimit > 0) ? (segmentLength / speedLimit) : 0.0;  

}

//helper function for finding non straight street
LatLon getClosestSegment(StreetSegmentIdx segmentID, IntersectionIdx intersection){
    StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segmentID); //getting the first street segment information
    LatLon IntersectionPos = getIntersectionPosition(intersection);
    if(segmentInfo.numCurvePoints > 0){ 
        LatLon closestPT = getStreetSegmentCurvePoint(segmentID, 0); // setting a first segment to compare with
        double minimum = findDistanceBetweenTwoPoints(IntersectionPos, closestPT);
        for (int currID = 1; currID < segmentInfo.numCurvePoints; currID++){   // for the later curve point on segment
            LatLon curvePT = getStreetSegmentCurvePoint(segmentID, currID);
            double curveDis = findDistanceBetweenTwoPoints(IntersectionPos, curvePT);
            if(curveDis < minimum){
                minimum = curveDis;
                closestPT = curvePT;
            }
        }
        return closestPT; //the latest curvepoint returned
    }
    //straight street
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
    if(src_info.to == dst_info.from || src_info.to == dst_info.to){
        intersection = src_info.to; //any one as long as to and from has a POI
    }
    else if(src_info.from == dst_info.to || src_info.from == dst_info.from){
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
    double cos_theta = (a*a + b*b - c*c) / (2*a*b);
    // constrain the  range of cosine into [-1, 1] with correcting tiny out of range
    if(cos_theta > 1){
        cos_theta = 1;
    }
    else if(cos_theta < -1){
        cos_theta = -1;
    }
    double turnAngle = acos(cos_theta);
    return M_PI - turnAngle;
}

double findStreetLength(StreetIdx street_id){

    double totalLength = 0.0;
    if (street_id >= 0 && street_id < streetSegmentVector.size())
    {
        
        const std::vector<StreetSegmentIdx>& segmentsOfStreetId = streetSegmentVector[street_id];
        for (StreetSegmentIdx i = 0; i < segmentsOfStreetId.size(); i++) {
            
        totalLength += segmentData[segmentsOfStreetId[i]].first;
        
    }
}
    
    return totalLength;
}

double findFeatureArea(FeatureIdx feature_id){
    return 0.0;
}

double findWayLength(OSMID way_id){
    double totalLength = 0.0;

    const OSMWay* way = getWayByIndex(static_cast<int>(uint64_t(way_id)));  
    if (!way) return 0.0;

    const std::vector<OSMID>& wayNodes = getWayMembers(way);
    if (wayNodes.size() < 2) return 0.0;

    
    for (int i = 0; i < (wayNodes.size() - 1); i++) {
        LatLon point1 = getNodeCoords(getNodeByIndex(static_cast<int>(uint64_t(wayNodes[i]))));  
        LatLon point2 = getNodeCoords(getNodeByIndex(static_cast<int>(uint64_t(wayNodes[i+1]))));

        totalLength += findDistanceBetweenTwoPoints(point1, point2);
    }

    return totalLength;

}

LatLonBounds findStreetBoundingBox(StreetIdx street_id){
    LatLon minLatLon(streetLat[street_id].first, streetLon[street_id].first);
    LatLon maxLatLon(streetLat[street_id].second, streetLon[street_id].second);

    return {minLatLon, maxLatLon};
}

POIIdx findClosestPOI(LatLon my_position, std::string poi_type){
    double minimum = 100000;
    POIIdx POI_idx = -1;
    for(POIIdx count = 0; count < getNumPointsOfInterest(); count++){
        if(getPOIType(count) == poi_type){
            LatLon POI_pos = getPOIPosition(count);
            double curr_distance = findDistanceBetweenTwoPoints(POI_pos, my_position);
            if(curr_distance < minimum){
                minimum = curr_distance;
                POI_idx = count;
            }
        }
    }
    return POI_idx;
}

std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id){
    return adjacent_street_segments[intersection_id];
}

IntersectionIdx findClosestIntersection(LatLon my_position){
    double minimum = findDistanceBetweenTwoPoints(getIntersectionPosition(0), my_position);
    IntersectionIdx intersectID = -1;
    for(IntersectionIdx i = 1; i < getNumIntersections(); i++){
        LatLon intersectPos = getIntersectionPosition(i);
        double distance = findDistanceBetweenTwoPoints(intersectPos, my_position);
        if(distance < minimum){
            minimum = distance;
            intersectID = i;
        }
    }
    return intersectID;
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


void preprocessStreetSegments() {
    int numStreets = getNumStreets();
    streetSegmentVector.resize(numStreets);
    int numSegments = getNumStreetSegments();
    segmentData.resize(numSegments);

    segmentLatLon.resize(numSegments);
    
    streetLat.resize(numStreets, {10000000, -10000000});
    streetLon.resize(numStreets, {10000000, -10000000});

    // Loop through all street segments and store coresponding info in the corresponding vectors
    for (int segmentId = 0; segmentId < getNumStreetSegments(); segmentId++) {
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segmentId);
        streetSegmentVector[segmentInfo.streetID].push_back(segmentId);
////////////////////        
        double segmentLength = findStreetSegmentLength(segmentId);
        segmentData[segmentId] = {segmentLength, segmentInfo.speedLimit};
///////////////////
        LatLon fromPos = getIntersectionPosition(segmentInfo.from);
        LatLon toPos = getIntersectionPosition(segmentInfo.to);
        segmentLatLon[segmentId] = {fromPos, toPos};
////////////////////
        int streetId = segmentInfo.streetID;
        
        streetLat[streetId].first = std::min({streetLat[streetId].first, fromPos.latitude(), toPos.latitude()});
        streetLat[streetId].second = std::max({streetLat[streetId].second, fromPos.latitude(), toPos.latitude()});

        
        streetLon[streetId].first = std::min({streetLon[streetId].first, fromPos.longitude(), toPos.longitude()});
        streetLon[streetId].second = std::max({streetLon[streetId].second, fromPos.longitude(), toPos.longitude()});

        // Include curve points in min/max lat/lon calculations
        for (int i = 0; i < segmentInfo.numCurvePoints; i++) {
            LatLon curvePoint = getStreetSegmentCurvePoint(segmentId, i);

            // Update 
            streetLat[streetId].first = std::min(streetLat[streetId].first, curvePoint.latitude());
            streetLat[streetId].second = std::max(streetLat[streetId].second, curvePoint.latitude());

            streetLon[streetId].first = std::min(streetLon[streetId].first, curvePoint.longitude());
            streetLon[streetId].second = std::max(streetLon[streetId].second, curvePoint.longitude());
        }

    }
}