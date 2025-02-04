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
#include <utility>
#include <unordered_map>
#include <string>
#include <cctype>
#include <map>



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

std::vector<std::vector<IntersectionIdx>> intersectionVector;
std::map<std::string, std::vector<StreetIdx>> streetNameCollections;
std::vector<std::pair<double, double>> segmentData;  //  First = length, Second = speed limit
std::vector<std::pair<LatLon, LatLon>> segmentLatLon;  // Stores (from, to) LatLon for each segment
std::vector<std::pair<double, double>> streetLat;  // Stores (min_lat, max_lat) for each street
std::vector<std::pair<double, double>> streetLon;  // Stores (min_lon, max_lon) for each street


void preprocessStreetSegments(); 
void preprocessStreets();

std::unordered_map<OSMID, int> osmidToNodeMap;
std::unordered_map<OSMID, int> osmidToWayMap;
std::vector<std::vector<int>> convertedWayIndex;  // Stores node indices 

LatLon getClosestSegment(StreetSegmentIdx segmentID, IntersectionIdx intersection);
void preprocessMappings();
int getNodeIndexFromOSMID(OSMID node_id);
int getWayIndexFromOSMID(OSMID way_id);

//global variables for function usage
std::vector<std::vector<StreetSegmentIdx>> intersection_street_segments;
std::vector<std::vector<IntersectionIdx>> adjacent_street_segments;

std::unordered_map<OSMID, std::unordered_map<std::string, std::string>> OSMvec;
bool loadMap(std::string map_streets_database_filename) {




    bool load_successful = loadStreetsDatabaseBIN(map_streets_database_filename); //Indicates whether the map has loaded
                                                                                  //successfully
    std::cout << "loadMap: " << map_streets_database_filename << std::endl;
    if (load_successful == false)
    {
        return false;
    }
    //
    // Load your map related data structures here.
    //
    std::string osm_mapfilename = map_streets_database_filename;
    osm_mapfilename.replace(osm_mapfilename.find(".street"), 8, ".osm");
    bool osmload_successful = loadOSMDatabaseBIN(osm_mapfilename);
    std::cout<<"loadMap: "<<  osm_mapfilename << std::endl;
    if(osmload_successful == false){
        return false;
    }

    intersection_street_segments.resize(getNumIntersections());
    adjacent_street_segments.resize(getNumIntersections());

    for (IntersectionIdx intersection_id = 0; intersection_id < getNumIntersections(); intersection_id++)
    {
        int segments = getNumIntersectionStreetSegment(intersection_id);

        for(int i = 0; i < segments; i++){
            StreetSegmentIdx ss_id = getIntersectionStreetSegment(intersection_id, i); //finding  the streetsegment intersection
            intersection_street_segments[intersection_id].push_back(ss_id);
            StreetSegmentInfo ss_info = getStreetSegmentInfo(ss_id); // get each street's info
            IntersectionIdx adjacent = 0;
            bool uniqueAdjSegment = false;
            //finding the adjacent point for forming a vector
            if (ss_info.from == intersection_id)
            {
                adjacent = ss_info.to;
            }
            else if (ss_info.to == intersection_id && !(ss_info.oneWay))
            {
                adjacent = ss_info.from;
            }
            else if (ss_info.to == intersection_id && ss_info.from == intersection_id)
            { //corner case for cul-de-sacs
                adjacent = ss_info.to;
            }

            //no duplicate happens
            if(std::find(adjacent_street_segments[intersection_id].begin(), adjacent_street_segments[intersection_id].end(), adjacent) == adjacent_street_segments[intersection_id].end()){
                uniqueAdjSegment = true;
            }
            if (adjacent != 0 && uniqueAdjSegment == true)
            {
                adjacent_street_segments[intersection_id].push_back(adjacent);
            }
        }
    }
    
    
    for(int i = 0; i < getNumberOfNodes(); i++){
        const OSMNode* node = getNodeByIndex(i);
        OSMID nodeId = node ->  id();
        std::unordered_map<std::string, std::string> storeTag;
        int nodeTag= getTagCount(node);

        for(int j = 0; j < nodeTag; j++){
            std::pair<std::string, std::string> tagPair =  getTagPair(node, j);
            storeTag[tagPair.first] = tagPair.second; //letting the index spot at map gets the value
        }
        OSMvec[nodeId] = storeTag;
    }


    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed
    

    preprocessStreetSegments();
    preprocessMappings();
    preprocessStreets();

    return load_successful;
}

void closeMap()
{
    //Clean-up your map related data structures here
    intersection_street_segments.clear();
    adjacent_street_segments.clear();
    OSMvec.clear();
    closeStreetDatabase();
    closeOSMDatabase();
    
    streetSegmentVector.clear();
    segmentData.clear();
    segmentLatLon.clear();
    streetLat.clear();
    streetLon.clear();
    
    osmidToNodeMap.clear();
    osmidToWayMap.clear();
    convertedWayIndex.clear();

}

// Returns the distance between two (latitude, longitude) coordinates in meters.
// Speed Requirement --> moderate
double findDistanceBetweenTwoPoints(LatLon point_1, LatLon point_2)
{

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
double findStreetSegmentLength(StreetSegmentIdx street_segment_id)
{

    StreetSegmentInfo streetSegment = getStreetSegmentInfo(street_segment_id);

    // Get the Start Point
    LatLon startPoint = getIntersectionPosition(streetSegment.from);

    double totalLength = 0.0;

    // Iterate through curve points
    for (int i = 0; i < streetSegment.numCurvePoints; i++)
    {
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
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id)
{
    double segmentLength = segmentData[street_segment_id].first;
    double speedLimit = segmentData[street_segment_id].second;
    return (speedLimit > 0) ? (segmentLength / speedLimit) : 0.0;
}


double findStreetSegmentTurnAngle(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_id)
{
    StreetSegmentInfo src_info = getStreetSegmentInfo(src_street_segment_id);
    StreetSegmentInfo dst_info = getStreetSegmentInfo(dst_street_id);
    IntersectionIdx intersection = -1;
    // checking intersection category
    if (src_info.to == dst_info.from  || src_info.to ==  dst_info.to){ //head intersection
        intersection =  src_info.to;
    }
    else if (src_info.from == dst_info.to || src_info.from == dst_info.from) //end intersection
    {
        intersection = src_info.from;
    }
    else
    {
        return NO_ANGLE;
    }

    //finding each street segment direction
    LatLon intersectionPos = getIntersectionPosition(intersection);
    LatLon srcPT, dstPT;
    if(src_info.numCurvePoints > 0){ //if having intersection and have curve point on src
        if(src_info.to == intersection){ //find closest curvepoint from end
            srcPT = getStreetSegmentCurvePoint(src_street_segment_id, src_info.numCurvePoints - 1);
        }
        else{ //find closest curvepoint from the head
            srcPT = getStreetSegmentCurvePoint(src_street_segment_id, 0);
        }
    }
    else{ //straight src street
        if(src_info.to == intersection){
            srcPT = getIntersectionPosition(src_info.from);
        }
        else{
            srcPT = getIntersectionPosition(src_info.to);
        }
    }

    if(dst_info.numCurvePoints > 0){ //if  having intersection around dst, check the closest curvepoint
        if(dst_info.to == intersection){ // if intersect at the dst end
            dstPT = getStreetSegmentCurvePoint(dst_street_id, dst_info.numCurvePoints - 1);
        }
        else{ // if intersect from the head
            dstPT = getStreetSegmentCurvePoint(dst_street_id, 0); 
        }
    }
    else{ //straight dst street
        if(dst_info.to == intersection){
            dstPT = getIntersectionPosition(dst_info.from);
        }
        else{
            dstPT = getIntersectionPosition(dst_info.to);
        }
    }

    //finding distance for street segment at an intersection
    double a = findDistanceBetweenTwoPoints(intersectionPos, srcPT);
    double b = findDistanceBetweenTwoPoints(intersectionPos, dstPT);
    double c = findDistanceBetweenTwoPoints(srcPT, dstPT);
    double cos_theta = (a*a + b*b  - c*c) / (2*a*b);;
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

double findStreetLength(StreetIdx street_id) {
    double totalLength = 0.0;

    if (street_id < 0 || street_id >= (streetSegmentVector.size())){
        return 0.0;
    }

    const std::vector<StreetSegmentIdx>& segmentsOfStreetId = streetSegmentVector[street_id];

    for (int i = 0; i < segmentsOfStreetId.size(); i++) {
        StreetSegmentIdx segmentId = segmentsOfStreetId[i];

        // Ensure segmentIdx is valid 
        if (segmentId < 0 || segmentId >= (segmentData.size())) {
            continue;
        }

        totalLength += segmentData[segmentId].first;
    }

    return totalLength;
}

double findFeatureArea(FeatureIdx feature_id) {
    int numOfPoints = getNumFeaturePoints(feature_id);
    double area = 0.0;
    
    //Check if the feature is closed
    if (numOfPoints >= 3 && (getFeaturePoint(feature_id,0).latitude() == getFeaturePoint(feature_id, numOfPoints-1).latitude())
            && (getFeaturePoint(feature_id,0).longitude() == getFeaturePoint(feature_id, numOfPoints-1).longitude())) {
        
        //Calculate the average latitude of the feature
        double avgLat = 0;
        for (int i=0; i<numOfPoints-1; i++){
            avgLat = avgLat + getFeaturePoint(feature_id, i).latitude();
        }
        avgLat = avgLat/(numOfPoints - 1);
        
        //Calculate the area of the feature using trapezoid formula
        double xi = 0.0;
        double x2 = 0.0;
        double yi = 0.0;
        double y2 = 0.0;
        for (int i = 0; i < numOfPoints - 1; i++) {
            xi = kEarthRadiusInMeters * getFeaturePoint(feature_id, i).longitude() * cos(kDegreeToRadian * avgLat);
            yi = kEarthRadiusInMeters * getFeaturePoint(feature_id, i).latitude();
            x2 = kEarthRadiusInMeters * getFeaturePoint(feature_id, i + 1).longitude() * cos(kDegreeToRadian * avgLat);
            y2 = kEarthRadiusInMeters * getFeaturePoint(feature_id, i + 1).latitude();
            area = area + 0.5 * (yi + y2)*(xi - x2)/3282.81;
        }
    }
    return abs(area);
}


double findWayLength(OSMID way_id) {
    int wayIndex = getWayIndexFromOSMID(way_id);
    if (wayIndex == -1) return 0.0;  

    const std::vector<int>& nodeIndices = convertedWayIndex[wayIndex];
    
    if (nodeIndices.size() < 2) return 0.0;  // Not enough nodes

    double totalLength = 0.0;

    for (int i = 1; i < nodeIndices.size(); i++) {
        totalLength += findDistanceBetweenTwoPoints(
            getNodeCoords(getNodeByIndex(nodeIndices[i - 1])),
            getNodeCoords(getNodeByIndex(nodeIndices[i]))
        );
    }

    return totalLength;
}
LatLonBounds findStreetBoundingBox(StreetIdx street_id)
{
    LatLon minLatLon(streetLat[street_id].first, streetLon[street_id].first);
    LatLon maxLatLon(streetLat[street_id].second, streetLon[street_id].second);

    return {minLatLon, maxLatLon};
}

POIIdx findClosestPOI(LatLon my_position, std::string poi_type)
{
    double minimum = 100000;
    POIIdx POI_idx = -1;
    for (POIIdx count = 0; count < getNumPointsOfInterest(); count++)
    {
        if (getPOIType(count) == poi_type)
        {
            LatLon POI_pos = getPOIPosition(count);
            double curr_distance = findDistanceBetweenTwoPoints(POI_pos, my_position);
            if (curr_distance < minimum)
            {
                minimum = curr_distance;
                POI_idx = count;
            }
        }
    }
    return POI_idx;
}

std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersection_id)
{
    return adjacent_street_segments[intersection_id];
}

IntersectionIdx findClosestIntersection(LatLon my_position)
{
    double minimum = findDistanceBetweenTwoPoints(getIntersectionPosition(0), my_position);
    IntersectionIdx intersectID = -1;
    for (IntersectionIdx i = 1; i < getNumIntersections(); i++)
    {
        LatLon intersectPos = getIntersectionPosition(i);
        double distance = findDistanceBetweenTwoPoints(intersectPos, my_position);
        if (distance < minimum)
        {
            minimum = distance;
            intersectID = i;
        }
    }
    return intersectID;
}

std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id)
{
    return intersection_street_segments[intersection_id];
}

// Returns all intersections along the given street.
// There should be no duplicate intersections in the returned vector.
// Speed Requirement --> high


std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id) {
    std::vector<IntersectionIdx> intersections;
    for (StreetSegmentIdx i = 0; i < streetSegmentVector[street_id].size(); i++) {
        intersections.push_back(getStreetSegmentInfo(streetSegmentVector[street_id][i]).from);
        intersections.push_back(getStreetSegmentInfo(streetSegmentVector[street_id][i]).to);
    }
    std::vector<int>::iterator ip;
    std::sort(intersections.begin(), intersections.end());
    ip = std::unique(intersections.begin(), intersections.begin() + intersections.size());
    intersections.resize(std::distance(intersections.begin(), ip));
    return intersections;
}

// Return all intersection ids at which the two given streets intersect.
// This function will typically return one intersection id for streets that
// intersect and a length 0 vector for streets that do not. For unusual curved
// streets it is possible to have more than one intersection at which two
// streets cross.
// There should be no duplicate intersections in the returned vector.
// Speed Requirement --> high


std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids) {
    
    //Find overlapping intersections between two streets
    std::vector<IntersectionIdx> twoStreetIntersections = {};
    auto begin = intersectionVector[street_ids.second].begin();
    auto end = intersectionVector[street_ids.second].end();
    for (int i = 0; i < intersectionVector[street_ids.first].size(); i++) {
        if (find(begin, end, intersectionVector[street_ids.first][i]) != end) {
            twoStreetIntersections.push_back(intersectionVector[street_ids.first][i]);
        }
    }
    
    //Remove duplicates from the overlapping intersection vector
    std::vector<int>::iterator ip;
    std::sort(twoStreetIntersections.begin(), twoStreetIntersections.end());
    ip = std::unique(twoStreetIntersections.begin(), twoStreetIntersections.begin() + twoStreetIntersections.size());
    twoStreetIntersections.resize(std::distance(twoStreetIntersections.begin(), ip));    
    return twoStreetIntersections;
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
std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix)
{
    std::vector<StreetIdx> streetsFound = {};
    int length = street_prefix.length();
    std::string prefixNoSpace = "";
    
    //Check if the prefix is empty
    if (street_prefix != ""){
        
        //Lowercase the prefix and remove white spaces in between
        for(int i = 0; i<length; i++){
            if(street_prefix[i] != ' ')
                prefixNoSpace += std::tolower(static_cast<unsigned char>(street_prefix[i]));               
        }

        //Search for street names that contain the prefix
        auto it = streetNameCollections.lower_bound(prefixNoSpace);
        while (it != streetNameCollections.end() && it->first.compare(0,prefixNoSpace.length(),prefixNoSpace) == 0){
            streetsFound.insert(streetsFound.end(), it->second.begin(), it->second.end());
            ++it;
        }
        std::vector<int>::iterator ip;
        std::sort(streetsFound.begin(), streetsFound.end());
        ip = std::unique(streetsFound.begin(), streetsFound.begin() + streetsFound.size());
        streetsFound.resize(std::distance(streetsFound.begin(), ip)); 
    }
    return streetsFound;
}


std::string getOSMNodeTagValue(OSMID osm_id, std::string key){
    if(OSMvec.find(osm_id) != OSMvec.end()){ //starting to find the  key value inside the OSMNode
        std::unordered_map<std::string, std::string>::iterator currTag = OSMvec.find(osm_id) -> second.find(key);  // the curret one tag  has the node tag information
        if(currTag != OSMvec.find(osm_id) -> second.end()){
            return  currTag -> second;
        }
    }
    return "";
}


void preprocessStreetSegments(){

    int numStreets = getNumStreets();
    streetSegmentVector.resize(numStreets);
    intersectionVector.resize(numStreets);
    int numSegments = getNumStreetSegments();
    segmentData.resize(numSegments);

    segmentLatLon.resize(numSegments);

    streetLat.resize(numStreets, {10000000, -10000000});
    streetLon.resize(numStreets, {10000000, -10000000});

    // Loop through all street segments and store coresponding info in the corresponding vectors
    for (int segmentId = 0; segmentId < getNumStreetSegments(); segmentId++){
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo(segmentId);
        streetSegmentVector[segmentInfo.streetID].push_back(segmentId);
        intersectionVector[segmentInfo.streetID].push_back(segmentInfo.from);
        intersectionVector[segmentInfo.streetID].push_back(segmentInfo.to);
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
        for (int i = 0; i < segmentInfo.numCurvePoints; i++)
        {
            LatLon curvePoint = getStreetSegmentCurvePoint(segmentId, i);

            // Update
            streetLat[streetId].first = std::min(streetLat[streetId].first, curvePoint.latitude());
            streetLat[streetId].second = std::max(streetLat[streetId].second, curvePoint.latitude());

            streetLon[streetId].first = std::min(streetLon[streetId].first, curvePoint.longitude());
            streetLon[streetId].second = std::max(streetLon[streetId].second, curvePoint.longitude());
        }
    }
}

void preprocessStreets() {
    int numStreet = getNumStreets();
    std::string nameNoSpace;
    for (StreetIdx i = 0; i < numStreet; i++) {
        nameNoSpace = "";
        for (int k = 0; k < getStreetName(i).length(); k++) {
            if (getStreetName(i)[k] != ' ')
                nameNoSpace += std::tolower(static_cast<unsigned char>(getStreetName(i)[k]));
        }
        streetNameCollections[nameNoSpace].push_back(i);
    }
}

int getNodeIndexFromOSMID(OSMID node_id) {
    auto it = osmidToNodeMap.find(node_id);
    if (it == osmidToNodeMap.end()) return -1;
    return it->second;
}

int getWayIndexFromOSMID(OSMID way_id) {
    auto it = osmidToWayMap.find(way_id);
    if (it == osmidToWayMap.end()) return -1;
    return it->second;
}

void preprocessMappings() {
    int numNodes = getNumberOfNodes();
    int numWays = getNumberOfWays();

    osmidToNodeMap.reserve(numNodes);
    osmidToWayMap.reserve(numWays);
    convertedWayIndex.resize(numWays);

    for (int i = 0; i < numNodes; i++) {
        OSMID nodeID = getNodeByIndex(i)->id();
        osmidToNodeMap[nodeID] = i;
    }

    // Store Way OSMID to Index AND Convert Way Nodes 
    for (int i = 0; i < numWays; i++) {
        OSMID wayID = getWayByIndex(i)->id();
        osmidToWayMap[wayID] = i;

        std::vector<int> nodeIndices;
        const std::vector<OSMID>& wayNodes = getWayMembers(getWayByIndex(i));
        nodeIndices.reserve(wayNodes.size());  

        for (int j = 0; j < wayNodes.size(); j++) {  
            auto it = osmidToNodeMap.find(wayNodes[j]);
            if (it != osmidToNodeMap.end()) {  
                nodeIndices.push_back(it->second);
            }
        }

        convertedWayIndex[i] = std::move(nodeIndices);  
    }
}
