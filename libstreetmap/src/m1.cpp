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

//function declared
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

//Returns the nearest intersection to the given position
//Speed Requirement --> none
IntersectionIdx findClosestIntersection(LatLon my_posotion){
    return IntersectionIdx(-1);
}
LatLonBounds findStreetBoundingBox (StreetIdx street_id){
    return LatLonBounds();
}

double findStreetSegmentTurnAngle(StreetSegmentIdx src_street_segment_id, StreetSegmentIdx dst_street_id){
    return 0.0;
}
// Returns the street segments for the given intersection
// Speed Requirement --> high
std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersetion_id){
    return std::vector<StreetSegmentIdx>();
}

//Returns tehe street names at the given intersetion (includes duplicate street names in returned vector)
//Speed Requiremnent --> high
std::vector<std::string> findStreetSegmentNamesOfIntersection(IntersectionIdx intersection_id){
    return std::vector<std::string>();
}

std::vector<IntersectionIdx> findAdjacentIntersections(IntersectionIdx intersetion_id){
    return std::vector<IntersectionIdx>();
}


bool loadMap(std::string map_streets_database_filename) {
    bool load_successful = false; //Indicates whether the map has loaded 
                                  //successfully

    std::cout << "loadMap: " << map_streets_database_filename << std::endl;

    //
    // Load your map related data structures here.
    //

    
    load_successful = true; //Make sure this is updated to reflect whether
                            //loading the map succeeded or failed

    return load_successful;
}

void closeMap() {
    //Clean-up your map related data structures here
    
}
