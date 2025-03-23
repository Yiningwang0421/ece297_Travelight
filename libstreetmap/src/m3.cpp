#include "global.h"
#include "m1.h"
#include "m3.h"
#define NO_EDGE -1

//Stores the next node to be explored
struct WaveElem {
    IntersectionIdx node_ID; //The node associated with the wave element
    StreetSegmentIdx edge_ID; //The reaching edge
    double travel_time; //The time it takes to reach this element
    double predicted_add_on; //Heuristic value
    
    //Constructor
    WaveElem(int n, int e, float time, float p){
        node_ID = n;
        edge_ID = e;
        travel_time = time;
        predicted_add_on = p;
    }
};

//Store information of a particular node
struct Node {
    StreetSegmentIdx reaching_edge; //The reaching edge of the node
    double best_time; //The shortest time it takes to reach this node
};

//Compare between wave elements by their travel time plus heuristic value
struct compareTime {
    bool operator()(const WaveElem& a, const WaveElem& b) const{
        return (a.travel_time+a.predicted_add_on) > (b.travel_time+b.predicted_add_on);
    }
};

std::vector<Node> nodes; //Store all possible nodes in the map

//Function declaration
bool searchPath(const double turn_penalty, IntersectionIdx srcID, IntersectionIdx destID);
std::vector<StreetSegmentIdx> pathTraceBack(IntersectionIdx destID);
IntersectionIdx findOtherEnd(StreetSegmentIdx ss, IntersectionIdx one_end);
float calculateHeuristics(IntersectionIdx src, IntersectionIdx dest);

// Returns the time required to travel along the path specified, in seconds.
// The path is given as a vector of street segment ids, and this function can
// assume the vector either forms a legal path or has size == 0. The travel
// time is the sum of the length/speed-limit of each street segment, plus the
// given turn_penalty (in seconds) per turn implied by the path. If there is
// no turn, then there is no penalty. Note that whenever the street id changes
// (e.g. going from Bloor Street West to Bloor Street East) we have a turn.
double computePathTravelTime(const double turn_penalty, const std::vector<StreetSegmentIdx>& path){
    double travel_time = 0;
    
    //Add the travel time for each street segment
    for(int i=0; i<path.size(); i++){
        travel_time = travel_time + findStreetSegmentTravelTime(path[i]);
        
        //Detect turns and apply turn penalties
        if (i > 0 && getStreetSegmentInfo(path[i]).streetID != getStreetSegmentInfo(path[i-1]).streetID){
            travel_time = travel_time + turn_penalty;
        }
    }
    return travel_time;
}


// Returns a path (route) between the start intersection (intersect_id.first)
// and the destination intersection (intersect_id.second), if one exists.
// This routine should return the shortest path
// between the given intersections, where the time penalty to turn right or
// left is given by turn_penalty (in seconds). If no path exists, this routine
// returns an empty (size == 0) vector. If more than one path exists, the path
// with the shortest travel time is returned. The path is returned as a vector
// of street segment ids; traversing these street segments, in the returned
// order, would take one from the start to the destination intersection.
std::vector<StreetSegmentIdx> findPathBetweenIntersections(const double turn_penalty, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids){
    std::vector<StreetSegmentIdx> path;
    if (searchPath(turn_penalty, intersect_ids.first, intersect_ids.second)){
        path = pathTraceBack(intersect_ids.second);
    }
    return path;
}

//Search if a path exists and update the node values accordingly
bool searchPath(const double turn_penalty, IntersectionIdx srcID, IntersectionIdx destID){ 
    bool pathFound = false;
    
    //Initialize the node array
    nodes.resize(getNumIntersections());
    for(int i=0; i<getNumIntersections();i++){
        nodes[i].reaching_edge = 0;
        nodes[i].best_time = std::numeric_limits<double>::max();
    }
    
    //Initialize the search queue
    std::priority_queue<WaveElem, std::vector<WaveElem>, compareTime> wavefront;
    wavefront.push(WaveElem(srcID,NO_EDGE,0,calculateHeuristics(srcID, destID))); 
    
    //Search for possible path
    while(wavefront.size() > 0){
        WaveElem wave = wavefront.top();
        wavefront.pop();
        int curr_ID = wave.node_ID;
        
        //Update node values when a shorter path is detected
        if(wave.travel_time < nodes[curr_ID].best_time){
            nodes[curr_ID].reaching_edge = wave.edge_ID;
            nodes[curr_ID].best_time = wave.travel_time;
            
            //Exit the search loop if path found
            if (curr_ID == destID){
                pathFound = true;
                break;
            }
            
            //Explore next layer of nodes
            for(int i=0; i<outgoingInfo[curr_ID].size(); i++){               
                    StreetSegmentIdx out_edge = outgoingInfo[curr_ID][i].second;
                    StreetSegmentIdx rea_edge = nodes[curr_ID].reaching_edge;
                    if(out_edge == rea_edge){
                        continue;
                    }
                    else{
                        IntersectionIdx to_node_id = outgoingInfo[curr_ID][i].first;
                        
                        //Check if a turn exists and apply penalty accordingly
                        double penalty = 0;
                        if (rea_edge != NO_EDGE){
                            if (getStreetSegmentInfo(out_edge).streetID != getStreetSegmentInfo(nodes[curr_ID].reaching_edge).streetID){
                                penalty = turn_penalty;
                            }
                        }
                        
                        //Push another valid node to be explored into the search queue
                        wavefront.push(WaveElem(to_node_id,out_edge,nodes[curr_ID].best_time+segment_travel_time[out_edge]+penalty,calculateHeuristics(srcID, destID))); 
                    }  
            }
        }
    }
    return pathFound;
}

//Retrieve the street segments of a path based on nodes' information
std::vector<StreetSegmentIdx> pathTraceBack(IntersectionIdx destID){
    std::list<StreetSegmentIdx> path;
    IntersectionIdx current_node_id = destID;
    StreetSegmentIdx prev_edge = nodes[destID].reaching_edge;
    
    //Backtrack the nodes from the destination through the reaching edge and store them
    while(prev_edge != NO_EDGE){
        path.push_front(prev_edge);
        if(getStreetSegmentInfo(prev_edge).to == current_node_id){
            current_node_id = getStreetSegmentInfo(prev_edge).from;
        }
        else{
            current_node_id = getStreetSegmentInfo(prev_edge).to;
        }
        prev_edge = nodes[current_node_id].reaching_edge;
    }
    
    //Clear the nodes and return the street segments as a vector
    nodes.clear();
    return std::vector<StreetSegmentIdx>(path.begin(),path.end());
}

//Calculate the heuristic value of a wave element for optimization
float calculateHeuristics(IntersectionIdx src, IntersectionIdx dest){
    LatLon p1 = getIntersectionPosition(src);
    LatLon p2 = getIntersectionPosition(dest);
    
    // Convert latitude and longitude from degrees to radians
    double lat1 = p1.latitude() * kDegreeToRadian;
    double lon1 = p1.longitude() * kDegreeToRadian;
    double lat2 = p2.latitude() * kDegreeToRadian;
    double lon2 = p2.longitude() * kDegreeToRadian;

    // Compute the average latitude
    double lat_avg = (lat1 + lat2) / 2.0;

    // Compute x and y distances
    double x = kEarthRadiusInMeters * (lon2 - lon1) * std::cos(lat_avg);
    double y = kEarthRadiusInMeters * (lat2 - lat1);
    return (x*x+y*y)/max_speed/max_speed;
}

