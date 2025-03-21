#include "global.h"
#include "m1.h"
#include "m3.h"
#define NO_EDGE -1

struct WaveElem {
    IntersectionIdx node_ID;
    StreetSegmentIdx edge_ID;
    double travel_time;
    double predicted_add_on;
    
    WaveElem(int n, int e, float time, float a){
        node_ID = n;
        edge_ID = e;
        travel_time = time;
        predicted_add_on = a;
    }
};

struct Node {
    StreetSegmentIdx reaching_edge;
    double best_time;
};

struct compareTime {
    bool operator()(const WaveElem& a, const WaveElem& b) const{
        return (a.travel_time+a.predicted_add_on) > (b.travel_time+b.predicted_add_on);
    }
};

//std::vector<Node> nodes;
//std::vector<std::vector<IntersectionIdx>> adjacent_street_segments;
std::vector<Node> nodes;

bool searchPath(const double turn_penalty, IntersectionIdx srcID, IntersectionIdx destID);
std::vector<StreetSegmentIdx> pathTraceBack(IntersectionIdx destID);
IntersectionIdx findOtherEnd(StreetSegmentIdx ss, IntersectionIdx one_end);

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

bool searchPath(const double turn_penalty, IntersectionIdx srcID, IntersectionIdx destID){ 
    bool pathFound = false;
    nodes.resize(getNumIntersections());
    for(int i=0; i<getNumIntersections();i++){
        nodes[i].reaching_edge = 0;
        nodes[i].best_time = std::numeric_limits<double>::max();
    }
    
    LatLon dest_pos = intersection_positions[destID];
    std::priority_queue<WaveElem, std::vector<WaveElem>, compareTime> wavefront;
    wavefront.push(WaveElem(srcID,NO_EDGE,0,findDistanceBetweenTwoPoints(intersection_positions[srcID],dest_pos)/max_speed)); 
    while(wavefront.size() > 0){
        WaveElem wave = wavefront.top();
        wavefront.pop();
        int curr_ID = wave.node_ID;
        
        if(wave.travel_time < nodes[curr_ID].best_time){
            nodes[curr_ID].reaching_edge = wave.edge_ID;
            nodes[curr_ID].best_time = wave.travel_time;
            
            if (curr_ID == destID){
                pathFound = true;
                break;
            }
            StreetIdx rea_edge_street = 0;
            if (nodes[curr_ID].reaching_edge != NO_EDGE){
                rea_edge_street = getStreetSegmentInfo(nodes[curr_ID].reaching_edge).streetID;
            }
            for(int i=0; i<outgoingInfo[curr_ID].size(); i++){
                //std::cout<<adjacent_street_segments[curr_ID].size()<<std::endl;
               
                    StreetSegmentIdx out_edge = outgoingInfo[curr_ID][i].second;
                    StreetSegmentIdx rea_edge = nodes[curr_ID].reaching_edge;
                    if(out_edge == rea_edge){
                        continue;
                    }
                    else{
                        IntersectionIdx to_node_id = outgoingInfo[curr_ID][i].first;
                        double penalty = 0;
                        if (rea_edge != NO_EDGE){
                            if (getStreetSegmentInfo(out_edge).streetID != rea_edge_street){
                                penalty = turn_penalty;
                            }
                        }
                        wavefront.emplace(WaveElem(to_node_id,out_edge,nodes[curr_ID].best_time+segment_travel_time[out_edge]+penalty,
                            findDistanceBetweenTwoPoints(intersection_positions[to_node_id],dest_pos)/max_speed)); 
                    }  
            }
        }
    }
    return pathFound;
}

std::vector<StreetSegmentIdx> pathTraceBack(IntersectionIdx destID){
    std::list<StreetSegmentIdx> path;
    IntersectionIdx current_node_id = destID;
    StreetSegmentIdx prev_edge = nodes[destID].reaching_edge;
    
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
    nodes.clear();
    return std::vector<StreetSegmentIdx>(path.begin(),path.end());
}

