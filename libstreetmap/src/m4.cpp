#include "global.h"
#include "m3.h"
#include "m4.h"

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