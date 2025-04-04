#include "global.h"
#include "m3.h"
#include "m4.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

void precomputePath(const std::vector<DeliveryInf>& deliveriesVec, const std::vector<IntersectionIdx>& depotVec, float turnPenalty);
struct PathInfo{
    float travel_time;
    std::vector<StreetSegmentIdx> path;
};
std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;

// precomputation
void precomputePath(const std::vector<DeliveryInf>& deliveriesVec, const std::vector<IntersectionIdx>& depotVec, float turnPenalty){
    std::unordered_set<IntersectionIdx> targetCompute;
    // store all the pickup and dropoff locations
    for(int i = 0; i < deliveriesVec.size(); i++){
        targetCompute.insert(deliveriesVec[i].pickUp);
        targetCompute.insert(deliveriesVec[i].dropOff);
    }
    // store the depot location
    for(int i = 0; i < depotVec.size(); i++){
        targetCompute.insert(depotVec[i]);
    }
    //process the time and path found using m3 functions, then store it into our unordered map
    for(std::unordered_set<IntersectionIdx>::iterator i = targetCompute.begin(); i != targetCompute.end(); i++){
        for(std::unordered_set<IntersectionIdx>::iterator j = targetCompute.begin(); j != targetCompute.end(); j++){
            if(*i == *j){
                continue;
            }
            IntersectionIdx from = *i;
            IntersectionIdx to = *j;
            std::vector<StreetSegmentIdx> path = findPathBetweenIntersections(turnPenalty, std::make_pair(from, to));
            if(!path.empty()){
                float penalty_time = computePathTravelTime(turnPenalty, path);
                precompute[from][to] = PathInfo{penalty_time, path};
            }
        }
    }
}

std::vector<CourierSubPath> travelingCourier(const float turn_penalty, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots){
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);
    std::vector<CourierSubPath> finalRoute;
    std::unordered_set<int> pickUp;
    std::unordered_set<int> droppedOff;
    IntersectionIdx current = depots[0];
    float quickTime = 99999.0;
    // choose depot location
    for(int i = 0; i < depots.size(); i++){
        for(int j = 0; j < deliveries.size(); j++){
            IntersectionIdx from = depots[i];
            IntersectionIdx to = deliveries[j].pickUp;
            if(precompute[from].count(to)){
                float pathTime = precompute[from][to].travel_time;
                if(pathTime < quickTime){
                    quickTime = pathTime;
                    current = from;
                }
            }
        }
    }
    while(droppedOff.size() < deliveries.size()){
        int minind = -1;
        float min = 99999;
        IntersectionIdx next;
        //finding closest legal dropoff
        for(int i = 0; i < deliveries.size(); i++){
            if(pickUp.count(i) && !droppedOff.count(i)){
                IntersectionIdx drop = deliveries[i].dropOff;
                if(precompute[current].count(drop)){
                    float time = precompute[current][drop].travel_time;
                    if(time < min){
                        min = time;
                        minind = i;
                        next = drop;
                    }
                }
            }
        }
        //after looped through dropoff location, no dropoff then head to pickup node
        if(minind == -1){
            for(int i = 0; i < deliveries.size(); i++){
                if(!pickUp.count(i)){
                    IntersectionIdx pick = deliveries[i].pickUp;
                    if(precompute[current].count(pick)){
                        float time = precompute[current][pick].travel_time;
                        if(time < min){
                            min = time;
                            minind  = i;
                            next = pick;
                        }
                    }
                }
            }
        }
        // no valid route then return an empty vector
        if(minind == -1){
            return {};
        }
        finalRoute.push_back({{current, next}, precompute[current][next].path});
        if(pickUp.count(minind)){
            droppedOff.insert(minind);
        }
        else{
            pickUp.insert(minind);
        }
        current = next;
    }
    // return back to the closest depot near dropoff
    float backTime = 99999.0;
    IntersectionIdx closestEnd = depots[0];
    for(int i = 0; i < depots.size(); i++){
        IntersectionIdx depot = depots[i];
        if(precompute[current].count(depot)){
            float time = precompute[current][depot].travel_time;
            if(time < backTime){
                backTime = time;
                closestEnd = depot;
            }
        }
    }
    finalRoute.push_back({{current, closestEnd}, precompute[current][closestEnd].path});
    return finalRoute;
}