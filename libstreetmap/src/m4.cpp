#include "global.h"
#include "m1.h"
#include "m3.h"
#include "m4.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct PathInfo{
    float travel_time;
    std::vector<StreetSegmentIdx> path;
};
std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty);
void precomputePath(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty);

// precomputation
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty){
    std::unordered_map<IntersectionIdx, PathInfo> result;
    std::unordered_map<IntersectionIdx, float> bestTime;
    std::unordered_map<IntersectionIdx, StreetSegmentIdx> startPt;
    std::unordered_set<IntersectionIdx> visited;
    std::vector<IntersectionIdx> selectRoute;
    bestTime[start] = 0.0;
    selectRoute.push_back(start);
    while(!selectRoute.empty()){
        float minT = 99999.0;
        int bestInd = -1;
        for(int i = 0; i < selectRoute.size(); i++){
            if(bestTime[selectRoute[i]] < minT){
                minT = bestTime[selectRoute[i]];
                bestInd = i;
            }
        }
        //find nodes that takes shortest time
        IntersectionIdx selection = selectRoute[bestInd];
        selectRoute.erase(selectRoute.begin() + bestInd);
        if(visited.count(selection)){
            continue;
        }
        visited.insert(selection);
        const std::vector<StreetSegmentIdx>& streetseg = findStreetSegmentsOfIntersection(selection);
        for(StreetSegmentIdx streetId: streetseg){
            const StreetSegmentInfo &streetInfo = getStreetSegmentInfo(streetId);
            if(streetInfo.oneWay && streetInfo.from != selection){
                continue;
            }
            IntersectionIdx intersectType;
            if(streetInfo.from == selection){
                intersectType = streetInfo.to;
            }
            else{
                intersectType = streetInfo.from;
            }
            float penalty = 0.0f;
            if(startPt.count(selection)){
                StreetSegmentIdx prevSeg = startPt[selection];
                if(getStreetSegmentInfo(prevSeg).streetID != streetInfo.streetID){
                    penalty = turnPenalty;
                }
            }
            float time = bestTime[selection] + findStreetSegmentTravelTime(streetId) + penalty;
            if(!bestTime.count(intersectType) || time < bestTime[intersectType]){
                bestTime[intersectType] = time;
                startPt[intersectType] = streetId;
                selectRoute.push_back(intersectType);
            }
        }
    }
    // building the paths
    for(const IntersectionIdx& to: target){
        if(to == start || !startPt.count(to)){ continue;}
        std::vector<StreetSegmentIdx> path;
        IntersectionIdx curr = to;
        while(curr != start){
            StreetSegmentIdx seg = startPt[curr];
            path.push_back(seg);
            const StreetSegmentInfo& segInfo = getStreetSegmentInfo(seg);
            if(segInfo.to == curr){
                curr = segInfo.from;
            }
            else{
                curr = segInfo.to;
            }
        }
        std::reverse(path.begin(), path.end());
        result[to] = {bestTime[to], path};
    }
    return result;
}

void precomputePath(const std::vector<DeliveryInf> & deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty){
    std::unordered_set<IntersectionIdx> nodes;
    for(int i = 0; i < deliveries.size(); i++){
        nodes.insert(deliveries[i].dropOff);
        nodes.insert(deliveries[i].pickUp);
    }
    for(int i = 0; i < depots.size(); i++){
        nodes.insert(depots[i]);
    }
    for(std::unordered_set<IntersectionIdx>::iterator i = nodes.begin(); i != nodes.end(); i++){
        IntersectionIdx start = *i;
        precompute[start] = dijkstra(start, nodes, turnPenalty);
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