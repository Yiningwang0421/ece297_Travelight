#include "global.h"
#include "m1.h"
#include "m3.h"
#include "m4.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <limits>
#include <set>

struct PathInfo{
    float travel_time;
    std::vector<StreetSegmentIdx> path;
};

std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty);
void precomputePath(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty);

std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start,const std::unordered_set<IntersectionIdx>& target,float turnPenalty){
    std::unordered_map<IntersectionIdx, PathInfo> result;
    std::vector<float> bestTime(getNumIntersections(), 999999);
    std::vector<StreetSegmentIdx> parent(getNumIntersections(), -1);
    std::set<std::pair<float, IntersectionIdx>> open;
    bestTime[start] = 0.0;
    open.insert(std::make_pair(0.0, start));
    while (!open.empty()) {
        std::pair<float, IntersectionIdx> current = *open.begin();
        open.erase(open.begin());
        IntersectionIdx from = current.second; //retrieve the value in pair and compute the possible paths
        const std::vector<StreetSegmentIdx>& segs = findStreetSegmentsOfIntersection(from);
        for (int i = 0; i < segs.size(); i++) {
            StreetSegmentIdx s = segs[i];
            const StreetSegmentInfo& info = getStreetSegmentInfo(s);
            if (info.oneWay && info.from != from){
                continue;
            }
            // returning the head and tail of an segment for finding further delivery nodes
            IntersectionIdx to;
            if (info.from == from){
                to = info.to;
            } 
            else {
                to = info.from;
            }
            float penalty = 0.0;
            if (parent[from] != -1) {
                const StreetSegmentInfo& prev = getStreetSegmentInfo(parent[from]);
                if (prev.streetID != info.streetID) {
                    penalty = turnPenalty;
                }
            }
            float newTime = bestTime[from] + findStreetSegmentTravelTime(s) + penalty;
            if (newTime < bestTime[to]) {
                if (bestTime[to] != std::numeric_limits<float>::max()) {
                    open.erase(std::make_pair(bestTime[to], to));
                }
                bestTime[to] = newTime;
                parent[to] = s;
                open.insert(std::make_pair(newTime, to));
            }
        }
    }

    for (std::unordered_set<IntersectionIdx>::const_iterator it = target.begin(); it != target.end(); ++it) {
        IntersectionIdx to = *it;
        if (to == start || parent[to] == -1){
            continue;
        }
        std::vector<StreetSegmentIdx> path;
        IntersectionIdx curr = to;
        while (curr != start) {
            StreetSegmentIdx seg = parent[curr];
            path.push_back(seg);
            const StreetSegmentInfo& info = getStreetSegmentInfo(seg);
            if (info.to == curr) curr = info.from;
            else curr = info.to;
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
        precompute[*i] = dijkstra(*i, nodes, turnPenalty);
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