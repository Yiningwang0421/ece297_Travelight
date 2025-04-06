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

std::unordered_map<IntersectionIdx, PathInfo> aStar(
    IntersectionIdx start,
    const std::unordered_set<IntersectionIdx>& target,
    float turnPenalty) {

    std::unordered_map<IntersectionIdx, PathInfo> result;
    std::vector<float> bestTime(getNumIntersections(), std::numeric_limits<float>::max());
    std::vector<StreetSegmentIdx> parent(getNumIntersections(), -1);
    std::set<std::pair<float, IntersectionIdx>> open; // (f = g + h, node)

    bestTime[start] = 0.0;

    // Choose a single goal for heuristic, if applicable
    IntersectionIdx goal = -1;
    bool useHeuristic = (target.size() == 1);
    if (useHeuristic) goal = *target.begin();

    auto heuristic = [&](IntersectionIdx from) -> float {
        if (!useHeuristic) return 0.0f;
        LatLon p1 = getIntersectionPosition(from);
        LatLon p2 = getIntersectionPosition(goal);
        return findDistanceBetweenTwoPoints(p1, p2) / 25.0f; // Rough average city speed
    };

    open.insert({heuristic(start), start});

    while (!open.empty()) {
        auto [currF, from] = *open.begin();
        open.erase(open.begin());

        // Optional early exit if we reach one of the targets
        if (target.count(from)) {
            if (useHeuristic) break; // if only one target, exit early
        }

        const auto& segs = findStreetSegmentsOfIntersection(from);
        for (StreetSegmentIdx s : segs) {
            const StreetSegmentInfo& info = getStreetSegmentInfo(s);

            if (info.oneWay && info.from != from) continue;

            IntersectionIdx to = (info.from == from) ? info.to : info.from;

            float penalty = 0.0f;
            if (parent[from] != -1) {
                const auto& prev = getStreetSegmentInfo(parent[from]);
                if (prev.streetID != info.streetID) penalty = turnPenalty;
            }

            float travelTime = findStreetSegmentTravelTime(s);
            float newG = bestTime[from] + travelTime + penalty;

            if (newG < bestTime[to]) {
                open.erase({bestTime[to] + heuristic(to), to});
                bestTime[to] = newG;
                parent[to] = s;
                open.insert({newG + heuristic(to), to});
            }
        }
    }

    for (IntersectionIdx to : target) {
        if (to == start || parent[to] == -1) continue;

        std::vector<StreetSegmentIdx> path;
        IntersectionIdx curr = to;
        while (curr != start) {
            StreetSegmentIdx seg = parent[curr];
            path.push_back(seg);
            const auto& info = getStreetSegmentInfo(seg);
            curr = (info.to == curr) ? info.from : info.to;
        }
        std::reverse(path.begin(), path.end());
        result[to] = {bestTime[to], path};
    }

    return result;
}

void precomputePath(const std::vector<DeliveryInf> & deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty){
    std::unordered_set<IntersectionIdx> nodes;

    // Collect all relevant nodes: pickups, dropoffs, and depots
    for (const auto& delivery : deliveries) {
        nodes.insert(delivery.pickUp);
        nodes.insert(delivery.dropOff);
    }
    for (const auto& depot : depots) {
        nodes.insert(depot);
    }

    // Precompute paths from every node to all other nodes
    for (IntersectionIdx start : nodes) {
        std::unordered_set<IntersectionIdx> targets = nodes;
        targets.erase(start);  // No need to compute path to self

        auto paths = aStar(start, targets, turnPenalty);

        // Store results in the global precompute map
        precompute[start] = paths;
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
