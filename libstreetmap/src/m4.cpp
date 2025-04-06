#include "global.h"
#include "m1.h"
#include "m3.h"
#include "m4.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <limits>
#include <set>
#include <omp.h>

struct PathInfo{
    float travel_time;
    std::vector<StreetSegmentIdx> path;
};

std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty);
void precomputePath(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty);
void swapOrder(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
void opt2Perturbation(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);

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
    std::vector<IntersectionIdx> nodeVector(nodes.begin(), nodes.end());
    #pragma omp parallel for
    for(int i = 0; i < nodeVector.size(); i++){
        IntersectionIdx from = nodeVector[i];
        std::unordered_map<IntersectionIdx, PathInfo> result = dijkstra(from, nodes,turnPenalty);
        #pragma omp critical
        precompute[from] = std::move(result);
    }
}

void swapOrder(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots){
    int maxSwap = 10000;
    int count = 0;
    for (int i = 0; i + 1 < bestOrder.size(); i++) {
        for (int j = i + 1; j < bestOrder.size(); j++) {
            if(++count > maxSwap){
                return;
            }
            std::vector<int> temp = bestOrder;
            std::swap(temp[i], temp[j]);
            std::unordered_set<int> pickedUp, droppedOff;
            IntersectionIdx curr = bestDepot;
            float newTime = 0;
            bool valid = true;

            for (int idx : temp) {
                IntersectionIdx next = pickedUp.count(idx) ? deliveries[idx].dropOff : deliveries[idx].pickUp;
                if (!precompute[curr].count(next)) {
                    valid = false;
                    break;
                }
                newTime += precompute[curr][next].travel_time;
                if(newTime > bestTime){
                    valid = false;
                    break;
                }
                curr = next;
                if (pickedUp.count(idx)){
                    droppedOff.insert(idx);
                }
                else {
                    pickedUp.insert(idx);
                }
            }
            if(!valid){continue;}

            float returnTime = std::numeric_limits<float>::max();
            for (const IntersectionIdx& depot : depots) {
                if (precompute[curr].count(depot)) {
                    float t = precompute[curr][depot].travel_time;
                    if (t < returnTime) returnTime = t;
                }
            }

            newTime += returnTime;
            if (valid && newTime < bestTime) {
                bestTime = newTime;
                bestOrder = temp;
            }
        }
    }
}

void opt2Perturbation(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots){
    bool improve = true;
    int iteration = 0;
    int maxIterations = 300;
    while (improve && iteration++ < maxIterations) {
        improve = false;
        for (int i = 0; i + 2 < bestOrder.size(); i++) {
            for (int j = i + 2; j < bestOrder.size()  && j - i <= 10; j++) {
                std::vector<int> newOrder = bestOrder;
                std::reverse(newOrder.begin() + i, newOrder.begin() + j + 1);
                std::unordered_set<int> pickedUp, droppedOff;
                IntersectionIdx curr = bestDepot;
                float newTime = 0;
                bool legal = true;

                for (int idx : newOrder) {
                    IntersectionIdx next = pickedUp.count(idx) ? deliveries[idx].dropOff : deliveries[idx].pickUp;
                    if (!precompute[curr].count(next)) {
                        legal = false;
                        break;
                    }
                    newTime += precompute[curr][next].travel_time;
                    curr = next;
                    if (pickedUp.count(idx)) droppedOff.insert(idx);
                    else pickedUp.insert(idx);
                }

                if (!legal) continue;

                float returnTime = std::numeric_limits<float>::max();
                for (const IntersectionIdx& depot : depots) {
                    if (precompute[curr].count(depot)) {
                        float t = precompute[curr][depot].travel_time;
                        if (t < returnTime) returnTime = t;
                    }
                }

                newTime += returnTime;
                if (newTime < bestTime) {
                    bestTime = newTime;
                    bestOrder = newOrder;
                    improve = true;
                    break;
                }
            }
            if (improve) break;
        }
    }
}

std::vector<CourierSubPath> travelingCourier(const float turn_penalty,
                                             const std::vector<DeliveryInf>& deliveries,
                                             const std::vector<IntersectionIdx>& depots) {
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);

    std::vector<int> bestOrder;
    float bestTime = std::numeric_limits<float>::max();
    IntersectionIdx bestDepot = depots[0];

    for (const IntersectionIdx& depot : depots) {
        std::unordered_set<int> pickedUp, droppedOff;
        std::vector<int> order;
        IntersectionIdx curr = depot;
        bool valid = true;

        while (droppedOff.size() < deliveries.size()) {
            int bestIdx = -1;
            float minTime = std::numeric_limits<float>::max();
            IntersectionIdx next = -1;

            for (int i = 0; i < deliveries.size(); i++) {
                if (pickedUp.count(i) && !droppedOff.count(i)) {
                    IntersectionIdx drop = deliveries[i].dropOff;
                    if (precompute[curr].count(drop)) {
                        float t = precompute[curr][drop].travel_time;
                        if (t < minTime) {
                            minTime = t;
                            bestIdx = i;
                            next = drop;
                        }
                    }
                }
            }

            if (bestIdx == -1) {
                for (int i = 0; i < deliveries.size(); i++) {
                    if (!pickedUp.count(i)) {
                        IntersectionIdx pick = deliveries[i].pickUp;
                        if (precompute[curr].count(pick)) {
                            float t = precompute[curr][pick].travel_time;
                            if (t < minTime) {
                                minTime = t;
                                bestIdx = i;
                                next = pick;
                            }
                        }
                    }
                }
            }

            if (bestIdx == -1) {
                valid = false;
                break;
            }

            order.push_back(bestIdx);
            if (pickedUp.count(bestIdx)) droppedOff.insert(bestIdx);
            else pickedUp.insert(bestIdx);
            curr = next;
        }

        if (!valid) continue;

        float totalTime = 0;
        pickedUp.clear();
        droppedOff.clear();
        curr = depot;

        for (int idx : order) {
            IntersectionIdx next = pickedUp.count(idx) ? deliveries[idx].dropOff : deliveries[idx].pickUp;
            totalTime += precompute[curr][next].travel_time;
            curr = next;
            if (pickedUp.count(idx)) droppedOff.insert(idx);
            else pickedUp.insert(idx);
        }

        float returnT = std::numeric_limits<float>::max();
        for (const IntersectionIdx& endDepot : depots) {
            if (precompute[curr].count(endDepot)) {
                float t = precompute[curr][endDepot].travel_time;
                if (t < returnT) returnT = t;
            }
        }

        totalTime += returnT;
        if (totalTime < bestTime) {
            bestTime = totalTime;
            bestOrder = order;
            bestDepot = depot;
        }
    }

    // Optimize with local search
    swapOrder(bestOrder, bestTime, bestDepot, deliveries, depots);
    opt2Perturbation(bestOrder, bestTime, bestDepot, deliveries, depots);

    // Build final route
    std::vector<CourierSubPath> route;
    std::unordered_set<int> pickedUp;
    IntersectionIdx curr = bestDepot;

    for (int idx : bestOrder) {
        IntersectionIdx next = pickedUp.count(idx) ? deliveries[idx].dropOff : deliveries[idx].pickUp;
        pickedUp.insert(idx);
        route.push_back({{curr, next}, precompute[curr][next].path});
        curr = next;
    }

    float returnT = std::numeric_limits<float>::max();
    IntersectionIdx bestEndDepot = depots[0];
    for (const IntersectionIdx& depot : depots) {
        if (precompute[curr].count(depot)) {
            float t = precompute[curr][depot].travel_time;
            if (t < returnT) {
                returnT = t;
                bestEndDepot = depot;
            }
        }
    }
    route.push_back({{curr, bestEndDepot}, precompute[curr][bestEndDepot].path});
    return route;
}
