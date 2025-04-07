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
#include <queue>

struct PathInfo{
    float travel_time;
    std::vector<StreetSegmentIdx> path;
};

std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty);
void precomputePath(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty);
void swapOrder(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
void opt2Perturbation(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
float evaluatePath(const std::vector<int>& order,
                   IntersectionIdx depot,
                   const std::vector<DeliveryInf>& deliveries,
                   const std::vector<IntersectionIdx>& depots);
void buildGreedyOrder(std::vector<int>& bestOrder,
                      float& bestTime,
                      IntersectionIdx& bestDepot,
                      const std::vector<DeliveryInf>& deliveries,
                      const std::vector<IntersectionIdx>& depots);
std::vector<CourierSubPath> buildFinalRoute(const std::vector<int>& order,
                                            IntersectionIdx startDepot,
                                            const std::vector<DeliveryInf>& deliveries,
                                            const std::vector<IntersectionIdx>& depots);

std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start,const std::unordered_set<IntersectionIdx>& target,float turnPenalty){
    std::unordered_map<IntersectionIdx, PathInfo> result;
    std::vector<float> bestTime(getNumIntersections(), 999999);
    std::vector<StreetSegmentIdx> parent(getNumIntersections(), -1);
    std::priority_queue<std::pair<float, IntersectionIdx>, std::vector<std::pair<float, IntersectionIdx>>, std::greater<>> computeQ;
    bestTime[start] = 0.0;
    computeQ.emplace(0.0, start);
    while (!computeQ.empty()) {
        auto [currTime, from] = computeQ.top();
        computeQ.pop();
        if(currTime > bestTime[from]) {continue;}
        const auto& segments = findStreetSegmentsOfIntersection(from);
        for(const StreetSegmentIdx seg:segments){
            const auto& info = getStreetSegmentInfo(seg);
            if(info.oneWay && info.from != from){continue;}
            IntersectionIdx to = -1;
            if(info.from == from){
                to = info.to;
            }
            else{
                to = info.from;
            }
            float penalty = 0.0;
            if(parent[from] != -1){
                const auto& prevInfo =  getStreetSegmentInfo(parent[from]);
                if(prevInfo.streetID != info.streetID){
                    penalty = turnPenalty;
                }
            }
            float newT = bestTime[from] + findStreetSegmentTravelTime(seg) + penalty;
            if(newT < bestTime[to]){
                bestTime[to] = newT;
                parent[to] = seg;
                computeQ.emplace(newT, to);
            }
        }
    }
    for(IntersectionIdx to: target){
        if(to == start || parent[to] == -1){
            continue;
        }
        std::vector<StreetSegmentIdx> path;
        for(IntersectionIdx curr = to; curr != start;){
            StreetSegmentIdx seg = parent[curr];
            path.push_back(seg);
            const auto& info = getStreetSegmentInfo(seg);
            if(info.to == curr){
                curr = info.from;
            }
            else{
                curr = info.to;
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
    std::vector<IntersectionIdx> nodeVector(nodes.begin(), nodes.end());
    std::vector<std::unordered_map<IntersectionIdx, PathInfo>> local_maps(nodeVector.size());
    #pragma omp parallel for
    for (int i = 0; i < nodeVector.size(); i++) {
        IntersectionIdx from = nodeVector[i];
        local_maps[i] = dijkstra(from, nodes, turnPenalty);
    }
    for (int i = 0; i < nodeVector.size(); i++) {
        precompute[nodeVector[i]] = std::move(local_maps[i]);
    }
}

void swapOrder(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot,
               const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    const int maxTrials = 40000;
    int trials = 0;
    int size = bestOrder.size();
    while (trials < maxTrials) {
        std::vector<int> tempOrder = bestOrder;
        int i = rand() % size;
        int j = rand() % size;
        while (j == i) j = rand() % size;
        std::swap(tempOrder[i], tempOrder[j]);
        std::unordered_set<int> pickedUp, droppedOff;
        IntersectionIdx curr = bestDepot;
        float newTime = 0;
        bool legal = true;
        for (int idx : tempOrder) {
            IntersectionIdx next;
            if (pickedUp.count(idx)) {
                next = deliveries[idx].dropOff;
                droppedOff.insert(idx);
            } else {
                next = deliveries[idx].pickUp;
                pickedUp.insert(idx);
            }
            if (!precompute[curr].count(next)) {
                legal = false;
                break;
            }
            newTime += precompute[curr][next].travel_time;
            if (newTime > bestTime) {
                legal = false;
                break;
            }
            curr = next;
        }
        if (!legal) {
            trials++;
            continue;
        }
        float returnTime = std::numeric_limits<float>::max();
        for (const IntersectionIdx& depot : depots) {
            if (precompute[curr].count(depot)) {
                float t = precompute[curr][depot].travel_time;
                if (t < returnTime) returnTime = t;
            }
        }
        newTime += returnTime;
        if (newTime < bestTime) {
            bestOrder = tempOrder;
            bestTime = newTime;
            trials = 0;
        } else {
            trials++;
        }
    }
}

void opt2Perturbation(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot,
                      const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    const int maxRounds = 30;
    for (int round = 0; round < maxRounds; ++round) {
        bool improved = false;
        for (int i = 0; i + 2 < bestOrder.size(); i++) {
            for (int j = i + 2; j < bestOrder.size() && j - i <= 20; j++) {
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
                    if (newTime > bestTime) {
                        legal = false;
                        break;
                    }
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
                    bestOrder = newOrder;
                    bestTime = newTime;
                    improved = true;
                    break;
                }
            }
            if (improved) break;
        }
        if (!improved) break;
    }
}



float evaluatePath(const std::vector<int>& order,
                   IntersectionIdx depot,
                   const std::vector<DeliveryInf>& deliveries,
                   const std::vector<IntersectionIdx>& depots) {
    std::unordered_set<int> pickedUp;
    IntersectionIdx curr = depot;
    float totalTime = 0.0f;

    for (int idx : order) {
        IntersectionIdx next = pickedUp.count(idx) ? deliveries[idx].dropOff : deliveries[idx].pickUp;

        if (!precompute[curr].count(next)) return std::numeric_limits<float>::max();

        totalTime += precompute[curr][next].travel_time;
        curr = next;
        pickedUp.insert(idx);
    }

    float returnTime = std::numeric_limits<float>::max();
    for (const IntersectionIdx& d : depots) {
        if (precompute[curr].count(d)) {
            float t = precompute[curr][d].travel_time;
            if (t < returnTime) returnTime = t;
        }
    }

    totalTime += returnTime;
    return totalTime;
}

void buildGreedyOrder(std::vector<int>& bestOrder,
                      float& bestTime,
                      IntersectionIdx& bestDepot,
                      const std::vector<DeliveryInf>& deliveries,
                      const std::vector<IntersectionIdx>& depots) {
    bestTime = std::numeric_limits<float>::max();
    bestOrder.clear();
    bestDepot = depots[0];

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

        float totalTime = evaluatePath(order, depot, deliveries, depots);
        if (totalTime < bestTime) {
            bestOrder = order;
            bestTime = totalTime;
            bestDepot = depot;
        }
    }
}

std::vector<CourierSubPath> travelingCourier(
    const float turn_penalty,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots) {

    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);

    std::vector<int> bestOrder;
    float bestTime;
    IntersectionIdx bestDepot;

    buildGreedyOrder(bestOrder, bestTime, bestDepot, deliveries, depots);

    bool improved = true;
    while (improved) {
        float prev = bestTime;
        swapOrder(bestOrder, bestTime, bestDepot, deliveries, depots);
        opt2Perturbation(bestOrder, bestTime, bestDepot, deliveries, depots);
        improved = (bestTime < prev - 0.1);
    }

    return buildFinalRoute(bestOrder, bestDepot, deliveries, depots);
}

std::vector<CourierSubPath> buildFinalRoute(const std::vector<int>& order,
                                            IntersectionIdx startDepot,
                                            const std::vector<DeliveryInf>& deliveries,
                                            const std::vector<IntersectionIdx>& depots) {
    std::vector<CourierSubPath> route;
    std::unordered_set<int> pickedUp;
    IntersectionIdx curr = startDepot;

    for (int idx : order) {
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