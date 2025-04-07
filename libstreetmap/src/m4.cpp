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
typedef std::pair<int, bool> DeliveryAction;
std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty);
void precomputePath(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty);
void swapOrder(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
void opt2Perturbation(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);

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
    int size = bestOrder.size();
    int maxIterations = 100;
    int maxNoImprovement = 15;
    int noImprovementCount = 0;

    for (int iter = 0; iter < maxIterations && noImprovementCount < maxNoImprovement; iter++) {
        int best_i = -1;
        int best_j = -1;
        float bestNewTime = bestTime;

        for (int i = 0; i < size - 1; i++) {
            for (int j = i + 1; j < size && j - i <= 15; j++) {
                std::vector<int> candidate = bestOrder;
                std::swap(candidate[i], candidate[j]);

                std::unordered_set<int> picked, dropped;
                IntersectionIdx curr = bestDepot;
                float travel = 0;
                bool legal = true;

                for (int idx : candidate) {
                    IntersectionIdx next;
                    if (picked.count(idx) && !dropped.count(idx)) {
                        next = deliveries[idx].dropOff;
                        dropped.insert(idx);
                    } else if (!picked.count(idx)) {
                        next = deliveries[idx].pickUp;
                        picked.insert(idx);
                    } else {
                        legal = false;
                        break;
                    }

                    if (!precompute[curr].count(next)) {
                        legal = false;
                        break;
                    }

                    travel += precompute[curr][next].travel_time;
                    if (travel > bestTime) {
                        legal = false;
                        break;
                    }

                    curr = next;
                }

                if (!legal) continue;

                float returnT = std::numeric_limits<float>::max();
                for (IntersectionIdx depot : depots) {
                    if (precompute[curr].count(depot)) {
                        float t = precompute[curr][depot].travel_time;
                        if (t < returnT) returnT = t;
                    }
                }

                travel += returnT;

                if (travel < bestNewTime) {
                    bestNewTime = travel;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        if (best_i != -1) {
            std::swap(bestOrder[best_i], bestOrder[best_j]);
            bestTime = bestNewTime;
            noImprovementCount = 0;
        } else {
            noImprovementCount++;
        }
    }
}

void opt2Perturbation(std::vector<int>& bestOrder, float& bestTime, IntersectionIdx bestDepot,
                      const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    int size = bestOrder.size();
    int maxWindow = 20;
    int maxIterations = 150;
    int maxNoImprovement = 10;
    int noImprovementCount = 0;

    for (int iter = 0; iter < maxIterations && noImprovementCount < maxNoImprovement; iter++) {
        int best_i = -1, best_j = -1;
        float bestNewTime = bestTime;

        for (int i = 0; i < size - 2; i++) {
            for (int j = i + 2; j < size && j - i <= maxWindow; j++) {
                std::vector<int> candidate = bestOrder;
                std::reverse(candidate.begin() + i, candidate.begin() + j + 1);

                std::unordered_set<int> picked, dropped;
                IntersectionIdx curr = bestDepot;
                float travel = 0;
                bool legal = true;

                for (int idx : candidate) {
                    IntersectionIdx next;
                    if (picked.count(idx) && !dropped.count(idx)) {
                        next = deliveries[idx].dropOff;
                        dropped.insert(idx);
                    } else if (!picked.count(idx)) {
                        next = deliveries[idx].pickUp;
                        picked.insert(idx);
                    } else {
                        legal = false;
                        break;
                    }

                    if (!precompute[curr].count(next)) {
                        legal = false;
                        break;
                    }

                    travel += precompute[curr][next].travel_time;
                    if (travel > bestTime) {
                        legal = false;
                        break;
                    }

                    curr = next;
                }

                if (!legal) continue;

                float returnT = std::numeric_limits<float>::max();
                for (IntersectionIdx depot : depots) {
                    if (precompute[curr].count(depot)) {
                        float t = precompute[curr][depot].travel_time;
                        if (t < returnT) returnT = t;
                    }
                }

                travel += returnT;

                if (travel < bestNewTime) {
                    bestNewTime = travel;
                    best_i = i;
                    best_j = j;
                }
            }
        }

        if (best_i != -1) {
            std::reverse(bestOrder.begin() + best_i, bestOrder.begin() + best_j + 1);
            bestTime = bestNewTime;
            noImprovementCount = 0;
        } else {
            noImprovementCount++;
        }
    }
}

std::vector<CourierSubPath> travelingCourier(const float turn_penalty, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);

    std::vector<DeliveryAction> bestOrder;
    float bestTime = std::numeric_limits<float>::max();
    IntersectionIdx bestDepot = depots[0];

    for (const IntersectionIdx& depot : depots) {
        std::unordered_set<int> pickedUp;
        std::unordered_set droppedOff;
        std::vector<DeliveryAction> order;
        IntersectionIdx curr = depot;
        bool valid = true;


        while (droppedOff.size() < deliveries.size()) {
            int bestIdx = -1;
            float minTime = std::numeric_limits<float>::max();
            IntersectionIdx next = -1;

            for (int i = 0; i < deliveries.size(); i++) { //If picked up but not dropped off yet
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

            //if (bestIdx == -1) {//Nowhere to drop off
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
            //}

            if (bestIdx == -1) {
                valid = false;
                break;
            }

            order.push_back(bestIdx);
            if (pickedUp.count(bestIdx)){
                droppedOff.insert(bestIdx);
            }
            else {
                pickedUp.insert(bestIdx);
            }
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
    bool improved = true;
    while(improved){
        float prevTime = bestTime;
        swapOrder(bestOrder, bestTime, bestDepot, deliveries, depots);
        opt2Perturbation(bestOrder, bestTime, bestDepot, deliveries, depots);
        improved = (bestTime < prevTime - 0.1);
    }

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
