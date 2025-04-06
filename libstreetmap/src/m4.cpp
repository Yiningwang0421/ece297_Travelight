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
bool RandomGreedyOptimized(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, std::vector<int>& outOrder, IntersectionIdx& outStartDepot, float& outTravelTime);
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

bool RandomGreedyOptimized(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, std::vector<int>& outOrder, IntersectionIdx& outStartDepot, float& outTravelTime){
    const int k = 3; 
    std::unordered_set<int> pickedUp, droppedOff;
    std::vector<int> order;
    IntersectionIdx curr = depots[rand() % depots.size()];
    outStartDepot = curr;
    outOrder.clear();
    outTravelTime = 0.0;

    while (droppedOff.size() < deliveries.size()) {
        std::vector<std::pair<float, int>> candidates;
        for (int i = 0; i < deliveries.size(); ++i) {
            if (pickedUp.count(i) && !droppedOff.count(i)) {
                IntersectionIdx drop = deliveries[i].dropOff;
                if (precompute[curr].count(drop))
                    candidates.emplace_back(precompute[curr][drop].travel_time, i);
            }
        }
        for (int i = 0; i < deliveries.size(); ++i) {
            if (!pickedUp.count(i)) {
                IntersectionIdx pick = deliveries[i].pickUp;
                if (precompute[curr].count(pick))
                    candidates.emplace_back(precompute[curr][pick].travel_time, i);
            }
        }
        if (candidates.empty()) return false;
        std::sort(candidates.begin(), candidates.end());
        int chosenIdx = candidates[rand() % std::min(k, (int)candidates.size())].second;
        order.push_back(chosenIdx);
        if (pickedUp.count(chosenIdx)) droppedOff.insert(chosenIdx);
        else pickedUp.insert(chosenIdx);
        curr = pickedUp.count(chosenIdx) ? deliveries[chosenIdx].dropOff : deliveries[chosenIdx].pickUp;
    }
    pickedUp.clear();
    droppedOff.clear();
    curr = outStartDepot;
    outTravelTime = 0.0;

    for (int idx : order) {
        IntersectionIdx next = pickedUp.count(idx) ? deliveries[idx].dropOff : deliveries[idx].pickUp;
        if (!precompute[curr].count(next)) return false;

        outTravelTime += precompute[curr][next].travel_time;
        curr = next;
        if (pickedUp.count(idx)) droppedOff.insert(idx);
        else pickedUp.insert(idx);
    }
    float returnT = std::numeric_limits<float>::max();
    for (IntersectionIdx depot : depots) {
        if (precompute[curr].count(depot))
            returnT = std::min(returnT, precompute[curr][depot].travel_time);
    }

    if (returnT == std::numeric_limits<float>::max()) return false;

    outTravelTime += returnT;
    outOrder = order;
    return true;
}
std::vector<CourierSubPath> travelingCourier(const float turn_penalty, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);

    std::vector<int> bestOrder;
    float bestTime = std::numeric_limits<float>::max();
    IntersectionIdx bestDepot = depots[0];

    const int NUM_ATTEMPTS = 8;
    for (int attempt = 0; attempt < NUM_ATTEMPTS; ++attempt) {
        std::vector<int> order;
        float time = 0.0;
        IntersectionIdx depot;
        if (!RandomGreedyOptimized(deliveries, depots, order, depot, time))
            continue;

        for (int i = 0; i < 3; ++i) {
            float prev = time;
            swapOrder(order, time, depot, deliveries, depots);
            opt2Perturbation(order, time, depot, deliveries, depots);
            if (time >= prev - 0.1f) break;
        }
        if (time < bestTime) {
            bestTime = time;
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
