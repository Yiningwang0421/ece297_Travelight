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
#include <random>

struct PathInfo{
    float travel_time;
    std::vector<StreetSegmentIdx> path;
};

struct VisitNode {
    IntersectionIdx id;
    bool isPickup;
    int deliveryIdx;
};

std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty);
void precomputePath(const std::vector<DeliveryInf> & deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty);
float evaluatePath(const std::vector<int>& order, IntersectionIdx depot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
std::vector<VisitNode> generateLegalGreedyRoute(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, IntersectionIdx& bestDepotOut);

std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start,const std::unordered_set<IntersectionIdx>& targets,float turnPenalty) {
    std::unordered_map<IntersectionIdx, PathInfo> result;
    std::vector<float> bestTime(getNumIntersections(), std::numeric_limits<float>::max());
    std::vector<StreetSegmentIdx> parent(getNumIntersections(), -1);
    std::priority_queue<std::pair<float, IntersectionIdx>, std::vector<std::pair<float, IntersectionIdx>>, std::greater<std::pair<float, IntersectionIdx>>> queue;
    bestTime[start] = 0.0;
    queue.push(std::make_pair(0.0, start));

    while (!queue.empty()) {
        float currTime = queue.top().first;
        IntersectionIdx from = queue.top().second;
        queue.pop();
        if (currTime > bestTime[from]){
            continue;
        }
        const std::vector<StreetSegmentIdx>& segments = findStreetSegmentsOfIntersection(from);
        for (int i = 0; i < segments.size(); i++) {
            StreetSegmentIdx seg = segments[i];
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if (info.oneWay && info.from != from){continue;}
            IntersectionIdx to = -1;
            if(info.from == from){
                to = info.to;
            }
            else{
                to = info.from;
            }
            float penalty = 0.0;
            if (parent[from] != -1) {
                StreetSegmentInfo prevInfo = getStreetSegmentInfo(parent[from]);
                if (prevInfo.streetID != info.streetID) {
                    penalty = turnPenalty;
                }
            }
            float travelT = findStreetSegmentTravelTime(seg) + penalty;
            if (bestTime[from] + travelT < bestTime[to]) {
                bestTime[to] = bestTime[from] + travelT;
                parent[to] = seg;
                queue.push(std::make_pair(bestTime[to], to));
            }
        }
    }

    for(IntersectionIdx to: targets){
        if(to == start || parent[to] == -1){continue;}
        std::vector<StreetSegmentIdx> path;
        IntersectionIdx curr = to;
        while(curr != start){
            StreetSegmentIdx seg = parent[curr];
            path.push_back(seg);
            StreetSegmentInfo info = getStreetSegmentInfo(seg);
            if(info.to == curr){
                curr = info.from;
            }
            else{
                curr = info.to;
            }
        }
        std::reverse(path.begin(), path.end());
        result[to] = PathInfo{bestTime[to], path};
    }
    return result;
}

void precomputePath(const std::vector<DeliveryInf> & deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty) {
    std::unordered_set<IntersectionIdx> nodes;
    for(int i = 0; i < deliveries.size(); i++){
        nodes.insert(deliveries[i].pickUp);
        nodes.insert(deliveries[i].dropOff);
    }
    for(int i = 0; i < depots.size(); i++){
        nodes.insert(depots[i]);
    }
    std::vector<IntersectionIdx> nodeList(nodes.begin(), nodes.end());
    std::vector<std::unordered_map<IntersectionIdx, PathInfo>> local_maps(nodeList.size());
    #pragma omp parallel for
    for (int i = 0; i < nodeList.size(); i++) {
        local_maps[i] = dijkstra(nodeList[i], nodes, turnPenalty);
    }
    for (int i = 0; i < nodeList.size(); i++) {
        precompute[nodeList[i]] = std::move(local_maps[i]);
    }
}

float evaluatePath(const std::vector<VisitNode>& order, IntersectionIdx depot, const std::vector<IntersectionIdx>& depots){
    std::unordered_set<int> pickedUp;
    IntersectionIdx curr = depot;
    float totalTime = 0;
    for (const auto& v : order) {
        if (!v.isPickup && pickedUp.find(v.deliveryIdx) == pickedUp.end()) {
            return 99999.0; // illegal dropoff
        }
        totalTime += precompute[curr][v.id].travel_time;
        curr = v.id;
        if (v.isPickup) pickedUp.insert(v.deliveryIdx);
    }
    float returnT = std::numeric_limits<float>::max();
    for (IntersectionIdx d : depots) {
        if (precompute[curr].count(d)) {
            float t = precompute[curr][d].travel_time;
            if (t < returnT){
                returnT = t;
            }
        }
    }
    return totalTime + returnT;
}

std::vector<VisitNode> generateLegalGreedyRoute(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, IntersectionIdx& bestDepotOut){
    std::vector<VisitNode> bestRoute;
    float bestTime = 999999.0;

    for (const auto& depot : depots) {
        std::unordered_set<int> pickedUp, droppedOff;
        std::vector<VisitNode> route;
        IntersectionIdx curr = depot;
        float totalTime = 0;

        while (route.size() < deliveries.size() * 2) {
            float min = std::numeric_limits<float>::max();
            VisitNode bestMove;
            bool found = false;
            for (int i = 0; i < deliveries.size(); ++i) {
                if (pickedUp.count(i) == false) { // not yet picked up
                    IntersectionIdx next = deliveries[i].pickUp;
                    if (precompute.count(curr) && precompute.at(curr).count(next)) {
                        float t = precompute.at(curr).at(next).travel_time;
                        if (t < min) {
                            min = t;
                            bestMove = {next, true, i};
                            found = true;
                        }
                    }
                } 
                else if (droppedOff.count(i) == false) { // not yet dropped off
                    IntersectionIdx next = deliveries[i].dropOff;
                    if (precompute.count(curr) && precompute.at(curr).count(next)) {
                        float t = precompute.at(curr).at(next).travel_time;
                        if (t < min) {
                            min = t;
                            bestMove = {next, false, i};
                            found = true;
                        }
                    }
                }
            }
            if (!found) break;
            totalTime += min;
            route.push_back(bestMove);
            curr = bestMove.id;
            if (bestMove.isPickup) pickedUp.insert(bestMove.deliveryIdx);
            else droppedOff.insert(bestMove.deliveryIdx);
        }

        float retT = 99999.0;
        for (const auto& d : depots) {
            float t = precompute[curr][d].travel_time;
            if (t < retT){
                retT = t;
            }
        }

        totalTime += retT;
        if (route.size() == deliveries.size() * 2 && totalTime < bestTime) {
            bestTime = totalTime;
            bestDepotOut = depot;
            bestRoute = route;
        }
    }

    return bestRoute;
}

void swapOrder(std::vector<VisitNode>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots){
    const int maxTrials = 100000;
    int trials = 0;
    int size = bestOrder.size();
    std::vector<VisitNode> tempOrder;
    float temperature = 1000;
    float cost = 99999999;
    
    while(trials < maxTrials){
        tempOrder = bestOrder;
        int i = rand()%size;
        int j = rand()%size;
        while (j == i) j = rand() % size;
        if (i > j) std::swap(i, j);
        std::swap(tempOrder[i], tempOrder[j]);
        for(int k=0; k<depots.size(); k++){
            float newTime = evaluatePath(tempOrder, depots[k], depots);
            if (newTime != std::numeric_limits<float>::max()){
                cost = newTime - bestTime;
            }
            if (newTime < bestTime || static_cast<float>(rand()) / RAND_MAX < std::exp(-cost/temperature)) {
                bestOrder = tempOrder;
                bestTime = newTime;
                if (cost < 0)
                    trials = 0;
            } else {
                trials++;
            }
            cost = 99999999;
            temperature = temperature * 0.98;
        }        
    }
    
    trials = 0;
    while(trials < maxTrials){
        tempOrder = bestOrder;
        int i = rand()%size;
        int j = rand()%size;
        int m = rand()%size;
        while (j == i) j = rand() % size;
        if (i > j) std::swap(i, j);
        int len = j - i;
        while (!(m + len <= size)) m = rand()%size;
        std::swap_ranges(tempOrder.begin() + i, tempOrder.begin() + j, tempOrder.begin() + m);
        for(int k=0; k<depots.size(); k++){
            float newTime = evaluatePath(tempOrder, depots[k], depots);
            if (newTime == std::numeric_limits<float>::max()){
                std::reverse(tempOrder.begin()+i, tempOrder.begin()+j);
            }
            newTime = evaluatePath(tempOrder, depots[k], depots);
            if (newTime < bestTime) {
                bestOrder = tempOrder;
                bestTime = newTime;
                trials = 0;
            } else{
                trials++;
            }
        }        
    }

}

void opt2Perturbation(std::vector<VisitNode>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<IntersectionIdx>& depots) {
    for (int round = 0; round < 30; ++round) {
        bool improved = false;
        for (int i = 0; i + 2 < bestOrder.size(); ++i) {
            for (int j = i + 2; j < bestOrder.size() && j - i <= 20; ++j) {
                std::vector<VisitNode> trialOrder = bestOrder;
                std::reverse(trialOrder.begin() + i, trialOrder.begin() + j + 1);
                std::unordered_set<int> pickedUp;
                bool legal = true;
                for (const auto& v : trialOrder) {
                    if (!v.isPickup && pickedUp.find(v.deliveryIdx) == pickedUp.end()) {
                        legal = false;
                        break;
                    }
                    if (v.isPickup) pickedUp.insert(v.deliveryIdx);
                }
                if (!legal){continue;}
                float trialTime = evaluatePath(trialOrder, bestDepot, depots);
                if (trialTime < bestTime) {
                    bestOrder = trialOrder;
                    bestTime = trialTime;
                    improved = true;
                    break;
                }
            }
            if (improved) break;
        }
        if (!improved) break;
    }
}

std::vector<CourierSubPath> buildCourierRoute(const std::vector<VisitNode>& visitList, IntersectionIdx startDepot, const std::vector<IntersectionIdx>& depots) {
    std::vector<CourierSubPath> route;
    IntersectionIdx curr = startDepot;
    for(int i = 0; i < visitList.size(); i++){
        VisitNode node = visitList[i];
        if(precompute.count(curr) == false || precompute[curr].count(node.id) == false){
            return {};
        }
        std::vector<StreetSegmentIdx> path = precompute[curr][node.id].path;
        route.push_back({{curr, node.id}, path});
        curr = node.id;
    }
    float bestReturn = 99999.0;
    IntersectionIdx bestEndDepot = depots[0];
    for(int i = 0; i < depots.size(); i++){
        float time = precompute[curr][i].travel_time;

    }
    for (const auto& d : depots) {
        float t = precompute[curr][d].travel_time;
        if (t < bestReturn) {
            bestReturn = t;
            bestEndDepot = d;
        }
    }
    const auto& path = precompute[curr][bestEndDepot].path;
    route.push_back({{curr, bestEndDepot}, path});
    return route;
}

std::vector<CourierSubPath> travelingCourier(const float turn_penalty,const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots) {
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);

    IntersectionIdx bestDepot;
    std::vector<VisitNode> bestOrder = generateLegalGreedyRoute(deliveries, depots, bestDepot);
    float bestTime = evaluatePath(bestOrder, bestDepot, depots);
    //optimization
    swapOrder(bestOrder, bestTime, bestDepot, deliveries, depots);
    opt2Perturbation(bestOrder, bestTime, bestDepot, depots);
    return buildCourierRoute(bestOrder, bestDepot, depots);
}
