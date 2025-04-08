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

struct VisitNode {
    IntersectionIdx id;
    bool isPickup;
    int deliveryIdx;
};
std::unordered_map<IntersectionIdx, std::unordered_map<IntersectionIdx, PathInfo>> precompute;
std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty);
void precomputePath(const std::vector<DeliveryInf> & deliveries, const std::vector<IntersectionIdx>& depots, float turnPenalty);
float evaluatePath(const std::vector<int>& order, IntersectionIdx depot, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
std::vector<VisitNode> GreedyHeuristic(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, IntersectionIdx& bestDepotOut);

std::unordered_map<IntersectionIdx, PathInfo> dijkstra(IntersectionIdx start, const std::unordered_set<IntersectionIdx>& target, float turnPenalty) {
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
    for (int i = 0; i < nodeList.size(); ++i) {
        local_maps[i] = dijkstra(nodeList[i], nodes, turnPenalty);
    }
    for (int i = 0; i < nodeList.size(); ++i) {
        precompute[nodeList[i]] = std::move(local_maps[i]);
    }
}

float evaluatePath(const std::vector<VisitNode>& order, IntersectionIdx depot, const std::vector<IntersectionIdx>& depots){
    std::unordered_set<int> pickedUp;
    IntersectionIdx curr = depot;
    float totalTime = 0;
    for(int i = 0; i < order.size(); i++){
        if(order[i].isPickup == false && pickedUp.find(order[i].deliveryIdx) == pickedUp.end()){
            return 999999;
        }
        totalTime += precompute[curr][order[i].id].travel_time;
        curr = order[i].id;
        if(order[i].isPickup == true){
            pickedUp.insert(order[i].deliveryIdx);
        }
    }
    float returnT = 999999;
    for (IntersectionIdx d : depots) {
        if (precompute[curr].find(d) != precompute[curr].end()) {
            float t = precompute[curr][d].travel_time;
            if (t < returnT) returnT = t;
        }
    }
    return totalTime + returnT;
}

std::vector<VisitNode> GreedyHeuristic(const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots, IntersectionIdx& bestDepotOut){
    std::vector<VisitNode> bestRoute;
    float bestTime = 999999.0;
    for(int depotIdx = 0; depotIdx  < depots.size(); depotIdx++){
        std::unordered_set<int> pickedUp;
        std::unordered_set<int> droppedOff;
        std::vector<VisitNode> route;
        IntersectionIdx curr = depots[depotIdx];
        float totalTime = 0;
        while (route.size() < deliveries.size() * 2) {
            float minT = 999999;
            VisitNode bestMove;
            bool found = false;
            for (int i = 0; i < deliveries.size(); ++i) {
                //next node is pickup
                if (pickedUp.count(i) == 0) {
                    IntersectionIdx next = deliveries[i].pickUp;
                    if (precompute.count(curr) > 0 && precompute[curr].count(next) > 0) {
                        float t = precompute[curr][next].travel_time;
                        if (t < minT) {
                            minT = t;
                            bestMove = {next, true, i};
                            found = true;
                        }
                    }
                } 
                //not pickup, check if it's dropoff
                else if (droppedOff.count(i) == 0){
                    IntersectionIdx next = deliveries[i].dropOff;
                    if (precompute.count(curr) > 0 && precompute[curr].count(next) > 0) {
                        float t = precompute[curr][next].travel_time;
                        if (t < minT) {
                            minT = t;
                            bestMove = {next, false, i};
                            found = true;
                        }
                    }
                }
            }
            if (found == false){ break;}
            totalTime += minT;
            route.push_back(bestMove);
            curr = bestMove.id;
            if (bestMove.isPickup){
                pickedUp.insert(bestMove.deliveryIdx);
            }
            else{
                droppedOff.insert(bestMove.deliveryIdx);
            }
        }

        float minT = 999999;
        for(int i = 0; i < depots.size(); i++){
            if(precompute.count(depots[i]) >= 0 && precompute[curr].count(depots[i]) > 0){
                float t = precompute[curr][depots[i]].travel_time;
                if(t < minT){
                    minT = t;
                }
            }
        }
        totalTime += minT;
        if (route.size() == deliveries.size() * 2 && totalTime < bestTime) {
            bestTime = totalTime;
            bestDepotOut = depots[depotIdx];
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
        while (j == i){
            j = rand() % size;
        }
        if (i > j){
            std::swap(i, j);
        }
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
        while (j == i){
            j = rand() % size;
        }
        if (i > j){
            std::swap(i, j);
        }
        int len = j - i;
        while (!(m + len <= size)){
            m = rand()%size;
        }
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
            } 
            else{
                trials++;
            }
        }        
    }
}

void opt2Perturbation(std::vector<VisitNode>& bestOrder, float& bestTime, IntersectionIdx bestDepot, const std::vector<IntersectionIdx>& depots) {
    const int maxRounds = 30;
    for (int round = 0; round < maxRounds; ++round) {
        bool improved = false;
        for (int i = 0; i + 2 < bestOrder.size(); ++i) {
            for (int j = i + 2; j < bestOrder.size() && j - i <= 20; ++j) {
                std::vector<VisitNode> trialOrder = bestOrder;
                std::reverse(trialOrder.begin() + i, trialOrder.begin() + j + 1);
                std::unordered_set<int> pickedUp;
                bool legal = true;
                for(int currOrder = 0; currOrder < trialOrder.size(); currOrder ++){
                    if(trialOrder[currOrder].isPickup == false && pickedUp.find(trialOrder[currOrder].deliveryIdx) == pickedUp.end()){
                        legal = false;
                        break;
                    }
                    if(trialOrder[currOrder].isPickup == true){
                        pickedUp.insert(trialOrder[currOrder].deliveryIdx);
                    }
                }
                if (legal == false){
                    continue;
                }
                float newTime = evaluatePath(trialOrder, bestDepot, depots);
                if (newTime < bestTime) {
                    bestOrder = trialOrder;
                    bestTime = newTime;
                    improved = true;
                    break;
                }
            }
            if (improved == true) {
                break;
            }
        }
        if(improved == false){
            break;
        }
    }
}

std::vector<CourierSubPath> buildCourierRoute(const std::vector<VisitNode>& visitList, IntersectionIdx startDepot, const std::vector<IntersectionIdx>& depots) {
    std::vector<CourierSubPath> route;
    IntersectionIdx curr = startDepot;
    for(int i = 0; i < visitList.size(); i++){
        if(precompute.find(curr) == precompute.end() || precompute[curr].find(visitList[i].id) == precompute[curr].end()){
            return {};
        }
        route.push_back(CourierSubPath({{curr, visitList[i].id}, precompute[curr][visitList[i].id].path}));
        curr = visitList[i].id;
    }
    float bestTime = 99999.0;
    IntersectionIdx bestEndDepot = depots[0];
    for(int i = 0; i <  depots.size(); i++){
        float t = precompute[curr][depots[i]].travel_time;
        if(t < bestTime){
            bestTime = t;
            bestEndDepot = depots[i];
        }
    }
    route.push_back({{curr, bestEndDepot}, precompute[curr][bestEndDepot].path});
    return route;
}

std::vector<CourierSubPath> travelingCourier(const float turn_penalty,const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots) {
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);
    IntersectionIdx bestDepot;
    std::vector<VisitNode> bestOrder = GreedyHeuristic(deliveries, depots, bestDepot);
    float bestTime = evaluatePath(bestOrder, bestDepot, depots);
    //optimization
    for(int i = 0; i < 3; i++){
        swapOrder(bestOrder, bestTime, bestDepot, deliveries, depots);
        opt2Perturbation(bestOrder, bestTime, bestDepot, depots);
    }
    return buildCourierRoute(bestOrder, bestDepot, depots);
}
