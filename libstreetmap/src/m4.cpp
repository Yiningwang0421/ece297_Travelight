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
#include <chrono> 
#define TIME_LIMIT 10

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
std::vector<VisitNode> threeOptVisit(const std::vector<VisitNode>& order,
                                     IntersectionIdx depot,
                                     const std::vector<DeliveryInf>& deliveries,
                                     const std::vector<IntersectionIdx>& depots);
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
    const int maxRounds = 10;
    const int maxWindowSize = bestOrder.size()/3;
    //for (int round = 0; round < maxRounds; ++round) {
        for (int i = 0; i + 2 < bestOrder.size(); ++i) {
            for (int j = i + 2; j < bestOrder.size() && j - i <= maxWindowSize; ++j) {
                std::vector<VisitNode> trialOrder = bestOrder;
                std::reverse(trialOrder.begin() + i, trialOrder.begin() + j + 1);
                float trialTime = evaluatePath(trialOrder, bestDepot, depots);
                if (trialTime < bestTime) {
                    bestOrder = trialOrder;
                    bestTime = trialTime;
                }
            }
        }
    //}
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

bool isLegalVisitOrder(const std::vector<VisitNode>& order) {
    std::unordered_map<int, bool> pickedUp; 
    for (const auto &node : order) {
        if (pickedUp.find(node.deliveryIdx) == pickedUp.end()) {
            
            if (!node.isPickup) return false;
            pickedUp[node.deliveryIdx] = true;
        } else {
           
            if (node.isPickup) return false;
        }
    }
    return true;
}


std::vector<VisitNode> threeOptVisit(const std::vector<VisitNode>& order,
                                     IntersectionIdx depot,
                                     const std::vector<DeliveryInf>& deliveries,
                                     const std::vector<IntersectionIdx>& depots) {
    std::vector<VisitNode> bestOrder = order;
    float bestCost = evaluatePath(order, depot, depots);
    bool improvement = true;
    int n = bestOrder.size();

    int N;
    if (n < 175) N = 1;
    else if (n < 350) N = 2;
    else if (n < 500) N = 4;
    else N = 8;

    if (n < 4) return bestOrder;

    auto start_time = std::chrono::high_resolution_clock::now();
    const int TIME_LIMIT_MS = 35;

    while (improvement) {
        improvement = false;
        for (int i = 1; i < n - 2; i += N) {
            for (int j = i + 1; j < n - 1; j += N) {
                for (int k = j + 1; k < n; k += N) {

                    auto now = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
                    if (duration > TIME_LIMIT_MS) {
                        std::cout << "threeOptVisit timeout at " << duration << " ms\n";
                        return bestOrder;
                    }

                    
                    std::vector<VisitNode> S1(bestOrder.begin(), bestOrder.begin() + i);
                    std::vector<VisitNode> S2(bestOrder.begin() + i, bestOrder.begin() + j);
                    std::vector<VisitNode> S3(bestOrder.begin() + j, bestOrder.begin() + k);
                    std::vector<VisitNode> S4(bestOrder.begin() + k, bestOrder.end());

                    #pragma omp parallel for
                    for (int opt = 0; opt < 7; opt++) {
                        std::vector<VisitNode> candidate;

                        switch (opt) {
                            case 0: { // reverse S2
                                candidate = S1;
                                std::vector<VisitNode> revS2 = S2;
                                std::reverse(revS2.begin(), revS2.end());
                                candidate.insert(candidate.end(), revS2.begin(), revS2.end());
                                candidate.insert(candidate.end(), S3.begin(), S3.end());
                                candidate.insert(candidate.end(), S4.begin(), S4.end());
                                break;
                            }
                            case 1: { // reverse S3
                                candidate = S1;
                                candidate.insert(candidate.end(), S2.begin(), S2.end());
                                std::vector<VisitNode> revS3 = S3;
                                std::reverse(revS3.begin(), revS3.end());
                                candidate.insert(candidate.end(), revS3.begin(), revS3.end());
                                candidate.insert(candidate.end(), S4.begin(), S4.end());
                                break;
                            }
                            case 2: { // reverse S2 + S3
                                candidate = S1;
                                std::vector<VisitNode> revS2 = S2;
                                std::vector<VisitNode> revS3 = S3;
                                std::reverse(revS2.begin(), revS2.end());
                                std::reverse(revS3.begin(), revS3.end());
                                candidate.insert(candidate.end(), revS2.begin(), revS2.end());
                                candidate.insert(candidate.end(), revS3.begin(), revS3.end());
                                candidate.insert(candidate.end(), S4.begin(), S4.end());
                                break;
                            }
                            case 3: { // swap S2 & S3
                                candidate = S1;
                                candidate.insert(candidate.end(), S3.begin(), S3.end());
                                candidate.insert(candidate.end(), S2.begin(), S2.end());
                                candidate.insert(candidate.end(), S4.begin(), S4.end());
                                break;
                            }
                            case 4: { // S3 + reverse(S2)
                                candidate = S1;
                                candidate.insert(candidate.end(), S3.begin(), S3.end());
                                std::vector<VisitNode> revS2 = S2;
                                std::reverse(revS2.begin(), revS2.end());
                                candidate.insert(candidate.end(), revS2.begin(), revS2.end());
                                candidate.insert(candidate.end(), S4.begin(), S4.end());
                                break;
                            }
                            case 5: { // reverse(S3) + S2
                                candidate = S1;
                                std::vector<VisitNode> revS3 = S3;
                                std::reverse(revS3.begin(), revS3.end());
                                candidate.insert(candidate.end(), revS3.begin(), revS3.end());
                                candidate.insert(candidate.end(), S2.begin(), S2.end());
                                candidate.insert(candidate.end(), S4.begin(), S4.end());
                                break;
                            }
                            case 6: { // reverse(S2 + S3)
                                candidate = S1;
                                std::vector<VisitNode> revS2S3 = S2;
                                revS2S3.insert(revS2S3.end(), S3.begin(), S3.end());
                                std::reverse(revS2S3.begin(), revS2S3.end());
                                candidate.insert(candidate.end(), revS2S3.begin(), revS2S3.end());
                                candidate.insert(candidate.end(), S4.begin(), S4.end());
                                break;
                            }
                        }

                        if (!isLegalVisitOrder(candidate)) continue;

                        float candCost = evaluatePath(candidate, depot, depots);

                        
                        #pragma omp critical
                        {
                            if (candCost < bestCost) {
                                bestCost = candCost;
                                bestOrder = candidate;
                                improvement = true;
                            }
                        }
                    } // end of parallel for
                }
            }
        }
    }

    return bestOrder;
}


std::vector<CourierSubPath> travelingCourier(const float turn_penalty,const std::vector<DeliveryInf>& deliveries,const std::vector<IntersectionIdx>& depots) {
    auto startTime = std::chrono::high_resolution_clock::now();
    bool timeOut = false;
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);
    IntersectionIdx bestDepot;
    std::vector<VisitNode> bestOrder = GreedyHeuristic(deliveries, depots, bestDepot);
    float bestTime = evaluatePath(bestOrder, bestDepot, depots);
    
//    std::vector<std::pair<std::vector<VisitNode>, float>> orders;
//    orders.resize(20);
//    for(int i=0; i<orders.size(); i++){
//        orders[i] = {bestOrder, bestTime};
//    }
    /*//optimization
    int iteration = 0;
    while(!timeOut && iteration<1){
        swapOrder(bestOrder, bestTime, bestDepot, deliveries, depots);
        opt2Perturbation(bestOrder, bestTime, bestDepot, depots);
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto wallClock = std::chrono::duration_cast<std::chrono::duration<double>> (currentTime - startTime);
        if(wallClock.count() > 0.9 * TIME_LIMIT){
            timeOut = true;
        }
        iteration++;
    }*/
    swapOrder(bestOrder, bestTime, bestDepot, deliveries, depots);
        opt2Perturbation(bestOrder, bestTime, bestDepot, depots);
    bestOrder = threeOptVisit(bestOrder, bestDepot, deliveries, depots); 

    return buildCourierRoute(bestOrder, bestDepot, depots);
}

