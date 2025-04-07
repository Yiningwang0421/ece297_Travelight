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
void runACOFromDepot(IntersectionIdx depot,
                     const std::vector<DeliveryInf>& deliveries,
                     const std::vector<IntersectionIdx>& depots,
                     std::vector<int>& bestOrder,
                     float& bestTime);
bool buildAntSolution(std::vector<int>& order,
                      IntersectionIdx& startDepot,
                      const std::vector<DeliveryInf>& deliveries,
                      IntersectionIdx depot,
                      const std::vector<std::vector<float>>& pheromone,
                      float alpha, float beta);
float evaluatePath(const std::vector<int>& order,
                   IntersectionIdx depot,
                   const std::vector<DeliveryInf>& deliveries,
                   const std::vector<IntersectionIdx>& depots);
void buildGreedyOrder(
    std::vector<int>& bestOrder,
    float& bestTime,
    IntersectionIdx& bestDepot,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots
);


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

std::vector<CourierSubPath> travelingCourier(const float turn_penalty, const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots) {
    precompute.clear();
    precomputePath(deliveries, depots, turn_penalty);

    std::vector<int> bestOrder;
    float bestTime = std::numeric_limits<float>::max();
    IntersectionIdx bestDepot = depots[0];

    #pragma omp parallel for
    for (int i = 0; i < depots.size(); i++) {
        const IntersectionIdx depot = depots[i];
        std::vector<int> localOrder;
        float localTime;
        runACOFromDepot(depot, deliveries, depots, localOrder, localTime);

        #pragma omp critical
        {
            if (localTime < bestTime) {
                bestTime = localTime;
                bestOrder = localOrder;
                bestDepot = depot;
            }
        }
    }

    // 构造最终路径
    std::vector<CourierSubPath> route;
    std::unordered_set<int> pickedUp;
    IntersectionIdx curr = bestDepot;

    for (int code : bestOrder) {
        int idx = code / 2;
        bool isPickup = (code % 2 == 0);
        IntersectionIdx next = isPickup ? deliveries[idx].pickUp : deliveries[idx].dropOff;
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

float evaluatePath(const std::vector<int>& order,
                   IntersectionIdx depot,
                   const std::vector<DeliveryInf>& deliveries,
                   const std::vector<IntersectionIdx>& depots) {
    std::unordered_set<int> pickedUp;
    IntersectionIdx curr = depot;
    float totalTime = 0;

    for (int code : order) {
        int idx = code / 2;
        bool isPickup = (code % 2 == 0);
        IntersectionIdx next = isPickup ? deliveries[idx].pickUp : deliveries[idx].dropOff;

        if (!precompute[curr].count(next)) return std::numeric_limits<float>::max();
        if (!isPickup && !pickedUp.count(idx)) return std::numeric_limits<float>::max();

        totalTime += precompute[curr][next].travel_time;
        curr = next;
        if (isPickup) pickedUp.insert(idx);
    }

    float retT = std::numeric_limits<float>::max();
    for (IntersectionIdx d : depots) {
        if (precompute[curr].count(d)) {
            retT = std::min(retT, precompute[curr][d].travel_time);
        }
    }
    return totalTime + retT;
}

bool buildAntSolution(std::vector<int>& order,
                      IntersectionIdx& startDepot,
                      const std::vector<DeliveryInf>& deliveries,
                      IntersectionIdx depot,
                      const std::vector<std::vector<float>>& pheromone,
                      float alpha, float beta) {
    int N = deliveries.size();
    int V = 2 * N;
    std::unordered_set<int> pickedUp, droppedOff;
    std::vector<int> remaining;

    for (int i = 0; i < N; ++i) {
        remaining.push_back(i * 2);     // pickup
        remaining.push_back(i * 2 + 1); // dropoff
    }

    std::vector<int> visited;
    startDepot = depot;
    IntersectionIdx curr = startDepot;
    int currCode = -1;

    while (!remaining.empty()) {
        std::vector<int> candidates;

        for (int code : remaining) {
            int idx = code / 2;
            bool isPickup = (code % 2 == 0);
            if (isPickup || (pickedUp.count(idx) && !droppedOff.count(idx))) {
                candidates.push_back(code);
            }
        }

        if (candidates.empty()) return false;

        std::vector<double> probs;
        double sum = 0.0;
        for (int nextCode : candidates) {
            int idx = nextCode / 2;
            bool isPickup = (nextCode % 2 == 0);
            IntersectionIdx next = isPickup ? deliveries[idx].pickUp : deliveries[idx].dropOff;
            if (!precompute[curr].count(next)) continue;

            float dist = precompute[curr][next].travel_time + 1e-3f;
            double tau = pheromone[(currCode == -1) ? idx : currCode][nextCode];
            double eta = 1.0 / dist;
            double score = pow(tau, alpha) * pow(eta, beta);
            probs.push_back(score);
            sum += score;
        }

        if (probs.empty()) return false;

        double r = ((double) rand() / RAND_MAX) * sum;
        double acc = 0.0;
        int chosenIdx = -1;
        for (int i = 0; i < candidates.size(); ++i) {
            acc += probs[i];
            if (r <= acc) {
                chosenIdx = candidates[i];
                break;
            }
        }

        if (chosenIdx == -1) return false;

        visited.push_back(chosenIdx);
        int idx = chosenIdx / 2;
        if (chosenIdx % 2 == 0) pickedUp.insert(idx);
        else droppedOff.insert(idx);
        curr = (chosenIdx % 2 == 0) ? deliveries[idx].pickUp : deliveries[idx].dropOff;
        currCode = chosenIdx;
        remaining.erase(std::remove(remaining.begin(), remaining.end(), chosenIdx), remaining.end());
    }

    order = visited;
    return true;
}

void runACOFromDepot(IntersectionIdx depot,
                     const std::vector<DeliveryInf>& deliveries,
                     const std::vector<IntersectionIdx>& depots,
                     std::vector<int>& bestOrder,
                     float& bestTime) {

    const int numAnts = 80;
    const int maxIter = 40;
    const float alpha = 1.0f;
    const float beta = 2.5f;
    const float rho = 0.1f;
    const float Q = 10000.0f;

    int N = deliveries.size();
    int V = 2 * N;

    std::vector<std::vector<float>> pheromone(V, std::vector<float>(V, 1.0f));

    // ✨ 贪心路径初始化信息素
    std::vector<int> greedyOrder;
    float greedyTime = std::numeric_limits<float>::max();
    IntersectionIdx dummyDepot = depot;

    buildGreedyOrder(greedyOrder, greedyTime, dummyDepot, deliveries, {depot});

    for (int i = 0; i + 1 < greedyOrder.size(); ++i) {
        pheromone[greedyOrder[i]][greedyOrder[i + 1]] = 10.0f;
    }

    bestTime = std::numeric_limits<float>::max();

    for (int iter = 0; iter < maxIter; iter++) {
        for (int ant = 0; ant < numAnts; ant++) {
            std::vector<int> order;
            IntersectionIdx dummy = depot;

            if (!buildAntSolution(order, dummy, deliveries, depot, pheromone, alpha, beta)) continue;

            float time = evaluatePath(order, depot, deliveries, depots);
            if (time < bestTime) {
                bestTime = time;
                bestOrder = order;

                // ✅ 本地搜索优化
                swapOrder(bestOrder, bestTime, depot, deliveries, depots);
                opt2Perturbation(bestOrder, bestTime, depot, deliveries, depots);
            }

            // 信息素更新
            for (int i = 0; i + 1 < order.size(); i++) {
                pheromone[order[i]][order[i + 1]] *= (1.0f - rho);
                pheromone[order[i]][order[i + 1]] += Q / time;
            }
        }
    }
}

void buildGreedyOrder(
    std::vector<int>& bestOrder,
    float& bestTime,
    IntersectionIdx& bestDepot,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots
) {
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

            // Try dropoffs first
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

            // Try pickups
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

        // Evaluate total time
        pickedUp.clear();
        droppedOff.clear();
        curr = depot;
        float totalTime = 0;

        for (int idx : order) {
            IntersectionIdx next = pickedUp.count(idx) ? deliveries[idx].dropOff : deliveries[idx].pickUp;
            if (!precompute[curr].count(next)) {
                totalTime = std::numeric_limits<float>::max();
                break;
            }
            totalTime += precompute[curr][next].travel_time;
            curr = next;
            if (pickedUp.count(idx)) droppedOff.insert(idx);
            else pickedUp.insert(idx);
        }

        // Add return-to-depot time
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

    // Convert to 2N format (i*2 = pickup, i*2+1 = dropoff)
    std::vector<int> convertedOrder;
    std::unordered_set<int> pickedUp;
    for (int idx : bestOrder) {
        if (!pickedUp.count(idx)) {
            convertedOrder.push_back(idx * 2);
            pickedUp.insert(idx);
        } else {
            convertedOrder.push_back(idx * 2 + 1);
        }
    }
    bestOrder = convertedOrder;
}