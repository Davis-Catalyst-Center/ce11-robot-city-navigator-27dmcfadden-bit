#include "CityMap.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <queue>
#include <sstream>

CityMap::CityMap() {
    locations.resize(8);

    locations[0].name = "Downtown";
    locations[0].x = 4; locations[0].y = 4;
    locations[0].neighbors = {{1, 8}, {3, 15}, {6, 12}};

    locations[1].name = "Harbor";
    locations[1].x = 4; locations[1].y = 0;
    locations[1].neighbors = {{0, 8}, {2, 20}, {4, 10}};

    locations[2].name = "Airport";
    locations[2].x = 10; locations[2].y = 0;
    locations[2].neighbors = {{1, 20}, {4, 5}, {7, 18}};

    locations[3].name = "University";
    locations[3].x = 0; locations[3].y = 6;
    locations[3].neighbors = {{0, 15}, {5, 9}, {6, 7}};

    locations[4].name = "Industrial";
    locations[4].x = 9; locations[4].y = 1;
    locations[4].neighbors = {{1, 10}, {2, 5}, {7, 8}};

    locations[5].name = "Medical Center";
    locations[5].x = 2; locations[5].y = 9;
    locations[5].neighbors = {{3, 9}, {6, 11}, {7, 14}};

    locations[6].name = "Suburb North";
    locations[6].x = 1; locations[6].y = 5;
    locations[6].neighbors = {{0, 12}, {3, 7}, {5, 11}};

    locations[7].name = "Suburb South";
    locations[7].x = 8; locations[7].y = 8;
    locations[7].neighbors = {{2, 18}, {4, 8}, {5, 14}};
}

std::string CityMap::printCity() const {
    std::ostringstream output;
    output << "City Locations:\n";
    for (int i = 0; i < (int)locations.size(); i++) {
        output << "  [" << i << "] " << locations[i].name << "\n";
        output << "       neighbors: ";
        for (int j = 0; j < (int)locations[i].neighbors.size(); j++) {
            auto [idx, time] = locations[i].neighbors[j];
            output << locations[idx].name << "(" << time << ")";
            if (j < (int)locations[i].neighbors.size() - 1) output << ", ";
        }
        output << "\n";
    }
    return output.str();
}

std::pair<std::vector<std::string>, int> CityMap::greedyPath(int start, int end){
    if (start < 0 || start >= (int)locations.size() || end < 0 || end >= (int)locations.size()) {
        return {{}, -1};
    }

    std::vector<bool> visited(locations.size(), false);
    std::vector<int> pathIndixes;
    pathIndixes.push_back(start);
    int current = start;
    int totalCost = 0;

    while (current != end) {
        visited[current] = true;
        int bestNeighbor = -1;
        int bestTravelTime = 1000000000;
        int bestHeuristic = 1000000000;
        for (const auto& neighbor : locations[current].neighbors) {
            int nextIndex = neighbor.first;
            int travelTime = neighbor.second;
            if (visited[nextIndex]) {
                continue;
            }
            int h = heuristic(nextIndex, end);
            if (travelTime < bestTravelTime
                || (travelTime == bestTravelTime && h < bestHeuristic)
                || (travelTime == bestTravelTime && h == bestHeuristic && nextIndex < bestNeighbor)) {
                bestTravelTime = travelTime;
                bestNeighbor = nextIndex;
                bestHeuristic = h;
            }
        }
        if (bestNeighbor < 0) {
            return {{}, -1};
        }
        totalCost += bestTravelTime;
        current = bestNeighbor;
        pathIndixes.push_back(current);
    }
    std::vector<std::string> path;
    path.reserve(pathIndixes.size());
    for (int index : pathIndixes) {
        path.push_back(locations[index].name);
    }
    return {path, totalCost};
}
std::pair<std::vector<std::string>, int> CityMap::dijkstraPath(int start, int end){
    if (start < 0 || start >= (int)locations.size() || end < 0 || end >= (int)locations.size()) {
        return {{}, -1};
    }

    using Node = std::pair<int, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<int> dist(locations.size(), 1000000000);
    std::vector<int> prev(locations.size(), -1);
    dist[start] = 0;
    pq.push({0, start});
    while (!pq.empty()) {
        auto [currentDist, current] = pq.top();
        pq.pop();
        if (currentDist > dist[current]) {
            continue;
        }
        if (current == end) {
            break;
        }
        for (const auto& neighbor : locations[current].neighbors) {
            int nextIndex = neighbor.first;
            int travelTime = neighbor.second;
            int candidateDist = currentDist + travelTime;
            if (candidateDist < dist[nextIndex]) {
                dist[nextIndex] = candidateDist;
                prev[nextIndex] = current;
                pq.push({candidateDist, nextIndex});
            }
        }
    }
    if (dist[end] == 1000000000) {
        return {{}, -1};
    }
    return {reconstructPath(prev, start, end), dist[end]};
}
std::pair<std::vector<std::string>, int> CityMap::aStarPath(int start, int end){
    if (start < 0 || start >= (int)locations.size() || end < 0 || end >= (int)locations.size()) {
        return {{}, -1};
    }

    using Node = std::pair<int, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    std::vector<int> gScore(locations.size(), 1000000000);
    std::vector<int> prev(locations.size(), -1);

    gScore[start] = 0;
    pq.push({heuristic(start, end), start});

    while (!pq.empty()) {
        auto [currentF, current] = pq.top();
        pq.pop();

        if (current == end) {
            break;
        }

        int currentG = gScore[current];
        if (currentF > currentG + heuristic(current, end)) {
            continue;
        }

        for (const auto& neighbor : locations[current].neighbors) {
            int nextIndex = neighbor.first;
            int travelTime = neighbor.second;
            int tentativeG = currentG + travelTime;
            if (tentativeG < gScore[nextIndex]) {
                gScore[nextIndex] = tentativeG;
                prev[nextIndex] = current;
                int nextF = tentativeG + heuristic(nextIndex, end);
                pq.push({nextF, nextIndex});
            }
        }
    }

    if (gScore[end] == 1000000000) {
        return {{}, -1};
    }

    return {reconstructPath(prev, start, end), gScore[end]};
}

int CityMap::heuristic(int from, int to) const{
    int dx = locations[from].x - locations[to].x;
    int dy = locations[from].y - locations[to].y;
    return static_cast<int>(std::sqrt(dx * dx + dy * dy));
}

std::vector<std::string> CityMap::reconstructPath(const std::vector<int>& prev, int start, int end) const {
    std::vector<std::string> places;
    if (start < 0 || start >= (int)locations.size() || end < 0 || end >= (int)locations.size()) {
        return places;
    }

    std::vector<int> reversedPath;
    int current = end;
    while (current != -1) {
        reversedPath.push_back(current);
        if (current == start) {
            break;
        }
        current = prev[current];
    }

    if (reversedPath.empty() || reversedPath.back() != start) {
        return places;
    }

    places.reserve(reversedPath.size());
    for (int i = static_cast<int>(reversedPath.size()) - 1; i >= 0; --i) {
        places.push_back(locations[reversedPath[i]].name);
    }

    return places;
}