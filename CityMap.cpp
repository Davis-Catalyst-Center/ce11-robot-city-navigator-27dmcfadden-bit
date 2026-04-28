#include "CityMap.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#include <string>

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

void CityMap::printCity() const {
    std::cout << "City Locations:\n";
    for (int i = 0; i < (int)locations.size(); i++) {
        std::cout << "  [" << i << "] " << locations[i].name << "\n";
        std::cout << "       neighbors: ";
        for (int j = 0; j < (int)locations[i].neighbors.size(); j++) {
            auto [idx, time] = locations[i].neighbors[j];
            std::cout << locations[idx].name << "(" << time << ")";
            if (j < (int)locations[i].neighbors.size() - 1) std::cout << ", ";
        }
        std::cout << "\n";
    }
}

std::pair<std::vector<std::string>, int> CityMap::greedyPath(int start, int end){

}
std::pair<std::vector<std::string>, int> CityMap::dijkstraPath(int start, int end){

}
std::pair<std::vector<std::string>, int> CityMap::aStarPath(int start, int end){

}

int CityMap::heuristic(int from, int to) const{
    return sqrt((locations[from].x-locations[to].x)^2 + (locations[from].y-locations[to].y)^2);
}

std::vector<std::string> CityMap::reconstructPath(const std::vector<int>& prev, int start, int end) const {
    std::vector<std::string> places;/*
    if (locations.empty()){
        return places;
    }*/
    for (int i = locations.size(); i > 0; i--){
        places.push_back(locations[i].name);
    }
    return places;

}