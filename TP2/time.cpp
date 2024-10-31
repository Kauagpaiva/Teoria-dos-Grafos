#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <random>
#include <chrono>
#include "graph.cpp"

using namespace std;

int main(){
    int nExecutions = 2;
    WeightedGraph g(0);
    g.readGraphFromFile("path-to-graph.txt"); // Carrega o grafo do arquivo

    vector<long long> executionTimes;

    std::random_device rd;
    std::mt19937 mt(rd()); // Mersenne Twister pseudo-random number generator
    std::uniform_int_distribution<int> distribution(1, g.getVertices()); // Generate integers between 1 and nVertices

    for(int i=0; i < nExecutions; i++){
        int randomNum = distribution(mt);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        // function being timed...
        g.dijkstra(randomNum);
        //g1.dijkstraWithHeap(randomNum);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;
        executionTimes.push_back(duration.count());
    }
    
    long long totalExecutionTime = 0;
    for (long long time : executionTimes) {
        totalExecutionTime += time;
    }

    long long meanExecutionTime = totalExecutionTime / nExecutions;

    cout << "Mean Execution Time: " << meanExecutionTime << " milliseconds" << endl;
    
    ofstream fout("time.tsx", std::ios::app);

    if (!fout){
        cout<<endl;
        cout << "Erro ao abrir o arquivo." << endl;
        return 0;
    }

    fout << "Mean Execution Time Vector Dijkstra: " << meanExecutionTime << " milliseconds" << endl;

    return 0;
}