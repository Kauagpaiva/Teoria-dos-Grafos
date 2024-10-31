#include <iostream>
#include <vector>
#include "graph.cpp"

using namespace std;

int main() {

    WeightedGraph graph(0); // Cria um grafo vazio
    graph.readGraphFromFile("path-to-graph.txt"); // Carrega o grafo do arquivo

    int startNode = 10; // Defina o vértice inicial
    auto [distances, parents] = graph.dijkstra(startNode); // Aplica Dijkstra

    // Imprime o índice e o valor de cada elemento
    cout << "Index " << 20 << ": " << distances[20] << endl;
    cout << "Index " << 30 << ": " << distances[30] << endl;
    cout << "Index " << 40 << ": " << distances[40] << endl;
    cout << "Index " << 50 << ": " << distances[50] << endl;
    cout << "Index " << 60 << ": " << distances[60] << endl;

    return 0;
}
