#include <iostream>
#include <vector>
#include <fstream>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>
#include <numeric>
#include "graph.cpp"

int main() {
    Graph g(0, false); // Inicializa com 0 vértices, pois serão lidos do arquivo, não utilizando matriz adjacencia
    g.loadFromFile("C:/Users/kauag/Documents/GitHub/Teoria-dos-Grafos/data/grafo_2.txt"); //lendo o arquivo

    // Percorrer o grafo usando BFS e salvar as informações
    g.bfs(1);

    // Percorrer o grafo usando DFS e salvar as informações
    g.dfs(1);

    // Calcular as estatísticas de grau
    g.calculateDegreeStats();

    // Descobrir componentes conexas
    g.connectedComponents();

    return 0;
}