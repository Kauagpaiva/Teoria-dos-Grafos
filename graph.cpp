#include <iostream>
#include <vector>
#include <fstream>
#include <queue>
#include <algorithm>
#include <limits>
#include <cmath>

using namespace std;

class Graph {
    int V; // Número de vértices
    vector<vector<int>> adjList; // Lista de adjacência
    vector<vector<int>> adjMatrix; // Matriz de adjacência
    bool useAdjMatrix; // Para definir o tipo de representação
    int numEdges; // numero de arestas

public:
    // Construtor
    Graph(int V, bool useMatrix) : V(V), useAdjMatrix(useMatrix), numEdges(0) {
        //adjList.resize(V + 1);
        //adjMatrix.resize(V + 1, vector<int>(V + 1, 0));
    }

    int getV() const { return V; } // Método para obter número de vértices
    int getEdgeCount() const { return numEdges; } // Método para obter número de arestas

    // Método para adicionar aresta
    void addEdge(int u, int v) {
        if (useAdjMatrix) {
            adjMatrix[u][v] = 1;
            adjMatrix[v][u] = 1;
        } else {
            adjList[u].push_back(v);
            adjList[v].push_back(u);
        }
        numEdges++; //conta uma aresta adicionada
    }

    // Método para carregar o grafo a partir de um arquivo
    void loadFromFile(const string &filename) {
        ifstream file(filename);

        if (!file.is_open()) {
            cerr << "Erro ao abrir o arquivo!" << endl;
            return;
        }
        
        file >> V; // Lê o número de vértices do grafo do arquivo.
        adjList.resize(V + 1);
        adjMatrix.resize(V + 1, vector<int>(V + 1, 0));

        int u, v; 
        while (file >> u >> v) { //lê pares de inteiros (as arestas) do arquivo até o final do arquivo.
            addEdge(u, v);
        }

        file.close(); //Fecha o arquivo após a leitura estar completa.
    }

    // BFS para percorrer o grafo
    void bfs(int start) {
        vector<bool> visited(V + 1, false); // lista de nós visitados
        vector<int> parent(V + 1, -1); //lista dos pais
        vector<int> level(V + 1, -1); //lista dos niveis

        queue<int> q; //lista de nós descobertos
        visited[start] = true; //marca o nó inicial como visitado
        level[start] = 0; //marca o nivel do nó inicial como 0
        q.push(start); //adiciona o nó inicial à fila

        while (!q.empty()) { //enquanto a lista de nós descobertos não estiver vazia
            int u = q.front(); // pega o primeiro nó da lista
            q.pop(); //remove ele da lista descobertos

            for (int v : (useAdjMatrix ? getNeighborsMatrix(u) : adjList[u])) { //para cada nó vizinho de u
                if (!visited[v]) { //se v não foi visitado ainda
                    visited[v] = true; //visita v
                    parent[v] = u; //define o pai de v como u
                    level[v] = level[u] + 1; //define o nivel de v
                    q.push(v); //remove v da lista de descobertos (agora foi pra lista de visitados)
                }
            }
        }

        ofstream output("bfs_tree.txt");
        for (int i = 1; i <= V; ++i) { //para cada nó informa o pai e o nível do nó no arquivo output
            output << "Vértice: " << i << ", Pai: " << parent[i] << ", Nível: " << level[i] << endl;
        }
        output.close();
    }


    void dfs(int start) {
        vector<bool> visited(V + 1, false);
        vector<int> parent(V + 1, -1);
        vector<int> level(V + 1, -1);

        ofstream output("dfs_tree.txt");
        level[start] = 0;
        dfsUtil(start, visited, parent, level, output);
        output.close();
    }

    void dfsUtil(int u, vector<bool> &visited, vector<int> &parent, vector<int> &level, ofstream &output) {
        visited[u] = true;

        for (int v : (useAdjMatrix ? getNeighborsMatrix(u) : adjList[u])) {
            if (!visited[v]) {
                parent[v] = u;
                level[v] = level[u] + 1;
                dfsUtil(v, visited, parent, level, output);
            }
        }

        output << "Vértice: " << u << ", Pai: " << parent[u] << ", Nível: " << level[u] << endl;
    }


    // Função para calcular o grau mínimo, máximo, médio e mediana
    void calculateDegreeStats() {
        vector<int> degrees(V + 1, 0);
        for (int i = 1; i <= V; ++i) {
            degrees[i] = (useAdjMatrix ? getNeighborsMatrix(i) : adjList[i]).size();
        }

        // Ordenar os graus para encontrar a mediana
        sort(degrees.begin() + 1, degrees.end());

        int minDegree = degrees[1]; //grau minimo
        int maxDegree = degrees[V]; //grau maximo
        double avgDegree = accumulate(degrees.begin() + 1, degrees.end(), 0.0) / V; //grau medio
        double medianDegree = (V % 2 == 0) ? (degrees[V / 2] + degrees[V / 2 + 1]) / 2.0 : degrees[V / 2 + 1]; //mediana de grau
        
        // Salvar no arquivo
        ofstream output("graph_info.txt");
        output << "Número de vértices: " << V << endl;
        output << "Número de arestas: " << numEdges << endl;
        output << "Grau mínimo: " << minDegree << endl;
        output << "Grau máximo: " << maxDegree << endl;
        output << "Grau médio: " << avgDegree << endl;
        output << "Mediana do grau: " << medianDegree << endl;
        output.close();
        }

        

    // Função para calcular a distância entre dois vértices usando BFS
    int bfsDistance(int start, int end) {
        vector<int> dist(V + 1, -1);  // Distâncias de cada vértice ao vértice inicial
        queue<int> q;

        dist[start] = 0;
        q.push(start);

        while (!q.empty()) {
            int v = q.front();
            q.pop();

            for (int u : adjList[v]) {
                if (dist[u] == -1) {  // Se o vértice ainda não foi visitado
                    dist[u] = dist[v] + 1;  // Distância do vértice inicial até u
                    q.push(u);
                    if (u == end) {  // Se já chegamos ao destino, podemos retornar
                        return dist[u];
                    }
                }
            }
        }

        return -1;  // Caso os vértices não estejam conectados
    }

    void calculateDiameter() {
        int diameter = 0;

        for (int i = 1; i <= V; i++) {
            for (int j = i + 1; j <= V; j++) {
                int d = bfsDistance(i, j);  // Calcula a distância entre os vértices i e j
                if (d > diameter) {
                    diameter = d;  // Atualiza o diâmetro se encontrarmos uma distância maior
                }
            }
        }

        // Escrever o diâmetro em um arquivo
        ofstream output("diameter.txt");
        if (!output) {
            cerr << "Erro ao abrir o arquivo para escrever o diâmetro!" << endl;
            return;
        }
        output << "Diâmetro do grafo: " << diameter << endl;
        output.close();
    }


    // Função para descobrir componentes conexas
    void connectedComponents() {
        vector<bool> visited(V + 1, false);
        vector<vector<int>> components;
        for (int i = 1; i <= V; ++i) {
            if (!visited[i]) {
                vector<int> component;
                bfsComponent(i, visited, component);
                components.push_back(component);
            }
        }
        // Ordena componentes por tamanho
        sort(components.begin(), components.end(), [](const vector<int> &a, const vector<int> &b) {
            return a.size() > b.size();
        });

        // Abrir o arquivo para escrita
        ofstream outputFile("componentes_conexas.txt");
        
        if (!outputFile) {
            cerr << "Erro ao abrir o arquivo!" << endl;
            return;
        }

        // Escrever as informações sobre as componentes no arquivo
        outputFile << "Número de componentes conexas: " << components.size() << endl;
        for (size_t i = 0; i < components.size(); i++) {
            outputFile << "Componente " << i + 1 << ": tamanho = " << components[i].size() << endl;
            outputFile << "Vértices: ";
            for (int v : components[i]) {
                outputFile << v << " ";
            }
            outputFile << endl;
        }

        // Fechar o arquivo
        outputFile.close();
    }

    // Função para BFS em componentes
    void bfsComponent(int start, vector<bool> &visited, vector<int> &component) {
        queue<int> q;
        q.push(start);
        visited[start] = true;
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            component.push_back(u);
            for (int v : (useAdjMatrix ? getNeighborsMatrix(u) : adjList[u])) {
                if (!visited[v]) {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }
    }

private:
    // Função auxiliar para obter vizinhos na matriz de adjacência
    vector<int> getNeighborsMatrix(int u) {
        vector<int> neighbors;
        for (int v = 1; v <= V; ++v) {
            if (adjMatrix[u][v]) {
                neighbors.push_back(v);
            }
        }
        return neighbors;
    }
};

// Função para salvar estatísticas do grafo em um arquivo
void saveGraphStats(const Graph &g, const string &filename) {
    ofstream outfile(filename);
    int minDegree, maxDegree;
    double avgDegree, medianDegree;
    g.calculateDegreeStats(minDegree, maxDegree, avgDegree, medianDegree);
    outfile << "Número de vértices: " << g.getV() << endl;
    outfile << "Número de arestas: " << g.getEdgeCount() << endl;
    outfile << "Grau mínimo: " << minDegree << endl;
    outfile << "Grau máximo: " << maxDegree << endl;
    outfile << "Grau médio: " << avgDegree << endl;
    outfile << "Mediana de grau: " << medianDegree << endl;
    outfile.close();
}

int main() {
    Graph g(0, false); // Inicializa com 0 vértices, pois serão lidos do arquivo, não utilizando matriz adjacencia
    g.loadFromFile("grafo.txt"); //lendo o arquivo

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

