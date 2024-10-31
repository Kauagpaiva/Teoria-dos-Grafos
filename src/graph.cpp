#include <iostream>
#include <vector>
#include <fstream>
#include <queue>
#include <list>
#include <algorithm>
#include <limits>
#include <cmath>
#include <numeric>
#include <cstdlib>   // For srand() and rand()
#include <ctime>     // For time()
#include <chrono>
#include <stack>
#include <map>

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
    }

    int getV() const { return V; } // Método para obter número de vértices
    int getEdgeCount() const { return numEdges; } // Método para obter número de arestas

    // Método para adicionar uma aresta
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

        if (!file.is_open()) { // Tratamento de erro
            cerr << "Erro ao abrir o arquivo " << filename << "!" << endl;
            return;
        }
        
        file >> V; // Lê o número de vértices do grafo do arquivo.
        adjList.resize(V + 1); // Ajusta o tamanho da lista adjacencia para comportar todos os vértices do grafo
        if (useAdjMatrix){ // Caso o usuário opte por usar a matriz adjacencia, também ajusta o tamanho da matriz
            adjMatrix.resize(V + 1, vector<int>(V + 1, 0));
        }
        
        int u, v; 
        while (file >> u >> v) { //lê pares de inteiros (as arestas) do arquivo até o final do arquivo.
            addEdge(u, v); // Adiciona as arestas ao grafo
        }

        file.close(); //Fecha o arquivo após a leitura estar completa.
    }

    // BFS para percorrer o grafo, nó inicial como parâmetro
    void bfs(int start, vector<bool> &visited, vector<int> &parent, vector<int> &level) {

        queue<int> q; // Fila
        visited[start] = true; //marca o nó inicial como visitado
        level[start] = 0; //marca o nivel do nó inicial como 0
        q.push(start); //adiciona o nó inicial à fila

        
        while (!q.empty()) { //enquanto a fila não estiver vazia
            int u = q.front(); // pega o primeiro nó da fila
            q.pop(); //remove ele da fila

            for (int v : (useAdjMatrix ? getNeighborsMatrix(u) : adjList[u])) { //para cada nó vizinho de u
                if (!visited[v]) { //se v não foi visitado ainda
                    visited[v] = true; //visita v
                    parent[v] = u; //define o pai de v como u
                    level[v] = level[u] + 1; //define o nivel de v
                    q.push(v); //remove v da fila para ser visitado
                }
            }
        }
    }

    // DFS para percorrer o grafo, nó inicial como parâmetro
    void dfs(int start,  vector<bool> &visited, vector<int> &parent, vector<int> &level) {
        
        stack<int> dfsStack; // Pilha para realizar a DFS iterativa
        dfsStack.push(start);
        level[start] = 0; //Define o nível do nó inicial como 0

        while (!dfsStack.empty()) { //Enquanto a fila não estiver vazia
            int u = dfsStack.top(); //Pega um nó da pilha
            dfsStack.pop(); //remove esse nó da pilha

            if (!visited[u]) { //verifica se esse nó ainda não foi visitado
                visited[u] = true; //marca o nó como visitado

                // Adiciona todos os vizinhos não visitados na pilha
                for (int v : (useAdjMatrix ? getNeighborsMatrix(u) : adjList[u])) { //percorre os vizinhos desse nó
                    if (!visited[v]) { //verifica se esse vizinho ainda não foi visitado
                        parent[v] = u; //define u como pai desse vizinho
                        level[v] = level[u] + 1; //define o nível do vizinho como um nível maior que o de u
                        dfsStack.push(v); //adiciona o vizinho à pilha
                    }
                }
            }
        }
    }

    // Função para calcular o grau mínimo, máximo, médio e mediana
    void calculateDegreeStats(int &minDegree, int &maxDegree, double &avgDegree, double &medianDegree) {
        vector<int> degrees(V + 1, 0); //Lista para registrar os graus
        for (int i = 1; i <= V; ++i) {
            degrees[i] = (useAdjMatrix ? getNeighborsMatrix(i) : adjList[i]).size(); //Define o grau de cada nó
        }

        // Ordena os graus do menor para o maior
        sort(degrees.begin() + 1, degrees.end());

        int minDegree = degrees[1]; //grau minimo
        int maxDegree = degrees[V]; //grau maximo
        double avgDegree = accumulate(degrees.begin() + 1, degrees.end(), 0.0) / V; //grau medio
        double medianDegree = (V % 2 == 0) ? (degrees[V / 2] + degrees[V / 2 + 1]) / 2.0 : degrees[V / 2 + 1]; //mediana de grau
        }

    // Função para calcular a distância entre dois vértices usando BFS
    int bfsDistance(int start, int end) {
        vector<int> dist(V + 1, -1);  // Distâncias de cada vértice ao vértice inicial
        queue<int> q; //fila

        dist[start] = 0; //definindo a distancia do nó inicial como 0
        q.push(start); //adiciona o nó inicial à fila

        while (!q.empty()) { //enquanto a fila não estiver vazia:
            int v = q.front(); // pega um nó da fila
            q.pop(); // remove esse nó da fila

            for (int u : adjList[v]) { //percorre os vizinhos desse nó
                if (dist[u] == -1) {  // Se o vizinho ainda não foi visitado
                    dist[u] = dist[v] + 1;  // Distância do nó inicial até o vizinho selecionado
                    q.push(u); // adiciona o vizinho à fila
                    if (u == end) {  // Se já chegamos ao destino, podemos parar
                        return dist[u];
                    }
                }
            }
        }

        // Tentativa de liberar memória
        vector<int>().swap(dist);
        return -1;  // Caso os vértices não estejam conectados
    }

    // Método para encontrar o diâmetro da maior componente conexa
    int getGraphDiameter() {
        vector<bool> visited(V + 1, false); //define um vetor para registrar os nós que já foram visitadps
        int maxDiameter = 0; // variavel para registrar o diametro

        for (int i = 1; i <= V; ++i) { //percorre os nós do grafo
            if (!visited[i]) { //verifica se o nó já foi visitado
                int diameter = getComponentDiameter(i, visited); //pega o diametro do grafo a partir desse nó
                maxDiameter = max(maxDiameter, diameter); // salva o maior diametro encontrado até o momento
            }
        }
        return maxDiameter; //retorna o maior diametro
    }

    // Função para descobrir componentes conexas
    vector<vector<int>> connectedComponents() {
        vector<bool> visited(V + 1, false); //vetor para registrar os nós que já foram visitados
        vector<vector<int>> components; //vetor de vetores para registrar os componentes
        for (int i = 1; i <= V; ++i) { // percorre os nós do grafo
            if (!visited[i]) { //verifica se o nó já foi visitado
                vector<int> component; //define um componente sem nada
                bfsComponent(i, visited, component); //aplica bfs nesse nó para descobrir todos os nós que são alcançáveis a partir dele
                components.push_back(component); //adiciona o componente à lista de componentes
            }
        }
        // Ordena componentes por tamanho
        sort(components.begin(), components.end(), [](const vector<int> &a, const vector<int> &b) {
            return a.size() > b.size();
        });
        return components; //retorna as componentes conexas
    }

    // Função auxiliar para aplicar BFS para descobrir componentes conexas
    void bfsComponent(int start, vector<bool> &visited, vector<int> &component) {
        vector<int> parent(V + 1, -1); //define uma lista para registrar os pais pois é requisito para rodar a bfs
        vector<int> level(V + 1, -1); //define uma lista para registrar o nível pois é requisito para a bfs

        bfs(start, visited, parent, level); //roda uma bfs para pegar os nós que são visitados a partir do nó inicial

        for (int i = 1; i <= V; ++i) { //percorre a lista de nós que foram visitados pela bfs
            if(visited[i]){
                component.push_back(i); //adiciona os nós visitados ao componente
            }
        }
    }

    // Função para salvar estatísticas do grafo em um arquivo
    void saveGraphStats(const string &filename) {
        ofstream outfile(filename); //define um local de saida pro arquivo
        int minDegree, maxDegree; //define as variaveis inteiras
        double avgDegree, medianDegree; // define as variaveis double
        calculateDegreeStats(minDegree, maxDegree, avgDegree, medianDegree); //calcula os dados do grafo fornecido

        //registra as informações no arquivo de saida
        outfile << "Número de vértices: " << V << endl;
        outfile << "Número de arestas: " << numEdges << endl;
        outfile << "Grau mínimo: " << minDegree << endl;
        outfile << "Grau máximo: " << maxDegree << endl;
        outfile << "Grau médio: " << avgDegree << endl;
        outfile << "Mediana de grau: " << medianDegree << endl;
        outfile.close(); //fecha o arquivo
    }

private:
    // Função auxiliar para obter vizinhos na matriz de adjacência
    vector<int> getNeighborsMatrix(int u) {
        vector<int> neighbors; //define um vetor para registrar os vizinhos
        for (int v = 1; v <= V; ++v) { 
            if (adjMatrix[u][v]) { //verifica na matriz se são vizinhos
                neighbors.push_back(v); //adiciona ao vetor de vizinhos
            }
        }
        return neighbors; //retorna o vetor
    }

    // Encontrar o diâmetro da componente conexa a partir de um nó inicial
    int getComponentDiameter(int start, vector<bool> &visited) {
        vector<int> parent(V + 1, -1); //lista dos pais
        vector<int> level(V + 1, -1); //lista dos niveis

        // BFS para encontrar o vértice mais distante
        bfs(start, visited, parent, level);

        // Obtem o o nó que está no maior nível da árvore (maior distancia do nó inicial)
        auto it = std::max_element(level.begin(), level.end());
        int farthestFromStart = std::distance(level.begin(), it);

        vector<bool> visited2(V + 1, false);
        vector<int> parent2(V + 1, -1); //lista dos pais
        vector<int> level2(V + 1, -1); //lista dos niveis
        // Fazer BFS a partir do vértice mais distante
        bfs(farthestFromStart, visited2, parent2, level2);

        // Obtem o o maior nível da lista de niveis da árvore
        int distance = *std::max_element(level2.begin(), level2.end());

        return distance;  // Retorna o maior nível encontrado pela segunda BFS
    }
};


class WeightedGraph {
private:
    int numVertices;
    vector<list<pair<int, double>>> adjList;  // Lista de adjacências com pesos (vértice, peso)

public:
    // Construtor
    WeightedGraph(int vertices) : numVertices(vertices), adjList(vertices) {}

    // Método para adicionar uma aresta com peso
    void addEdge(int v1, int v2, double weight) {
        adjList[v1].emplace_back(v2, weight);
        adjList[v2].emplace_back(v1, weight);  // Grafo não direcionado
    }

    // Leitura do grafo a partir de um arquivo
    void readGraphFromFile(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Erro ao abrir o arquivo!" << endl;
            return;
        }

        int vertices;
        file >> vertices;
        numVertices = vertices;
        adjList.resize(numVertices);

        int v1, v2;
        double weight;
        while (file >> v1 >> v2 >> weight) {
            addEdge(v1, v2, weight);
        }

        file.close();
    }

    // fuunção auxiliar para encontrar o nó com a menor distancia
    int getNodeWithMinDist(const vector<double>& dist, const vector<bool>& visited) {
        float minDistance = numeric_limits<float>::infinity();
        int nodeWithMinDistance = -1;

        for (int i = 0; i < numVertices; i++) {
            if (!visited[i] && dist[i] <= minDistance) {
                // cout << "Entrou" << endl;
                minDistance = dist[i];
                nodeWithMinDistance = i;
            }
        }
        return nodeWithMinDistance;
    }

    // Algoritmo de Dijkstra sem heap
    pair<vector<double>, vector<int>> dijkstra(int start) {
        vector<double> dist(numVertices, numeric_limits<double>::infinity()); //iniciando todos os vertices com distancia infinita
        vector<int> parent(numVertices, -1); // lista para registrar o pai de cada vertice
        vector<bool> visited(numVertices, false);  // Conjunto de vértices explorados

        dist[start] = 0.0;  // Distância para o vértice inicial é zero
        parent[start] = 0; // Vertice inicial sem pai pois é a raiz

        for (int i = 0; i < numVertices; i++) {
            // Seleciona u em V-S tal que dist[u] é mínima
            int u = getNodeWithMinDist(dist, visited);

            visited[u] = true; //visita u

            // Para cada vizinho v de u
            for (const auto& neighbor : adjList[u]) {
                int v = neighbor.first; // define o vizinho
                double weight = neighbor.second; // define o peso da aresta ate esse vizinho

                // Se dist[v] > dist[u] + w(u,v), atualiza dist[v]
                if (dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                }
            }
        }

        // Retorna distâncias e pais
        return {dist, parent};
    }

    // Algoritmo de Dijkstra usando heap (priority queue)
    pair<vector<double>, vector<int>> dijkstraWithHeap(int start) {
        vector<double> dist(numVertices, numeric_limits<double>::infinity()); // Inicializa distâncias
        vector<int> parent(numVertices, -1); // Inicializa pais
        dist[start] = 0.0;  // A distância do vértice inicial é zero

        // Min-heap para armazenar (distância, vértice)
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> minHeap;
        minHeap.push({0.0, start});  // Começa com o vértice inicial

        while (!minHeap.empty()) {
            int u = minHeap.top().second;  // Vértice com menor distância
            double currDist = minHeap.top().first;
            minHeap.pop();

            // Se a distância atual é maior que a registrada, ignore
            if (currDist > dist[u]) continue;

            // Para cada vizinho v de u
            for (const auto& neighbor : adjList[u]) {
                int v = neighbor.first;
                double weight = neighbor.second;

                // Relaxamento da aresta (u, v)
                if (dist[v] > dist[u] + weight) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    minHeap.push({dist[v], v});
                }
            }
        }

        // Retorna as distâncias e os pais
        return {dist, parent};
    }
};

 int main() {
     return 0;
 }
