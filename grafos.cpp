#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <map>
#include <algorithm>
#include <sstream>
#include <limits>
#include <climits>

using namespace std;

int order; // para o algoritmo de tarjan
int time_s; // para o algoritomo de articulações

struct Edge {
    int vertex;
    int weight;
    int id;

    Edge(int vertex, int weight, int id = -1) {
        this->vertex = vertex;
        this->weight = weight;
        this->id = id;
    }

    bool operator=(const Edge& other) const {
        return vertex == other.vertex;
    }

    bool operator<(const Edge& other) const {
        return weight < other.weight; // necessário para o algoritmo de Kruskal
    }
    bool operator>(const Edge& other) const {
        return this->weight > other.weight; //necessario para o algortimo de dijkistra
    }
};


// dfs e bfs
void DFSFecho(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& reached);
void DFS(int vertex, vector<Edge> *adjList, int numVertex, bool* visited);
void DFSTarjan(int vertex, vector<Edge> *adjList, int numVertex, int *pre, int *lo, stack<int>& stack, bool *stackMember);
void BFSTree(int initialVertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
void DFSTree(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
bool DFSCicloGrafoDirecionado(int vertex, vector<Edge> *adjList,  int numVertex,vector<string>& state);
bool DFSCiclo(int vertex, vector<Edge> *adjList,  int numVertex, bool *visited) ;

// funções essenciais
bool eulerian(vector<Edge> *adjList, int numVertex, string direction);
map<int, vector<int>> tarjanStronglyComponents(vector<Edge> *adjList, int numVertex);
void printDephTree(vector<Edge> *adjList, int numVertex);
void printBreadthTree(vector<Edge> *adjList, int numVertex);
vector<int> kahnTopologicalSort(vector<Edge> *adjList, int numVertex);
void fechoTransitivo(vector<Edge> *adjList, int numVertex);

// funções auxiliares
int getVertexIncomingDegree(int vertex, vector<Edge> *adjList, int numVertex);
bool contemCiclo(vector<Edge> *adjList, int numVertex, string direction);
vector<Edge>* transpose(vector<Edge> *adjList, int numVertex);
bool stronglyConnected(vector<Edge> *adjList, int numVertex);
void printAdjList(vector<Edge> *adjList, int numVertex);

// funcoes do paulo silveira
bool connected(vector<Edge> *adjList, int numVertex, string direction);
vector<int> getArticulacoes(vector<Edge> *adjList, int numVertex);
void DFSArticulacoes(int u, vector<Edge>* adjList, vector<int>& visitedOrder, vector<int>& low, vector<int>& parent, vector<int>& ans);
void dijkstra(int org, vector<Edge>* adjList, int numVertex);

// funções do gabriel
bool checkBipartite(vector<Edge> *adjList, int numVertex);
int countConnectedComponents(int numVertex, vector<Edge> *adjList);
void DFSConnectedComponents(int vertex, vector<Edge> *adjList, vector<bool>& visited);
void DFSBridges(int u, int parent, vector<Edge> *adjList, int numVertex, vector<int>& visitedOrder, vector<int>& low, vector<Edge>& bridges);
vector<Edge> getBridges(vector<Edge> *adjList, int numVertex);

// 2 5 8 11 14
int main() {
    int command, numVertex, numEdges, edgeId, vertexU, vertexV, weight;
    vector<Edge> *adjList = nullptr;
    vector<int> commands;
    string line, direction;

    getline(cin, line); // ler a lista de comandos passada na entrada padrao   

    stringstream buffer(line);
    while (buffer >> command)
        commands.push_back(command);

    cin >> numVertex >> numEdges >> direction; // ler o numero de vertices, numero de arestas e se o grafo e direcionado
    adjList = new vector<Edge>[numVertex]; 

    for (int i = 0; i < numEdges; i++) {
        // preenche a lista de adjacência
        cin >> edgeId >> vertexU >> vertexV >> weight;

        adjList[vertexU].push_back(Edge(vertexV, weight, edgeId));

        if (direction.compare("nao_direcionado") == 0) 
            adjList[vertexV].push_back(Edge(vertexU, weight, edgeId));
    }

    for (int i = 0; i < commands.size(); i++) {
        switch(commands[i]) {
            case 0: {// paulo silveira
                cout << connected(adjList, numVertex, direction) << endl;
                break;
            }
            case 1: {// gabriel 
                cout << checkBipartite(adjList, numVertex) << endl;
                break;
            }
            case 2: {// paulo alves
                cout << eulerian(adjList, numVertex, direction) << endl;
                break;
            }
            case 3: {// paulo silveira
                cout << contemCiclo(adjList, numVertex, direction) << endl;
                break;
            }
            case 4: {
                // funcao 5 - gabriel 
                //cout << "5 - connected components:" << countConnectedComponents(numVertex, adjList) << endl;
                break;
            }
            case 5: {
                // funcao 6 - paulo alves - strongly components
                //map<int, vector<int>> stronglyComponents = tarjanStronglyComponents(adjList, numVertex);
                //cout << "6 - amount of strongly components: " << stronglyComponents.size() << endl;
                break;
            }
            case 6: {
                // funcao 7 - paulo silveira
                cout << "6 - articulacoes: ";
                if (direction.compare("nao_direcionado") == 0) {
                    vector<int> articulacoes = getArticulacoes(adjList, numVertex);

                    if(articulacoes.empty()) {
                        cout << "0" << endl;
                    }
                    else {
                        for (int i = 0; i < articulacoes.size(); i++) 
                            cout << articulacoes[i] << " ";
                        cout << endl;
                    }
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 7: {
                // funcao 8 - gabriel
                //vector<Edge> bridges = getBridges(adjList, numVertex);
                //cout << "7 - num bridges: " << bridges.size() << endl;
                break;
            }
            case 8: {
                // deep search tree
                cout << "9 - arvore de busca em profundidade ";
                //printDephTree(adjList, numVertex);
                break;
            }
            case 9: {// paulo silveira
                printBreadthTree(adjList, numVertex);
                break;
            }
            case 10: {
                cout << "-1" << endl;
                break;
            }
            case 11: {
                // funcao 12 - paulo alves - ordenação topologica
                cout << "12 - topologial sort: ";
                if (direction.compare("direcionado") == 0) {
                    // se o grafo for direciondo e nao possuir ciclos
                    vector<int> top = kahnTopologicalSort(adjList, numVertex);

                    for (int i = 0; i < top.size(); i++) {
                        cout << top[i] << " ";
                    }
                    cout << endl;
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 12: {
                // funcao 13 - paulo silveira
                cout << "12 - caminho mínimo: ";
                if (direction.compare("nao_direcionado") == 0) {
                    dijkstra(0, adjList, numVertex);
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 13: {
                cout << "-1" << endl;
                break;
            }
            case 14: {// paulo alves
                if (direction.compare("direcionado") == 0) {
                    fechoTransitivo(adjList, numVertex);
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
        }
    }

    return 0;
}

//
//
//
//
// PARTE PAULO HENRIQUE
//
//
//

bool connected(vector<Edge> *adjList, int numVertex, string direction) {
    bool *visited = new bool[numVertex];

    if (direction.compare("direcionado") == 0) {
        // converter o grafo para nao direcionado e chamar a DFS nele
        vector<Edge> *newAdjList = new vector<Edge>[numVertex]; // armazenar a nova versao nao direcionada
    
        for (int i = 0; i < numVertex; i++) 
            newAdjList[i] = adjList[i];
        
        for (int i = 0; i < numVertex; i++) {
            for (int j = 0; j < newAdjList[i].size(); j++) {
                Edge edge = newAdjList[i][j];
                newAdjList[edge.vertex].push_back(Edge(i, edge.weight, edge.id));
            }
        }

        // chamar a dfs para o novo grafo direcionado convertido para nao direcionado
        DFS(0, newAdjList, numVertex, visited);
    } 

    // chamar a dfs para o grafo nao direcionado
    DFS(0, adjList, numVertex, visited);
    
    for (int i = 0; i < numVertex; i++) 
        if (!visited[i])
            return false;

    return true;
}


// verificar se o grafo é euleriano
bool eulerian(vector<Edge> *adjList, int numVertex, string direction) {
    int numOddDegreeVertex = 0;

    if (direction.compare("nao_direcionado") == 0) {
        // caso o grafo seja não direcionado
        for (int i = 0; i < numVertex; i++) {
        int vertex = i;
        int count = 0;

        for (int j = 0; j < numVertex; j++) {
            for (int k = 0; k < adjList[j].size(); k++) {
                if (adjList[j][k].vertex == vertex)
                    count++;
            }
        }

        if (count % 2 != 0)
            numOddDegreeVertex++;
        }

        // se o numero de vértices com grau ímpar for igual a 0
        // o grafo é eleriano
        return numOddDegreeVertex == 0;
    } 
    
    // caso o grafo seja direcionado
    map<int, vector<int>> stronglyComponents = tarjanStronglyComponents(adjList, numVertex);

    if (!stronglyConnected(adjList, numVertex))
        // se o grafo nao for fortemente conectado, ele nao é euleriano
        return false;

    // se o grafo for fortemente conectado, verifica se em cada vértice
    // os graus de entrada são iguais aos graus de saída
    for (int i = 0; i < numVertex; i++) {
        int vertex = i;
        int inDegree = 0;
        int outDegree = 0;

        for (int j = 0; j < numVertex; j++) {
            for (int k = 0; k < adjList[j].size(); k++) {
                if (adjList[j][k].vertex == vertex)
                    inDegree++;
            }
        }

        outDegree = adjList[vertex].size();

        if (inDegree != outDegree)
            return false;
    }


    // se o grafo direcionado for euleriano
    return true;
}

// algoritmo de tarjan para encontrar as componentes fortementes conectadas em grafos direcionados
map<int, vector<int>> tarjanStronglyComponents(vector<Edge> *adjList, int numVertex) {
    int *pre = new int[numVertex];
    int *lo = new int[numVertex];
    bool *stackMember = new bool[numVertex];
    stack<int> stack;
    map<int, vector<int>> stronglyComponents;
    order = 0;

    for (int i = 0; i < numVertex; i++) {
        pre[i] = -1;
        lo[i] = -1;
        stackMember[i] = false;
    }
        
    for (int i = 0; i < numVertex; i++) {
        if (pre[i] == -1)
            DFSTarjan(i, adjList, numVertex, pre, lo, stack, stackMember);
    }

    for (int i = 0; i < numVertex; i++) {
        stronglyComponents[lo[i]].push_back(i);
    }

    return stronglyComponents;
}

// dfs para o algoritmo de tarjan
void DFSTarjan(int vertex, vector<Edge> *adjList, int numVertex, int *pre, int *lo, stack<int>& sc, bool *stackMember) {
    pre[vertex] = order++;
    lo[vertex] = pre[vertex];
    sc.push(vertex);
    stackMember[vertex] = true;

    sort(adjList[vertex].begin(), adjList[vertex].end());

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (pre[adjVertex] == -1) {
            // se encontrou um vértice adjacente ainda não visitado
            DFSTarjan(adjVertex, adjList, numVertex, pre, lo, sc, stackMember); 
            lo[vertex] = min(lo[vertex], lo[adjVertex]);
        } else if (stackMember[adjVertex] == true) { 
            // se encontrou um vértice adjacente que já foi visitado
            lo[vertex] = min(lo[vertex], pre[adjVertex]); 
        } 
    }
    
    int v = 0;

    if (lo[vertex] == pre[vertex]) {
        // cout << "finded new head " << vertex << endl;
        // encontrou uma "head"

        while (sc.top() != vertex) {
            v = sc.top();
            // cout << v << " ";
            stackMember[v] = false;
            sc.pop();
        }

        v = sc.top();
        // cout << v << "\n";
        stackMember[v] = false;
        sc.pop();
    }
}

// verifica se o grafo é fortemente conectado
bool stronglyConnected(vector<Edge> *adjList, int numVertex) {
    int initialVertex = 0;
    bool* visited = new bool[numVertex];
    
    for (int i = 0; i < numVertex; i++) 
        visited[i] = false;

    DFS(initialVertex, adjList, numVertex, visited);

    for (int i = 0; i < numVertex; i++) {
        if (!visited[i]) 
            return false;

        visited[i] = false;
    }

    vector<Edge> *trans = transpose(adjList, numVertex);

    DFS(initialVertex, trans, numVertex, visited);

    for (int i = 0; i < numVertex; i++) 
        if (!visited[i]) 
            return false;

    return true;
}

void DFSFecho(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& reached) {
    visited[vertex] = true;
    reached.push_back(vertex);

    sort(adjList[vertex].begin(), adjList[vertex].end());

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (!visited[adjVertex]) 
            DFSFecho(adjVertex, adjList, numVertex, visited, reached);
    }
}

/// dfs 
void DFS(int vertex, vector<Edge> *adjList, int numVertex, bool* visited) {
    visited[vertex] = true;

    sort(adjList[vertex].begin(), adjList[vertex].end());

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (!visited[adjVertex]) 
            DFS(adjVertex, adjList, numVertex, visited);
    }
}

void printDephTree(vector<Edge> *adjList, int numVertex) {
    int initialVertex = 0;
    bool* visited = new bool[numVertex];
    vector<int> order;
    
    for (int i = 0; i < numVertex; i++) 
        visited[i] = false;

    DFSTree(initialVertex, adjList, numVertex, visited, order);

    for (int i = 0; i < order.size(); i++)
        cout << order[i] << " ";
    cout << endl;
}

void printBreadthTree(vector<Edge> *adjList, int numVertex) {
    int initialVertex = 0;
    bool* visited = new bool[numVertex];
    vector<int> order;
    
    for (int i = 0; i < numVertex; i++) 
        visited[i] = false;

    BFSTree(initialVertex, adjList, numVertex, visited, order);

    for (int i = 0; i < order.size(); i++)
        cout << order[i] << " ";
    cout << endl;
}

void BFSTree(int initialVertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order) {
    queue<int> q;

    visited[initialVertex] = true;
    q.push(initialVertex);

    while (!q.empty()) {
        int vertex = q.front();
        q.pop();

        sort(adjList[vertex].begin(), adjList[vertex].end());

        for (int i = 0; i < adjList[vertex].size(); i++) {
            int adjVertex = adjList[vertex][i].vertex;

            if (!visited[adjVertex]) {
                order.push_back(adjList[vertex][i].id);
                q.push(adjVertex);
                visited[adjVertex] = true;
            }
        }
    }
}

void DFSTree(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order) {
    visited[vertex] = true;

    sort(adjList[vertex].begin(), adjList[vertex].end());

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (!visited[adjVertex]) {
            order.push_back(adjList[vertex][i].id);
            DFSTree(adjVertex, adjList, numVertex, visited, order);
        }
    }
}

vector<Edge>* transpose(vector<Edge> *adjList, int numVertex) {
    vector<Edge> *trans = new vector<Edge>[numVertex];

    for (int i = 0; i < numVertex; i++) 
        for (int j = 0; j < adjList[i].size(); j++) 
            trans[j].push_back(Edge(i, adjList[i][j].weight));

    return trans;
}

// verificar se o grafo contem ciclo
bool contemCiclo(vector<Edge> *adjList, int numVertex, string direction) {
    if (direction.compare("nao_direcionado") == 0) {
        // caso o grafo seja nao direcionado
        bool *visited = new bool[numVertex];
    
        for (int i = 0; i < numVertex; i++)
            visited[i] = false;

        for (int i = 0; i < numVertex; ++i) {
            if (!visited[i] && DFSCiclo(i, adjList, numVertex, visited)) {
                return true;
            }    
        }

        return false;
    } 

    // caso o grafo seja direcionado
    vector<string> state(numVertex, "WHITE");

    for (int i = 0; i < numVertex; i++) {
        if (state[i].compare("WHITE") == 0 && DFSCicloGrafoDirecionado(i, adjList, numVertex, state)) {
            return true;
        }
    }

    return false;
}

// dfs para encontrar ciclos
// grafos nao direcionados
bool DFSCiclo(int vertex, vector<Edge> *adjList,  int numVertex, bool *visited) {
    // DFS(v, pai):    
    // visitado[v] = true
    
    visited[vertex] = true;

    sort(adjList[vertex].begin(), adjList[vertex].end());

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;
        
        if (!visited[adjVertex] && DFSCiclo(adjVertex, adjList, numVertex, visited)) {
            return true;
        } else if (adjVertex != vertex) {
            return true; // Se adjVertex foi visitado e não é o pai, há um ciclo
        }
    }

    return false; 
}

// dfs para encontrar ciclos 
// grafos direcionados
bool DFSCicloGrafoDirecionado(int vertex, vector<Edge> *adjList,  int numVertex, vector<string>& state) {
    state[vertex] = "GRAY";  // Marca o vértice como "em processo"

    sort(adjList[vertex].begin(), adjList[vertex].end());

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (state[adjVertex].compare("GRAY") == 0) 
            return true;
        

        if (state[adjVertex].compare("WHITE") == 0) 
            if (DFSCicloGrafoDirecionado(adjVertex, adjList, numVertex, state)) 
                return true;
    }

    state[vertex] = "BLACK"; 

    return false;
}

// algoritmo de kahn para encontrar ordem topológica
vector<int> kahnTopologicalSort(vector<Edge> *adjList, int numVertex) {
    vector<int> top; // armazenar a ordenação topológica
    vector<int> inDegree(numVertex, 0); // grau de entrada dos vértices
    vector<Edge> *newAdjList = new vector<Edge>[numVertex]; // armazenar a nova versao nao direcionada
    stack<int> zeroDegreeQueue;

    for (int i = 0; i < numVertex; i++) 
        newAdjList[i] = adjList[i];

    // calcula o grau de entrada de cada vértice
    for (int i = 0; i < numVertex; i++) {
        for (int j = 0; j < adjList[i].size(); j++) {
            int adjVertex = adjList[i][j].vertex;
            inDegree[adjVertex]++; 
        }
    }

    // calcula o grau de entrada de cada vértice
    for (int i = 0; i < numVertex; i++) {
        for (int j = 0; j < adjList[i].size(); j++) {
            int adjVertex = adjList[i][j].vertex;
            if (adjVertex == 0)
                cout << "\n\ntem alguem apontando para o zero\n\n";
        }
    }

    // adiciona todos os vértices com grau de entrada zero na fila
    for (int i = 0; i < numVertex; i++) {
        if (inDegree[i] == 0) {
            zeroDegreeQueue.push(i);
        }
    }

    // cout << "\n\nzero degree queue\n\n\n";
    // for (int i = 0; i  <zeroDegreeQueue.size(); i++) {
    //     cout << zeroDegreeQueue.front() << " ";
    //     zeroDegreeQueue.pop();
    // }

    while (!zeroDegreeQueue.empty()) {
        int u = zeroDegreeQueue.top();
        zeroDegreeQueue.pop();
        top.push_back(u);

        //sort(adjList[u].begin(), adjList[u].end());

        // reduz o grau de entrada dos vizinhos
        for (int i = 0; i < newAdjList[u].size(); i++) {
            int adjVertex = newAdjList[u][i].vertex;

            inDegree[adjVertex]--;

            if (inDegree[adjVertex] == 0) {
                zeroDegreeQueue.push(adjVertex);
            }
        }
    }

    // Verifica se há ciclos no grafo
    if (top.size() != numVertex) {
        // O grafo contém ciclos e a ordenação topológica não é possível
        top.clear();
    }

    return top;
}

int getVertexIncomingDegree(int vertex, vector<Edge> *adjList, int numVertex) {
    int count = 0; 

    for (int i = 0; i < numVertex; i++) 
        for (int j = 0; j < adjList[i].size(); j++) 
            if (adjList[i][j].vertex == vertex)
                count++;
        
    return count;
}

void fechoTransitivo(vector<Edge> *adjList, int numVertex) {
    bool *visited = new bool[numVertex];
    vector<int> fechoTransitivo;

    for (int j = 0; j < numVertex; j++)
        visited[j] = false;

    DFSFecho(0, adjList, numVertex, visited, fechoTransitivo);
 
    for (int j = 0; j < fechoTransitivo.size(); j++) {
        if (fechoTransitivo[j] != 0)
            cout << fechoTransitivo[j] << " ";
    }
}

void printAdjList(vector<Edge> *adjList, int numVertex) {
    for (int i = 0; i < numVertex; i++) {
        cout << "[" << i << "] -> ";
        for (int j = 0; j < adjList[i].size(); j++)
            cout << "(" << adjList[i][j].vertex << ", " << adjList[i][j].weight << ") ";
        cout << endl;
    }
}

//
//
//
//
// PARTE PAULO SILVEIRA
//
//
//

// retorna um vetor contendo todos vertices articulacoes
vector<int> getArticulacoes(vector<Edge> *adjList, int numVertex) {
    vector<int> visitedOrder(numVertex, -1), low(numVertex, -1), parent(numVertex, -1), ans;
    time_s = 0;

    for(int i =0; i < numVertex; i++) {
        if(visitedOrder[i] == -1) {
            DFSArticulacoes(i, adjList, visitedOrder, low, parent, ans);
        }
    }
    
    return ans;
}

// dfs para o algoritmo de encontrar articulacoes
void DFSArticulacoes(int u, vector<Edge>* adjList, vector<int>& visitedOrder, vector<int>& low, vector<int>& parent, vector<int>& ans) {  
    visitedOrder[u] = low[u] = time_s++;
    int filhos = 0;

    // u -> vértice atual sendo explorado pela DFS
    // menor -> mantém o menor valor de visitedOrder alcançado na subárvore DFS do vértice u
    // filhos -> conta o número de filhos diretos de u na árvore DFS
    // adjList[u] -> acessar os filhos (arestas) do vertice u
    // adjList[u][1] -> acessar o primeiro filho (aresta) do vertice u
    
    for (int i = 0; i < adjList[u].size(); i++) {
        int adjVertex = adjList[u][i].vertex;

         if (visitedOrder[adjVertex] == -1) {  // Vértice adjacente ainda não visitado
            filhos++;
            parent[adjVertex] = u;
             
            DFSArticulacoes(adjVertex, adjList, visitedOrder, low, parent, ans);
            
            low[u] = min(low[u], low[adjVertex]);

            // Verifica se u é um ponto de articulação
            if ((parent[u] == -1 && filhos > 1) || (parent[u] != -1 && low[adjVertex] >= visitedOrder[u])) {
                ans.push_back(u);  // u é um ponto de articulação
            }
        } else if (adjVertex != parent[u]) {  // Atualiza low[u] para back edge
            low[u] = min(low[u], visitedOrder[adjVertex]);
        }
    }  
}

void dijkstra(int org, vector<Edge>* adjList, int numVertex) {
    vector<int> distance(numVertex, INT_MAX);

    // A distância da origem "org" é sempre zero
    distance[org] = 0;

    // Heap que auxilia na obtenção do vértice com maior prioridade a cada iteração
    priority_queue<Edge, vector<Edge>, greater<Edge>> heap;

    // Primeiro par inserido na heap: "org" com custo zero
    heap.push(Edge(org, 0));

    vector<bool> visited(numVertex, false);

    // O algoritmo para quando a heap estiver vazia
    while (!heap.empty()) {
        Edge vertice = heap.top();
        heap.pop();

        int u = vertice.vertex;
        int distancia = vertice.weight;

        if (visited[u]) // "u" já foi explorado
            continue;

        visited[u] = true;

        for (int j = 0; j < adjList[u].size(); j++) {
            Edge vizinho = adjList[u][j];
            int v = vizinho.vertex;
            int custo = distancia + vizinho.weight;

            if (custo < distance[v]) {
                distance[v] = custo;
                heap.push(Edge(v, custo));
            }
        }
    }

    // Verifica se há um caminho para o último vértice
    if (distance[numVertex - 1] == INT_MAX) {
        cout << "-1" << endl;
    } else {
        cout << distance[numVertex - 1] << endl;
    }
}

// // função para verificar se a aresta u-v é uma ponte
// bool ehPonte(int u, int v) {
//     // 1: Verifica quantos vértices são acessíveis antes de remover a aresta (conexo)
//     vector<bool> visitado(V, false);
//     int idFunc = 7;
//     DFS(u, visitado, idFunc);

//     int contAntes = count(visitado.begin(), visitado.end(), true);

//     // 2: Remove a aresta u-v e verifica quantos vértices são acessíveis depois de remover a aresta
//     removeAresta(u, v);
//     fill(visitado.begin(), visitado.end(), false);
//     DFS(u, visitado, idFunc);
//     int contDepois = count(visitado.begin(), visitado.end(), true);

//     // 3: Recoloca a aresta u-v no grafo
//     adj[u].push_back({v, {0, 0}}); // id_aresta e custo são placeholders
//     if (!ehDirecionado) adj[v].push_back({u, {0, 0}});

//     // Se o número de vértices acessíveis diminuir, u-v é uma ponte
//     return (contAntes > contDepois);
// }

// // Encontra e imprime a trilha euleriana
// void trilhaEuleriana(int u) {
//     for (auto& v : adj[u]) {
//         int vAtual = v.first;

//         // Verifica se a aresta u-vAtual é válida para a trilha euleriana
//         if (adj[u].size() == 1 || !ehPonte(u, vAtual)) {
//             cout << u << " ";

//             removeAresta(u, vAtual);
//             trilhaEuleriana(vAtual);
//         }
//     }
// }
/*
bool verificaTrilhaEuleriana(int& inicio) {
    int startNodes = 0, endNodes = 0;

    for (int i = 0; i < V; ++i) {
        if (ehDirecionado) {
            if (grauSaida[i] - grauEntrada[i] == 1) {
                inicio = i;
                startNodes++;
            } else if (grauEntrada[i] - grauSaida[i] == 1) {
                endNodes++;
            } else if (grauEntrada[i] != grauSaida[i]) {
                return false;
            }
        } else {
            if (grauSaida[i] % 2 == 1) {
                startNodes++;
                inicio = i;
            }
        }
    }

    return ehDirecionado ? 
        (startNodes == 1 && endNodes == 1) || (startNodes == 0 && endNodes == 0) :
        (startNodes == 0 || startNodes == 2);
}
*/

//
//
//
//
// PARTE GABRIEL
//
//
//

//Função para verificar se o grafo é bipartido
bool checkBipartite(vector<Edge> *adjList, int numVertex) {
    vector<int> color(numVertex, -1);

    for (int i = 0; i < numVertex; i++){

        //Se o vértice não foi visitado
        if (color[i] == -1){
            queue<int> queue;
            queue.push(i);
            color[i] = 0;

            while (!queue.empty()){
                int u = queue.front();
                queue.pop();
                
                //Atribui a cor oposta ao vértice adjacente
                for (const auto &edge : adjList[u]){
                    int v = edge.vertex;
                    
                    if (color[v] == -1){
                        color[v] = 1 - color[u];
                        queue.push(v);
                    }
                    else if (color[v] == color[u])
                        return false; // Se dois vértices adjacentes têm a mesma cor, não é bipartido
                    }
                }
            }
    }
    return true;
}

// DFS (usadacountConnectedComponents
void DFSConnectedComponents(int vertex, vector<Edge> *adjList, vector<bool>& visited) {
    visited[vertex] = true;

    for (const Edge& edge : adjList[vertex]) {
        if (!visited[edge.vertex]) {
            DFSConnectedComponents(edge.vertex, adjList, visited);
        }
    }
}

// Função para contar componentes conexas
int countConnectedComponents(int numVertex, vector<Edge> *adjList) {
    vector<bool> visited(numVertex, false);
    int connectedComponents = 0;

    for (int i = 0; i < numVertex; i++) {
        if (!visited[i]) {
            DFSConnectedComponents(i, adjList, visited);
                connectedComponents++;
        }
    }
    return connectedComponents;
}


// DFS para encontrar pontes
void DFSBridges(int u, int parent, vector<Edge> *adjList, int numVertex, vector<int>& visitedOrder, vector<int>& low, vector<Edge>& bridges) {
    visitedOrder[u] = low[u] = time_s++;
    
    for (int i = 0; i < adjList[u].size(); i++) {
        int v = adjList[u][i].vertex;

        if (v == parent) continue;  // Ignora a aresta para o pai

        if (visitedOrder[v] == -1) {  // Se o vértice não foi visitado
            DFSBridges(v, u, adjList, numVertex, visitedOrder, low, bridges);
            low[u] = min(low[u], low[v]);

            // Se a condição de ponte for satisfeita, adiciona a ponte
            if (low[v] > visitedOrder[u]) {
                bridges.push_back(Edge(u, v));
            }
        } else {
            low[u] = min(low[u], visitedOrder[v]);
        }
    }
}

// retorna um vetor contendo todas as pontes (arestas)
vector<Edge> getBridges(vector<Edge> *adjList, int numVertex) {
    vector<int> visitedOrder(numVertex, -1), low(numVertex, -1);
    vector<Edge> bridges;
    int initialVertex = 0;
    time_s = 0;

    // Percorre todos os vértices, para garantir que todas as componentes sejam exploradas
    for (int i = 0; i < numVertex; i++) {
        if (visitedOrder[i] == -1) {
            DFSBridges(i, -1, adjList, numVertex, visitedOrder, low, bridges);
        }
    }

    return bridges;
}


// Função para encontrar o conjunto de um elemento (Union-Find)
int find(int parent[], int i) {
    if (parent[i] == i)
        return i;
    return find(parent, parent[i]);
}

// Função para unir dois conjuntos (Union-Find)
void Union(int parent[], int rank[], int x, int y) {
    int xroot = find(parent, x);
    int yroot = find(parent, y);

    if (xroot != yroot) {
        if (rank[xroot] < rank[yroot])
            parent[xroot] = yroot;
        else if (rank[xroot] > rank[yroot])
            parent[yroot] = xroot;
        else {
            parent[yroot] = xroot;
            rank[xroot]++;
        }
    }
}


// Algoritmo de Kruskal para encontrar a Árvore Geradora Mínima (MST)
vector<Edge> kruskalMST(vector<Edge> *adjList, int numVertex) {
    vector<Edge> result; // Armazena a MST
    vector<Edge> edges;
    int *parent = new int[numVertex];
    int *rank = new int[numVertex];

    // Inicializa o conjunto
    for (int i = 0; i < numVertex; i++) {
        parent[i] = i;
        rank[i] = 0;
    }

    // Coleta todas as arestas do grafo
    for (int i = 0; i < numVertex; i++) {
        for (const auto& edge : adjList[i]) {
            edges.push_back(edge);
        }
    }

    // Ordena todas as arestas por peso
    sort(edges.begin(), edges.end());

    // Processa todas as arestas em ordem crescente
    int i = 0; // SUSPEITO ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    for (const auto& edge : edges) {
        int x = find(parent, i); // Encontra o conjunto de u
        int y = find(parent, edge.vertex); // Encontra o conjunto de v

        // Se adicionar esta aresta não cria um ciclo
        if (x != y) {
            result.push_back(edge);
            Union(parent, rank, x, y); // Une os conjuntos
        }
        i++;
    }
    delete[]parent;
    delete[]rank;

    return result;
}

bool bfsMaxFlow(const vector<vector<int>>& capacity, const vector<vector<int>>& residual, int source, int sink, vector<int>& parent) {
    int numVertex = capacity.size();
    vector<bool> visited(numVertex, false);
    queue<int> q;

    q.push(source);
    visited[source] = true;
    parent[source] = -1; // Fonte não tem pai

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v = 0; v < numVertex; ++v) {
            if (!visited[v] && residual[u][v] > 0) { // Se o vértice não foi visitado e a capacidade residual > 0
                q.push(v);
                parent[v] = u;
                visited[v] = true;
                
                if (v == sink) {
                    return true; // Caminho encontrado
                }
            }
        }
    }

    return false; // Caminho não encontrado
}

// Função principal que calcula o fluxo máximo usando o algoritmo Edmonds-Karp
int edmondsKarp(const vector<vector<int>>& capacity, int source, int sink) {
    int numVertex = capacity.size();
    vector<vector<int>> residual(numVertex, vector<int>(numVertex, 0));
    
    // Inicializa a rede residual com as capacidades originais
    for (int u = 0; u < numVertex; ++u) {
        for (int v = 0; v < numVertex; ++v) {
            residual[u][v] = capacity[u][v];
        }
    }

    vector<int> parent(numVertex);
    int maxFlow = 0;

    // Enquanto existir um caminho de aumento
    while (bfsMaxFlow(capacity, residual, source, sink, parent)) {
        int pathFlow = numeric_limits<int>::max();

        // Encontrar a capacidade mínima do caminho encontrado
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            pathFlow = min(pathFlow, residual[u][v]);
        }

        // Atualizar capacidades residuais da rede
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residual[u][v] -= pathFlow;
            residual[v][u] += pathFlow;
        }

        maxFlow += pathFlow; // Adiciona o fluxo do caminho ao fluxo máximo total
    }

    return maxFlow;
}