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

struct Edge {
    int vertex;
    int weight;
    int id;

    Edge(int vertex, int weight, int id = -1) {
        this->vertex = vertex;
        this->weight = weight;
        this->id = id;
    }

    bool operator<(const Edge& other) const {
        return vertex < other.vertex; 
    }

    bool operator>(const Edge& other) const {
        return vertex > other.vertex;
    }
};

// dfs e bfs
void DFSFecho(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& reached);
void DFS(int vertex, vector<Edge> *adjList, int numVertex, bool* visited);
void DFSTarjan(int vertex, vector<Edge> *adjList, int numVertex, int *pre, int *lo, stack<int>& stack, bool *stackMember);
void BFSTree(int initialVertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
void DFSTree(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
bool DFSCicloGrafoDirecionado(int vertex, vector<Edge> *adjList,  int numVertex,vector<string>& state);
bool DFSCiclo(int vertex, vector<Edge> *adjList,  int numVertex, bool *visited, int parent);
void DFSConnectedComponents(int vertex, vector<Edge> *adjList, vector<bool>& visited);
void DFSBridges(int u, int parent, vector<Edge> *adjList, int numVertex, vector<int>& visitedOrder, vector<int>& low, vector<Edge>& bridges);
void DFSArticulacoes(int u, vector<Edge>* adjList, vector<int>& visitedOrder, vector<int>& low, vector<int>& parent, vector<int>& ans);

// funções essenciais
bool euleriano(vector<Edge> *adjList, int numVertex, string direction);
map<int, vector<int>> tarjanComponentesFortementeConectados(vector<Edge> *adjList, int numVertex);
void imprimirArvoreProfundidade(vector<Edge> *adjList, int numVertex);
void imprimirArvoreLargura(vector<Edge> *adjList, int numVertex);
vector<int> kahnOrdernacaoTopologica(vector<Edge> *adjList, int numVertex);
void fechoTransitivo(vector<Edge> *adjList, int numVertex);
int kruskal(int numVertex, vector<vector<int>>& capacity);
int edmondsKarp(const vector<vector<int>>& capacity, int source, int sink);
void getBridges(vector<Edge> *adjList, int numVertex);
bool checkBipartite(vector<Edge> *adjList, int numVertex);
int countConnectedComponents(int numVertex, vector<Edge> *adjList);
bool connected(vector<Edge> *adjList, int numVertex, string direction);
vector<int> getArticulacoes(vector<Edge> *adjList, int numVertex);
void dijkstra(int org, vector<Edge>* adjList, int numVertex);
bool contemCiclo(vector<Edge> *adjList, int numVertex, string direction);

// funções auxiliares
int numeroVerticesEntrada(int vertex, vector<Edge> *adjList, int numVertex);
vector<Edge>* transposta(vector<Edge> *adjList, int numVertex);
bool fortementeConectado(vector<Edge> *adjList, int numVertex);
void imprimirListaAdjacencia(vector<Edge> *adjList, int numVertex);
int find(int u, vector<int>& parent);
bool compareEdges(const Edge& a, const Edge& b);

int order; // para o algoritmo de tarjan
int time_s; // para o algoritomo de articulações

int main() {
    int command, numVertex, numEdges, edgeId, vertexU, vertexV, weight;
    vector<Edge> *adjList = nullptr;
    vector<int> commands;
    string line, direction;

    // ler a lista de comandos passada na entrada padrao   
    getline(cin, line); 

    stringstream buffer(line);
    while (buffer >> command)
        commands.push_back(command);

    // ler o numero de vertices, numero de arestas e se o grafo e direcionado
    cin >> numVertex >> numEdges >> direction; 
    adjList = new vector<Edge>[numVertex]; 

    // Vetor usado em fluxo máximo
    vector<vector<int>> capacity(numVertex, vector<int>(numVertex, 0)); 

    for (int i = 0; i < numEdges; i++) {
        // preenche a lista de adjacência
        cin >> edgeId >> vertexU >> vertexV >> weight;

        // usado em fluxo máximo e kruskal
        capacity[vertexU][vertexV] = weight; 

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
                cout << euleriano(adjList, numVertex, direction) << endl;
                break;
            }
            case 3: {// paulo silveira
                cout << contemCiclo(adjList, numVertex, direction) << endl;
                break;
            }
            case 4: {// gabriel
                if (direction.compare("nao_direcionado") == 0)
                    cout << countConnectedComponents(numVertex, adjList) << endl;
                else
                    cout << "-1" << endl;
                break;
            }
            case 5: {// paulo alves
                if (direction.compare("direcionado") == 0) {
                    map<int, vector<int>> stronglyComponents = tarjanComponentesFortementeConectados(adjList, numVertex);
                    cout << stronglyComponents.size() << endl;
                } else {
                    cout << "-1" << endl;
                }           
                break;
            }
            case 6: {// paulo silveira
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
                if (direction.compare("nao_direcionado") == 0)
                    getBridges(adjList, numVertex);
                else
                    cout << "-1" << endl;
                break;
            }
            case 8: {// paulo alves
                imprimirArvoreProfundidade(adjList, numVertex);
                break;
            }
            case 9: {// paulo silveira
                imprimirArvoreLargura(adjList, numVertex);
                break;
            }
            case 10: {
                // funcao 11 - gabriel
                if (direction.compare("nao_direcionado") == 0) {
                    int mstWeight = kruskal(numVertex, capacity);
                    cout << mstWeight << endl;
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 11: {// paulo alves
                if (direction.compare("direcionado") == 0 && !contemCiclo(adjList, numVertex, direction)) {
                    vector<int> top = kahnOrdernacaoTopologica(adjList, numVertex);
                    for (int i = 0; i < top.size(); i++) 
                        cout << top[i] << " ";
                    cout << endl;
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 12: {// paulo silveira
                dijkstra(0, adjList, numVertex);
                break;
            }
            case 13: {// gabriel
                if (direction.compare("nao_direcionado") == 0)
                    cout << "-1" << endl;
                else
                    cout << edmondsKarp(capacity, 0, numVertex - 1) << endl;
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


// verificar se o grafo é eulerianoo
bool euleriano(vector<Edge> *adjList, int numVertex, string direction) {
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
    map<int, vector<int>> stronglyComponents = tarjanComponentesFortementeConectados(adjList, numVertex);

    if (!fortementeConectado(adjList, numVertex))
        // se o grafo nao for fortemente conectado, ele nao é eulerianoo
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


    // se o grafo direcionado for eulerianoo
    return true;
}

// algoritmo de tarjan para encontrar as componentes fortementes conectadas em grafos direcionados
map<int, vector<int>> tarjanComponentesFortementeConectados(vector<Edge> *adjList, int numVertex) {
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
bool fortementeConectado(vector<Edge> *adjList, int numVertex) {
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

    vector<Edge> *trans = transposta(adjList, numVertex);

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

void imprimirArvoreProfundidade(vector<Edge> *adjList, int numVertex) {
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

void imprimirArvoreLargura(vector<Edge> *adjList, int numVertex) {
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

vector<Edge>* transposta(vector<Edge> *adjList, int numVertex) {
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

        for (int i = 0; i < numVertex; i++) {
            if (!visited[i]) {
                if (DFSCiclo(i, adjList, numVertex, visited, i)) {
                    return true;
                }
            }    
        }

        return false;
    } 

    // caso o grafo seja direcionado
    vector<string> state(numVertex, "WHITE");

    for (int i = 0; i < numVertex; i++) {
        if (state[i].compare("WHITE") == 0) {
            if (DFSCicloGrafoDirecionado(i, adjList, numVertex, state)) {
                return true;
            }
        }
    }

    return false;
}

// dfs para encontrar ciclos
// grafos nao direcionados
bool DFSCiclo(int vertex, vector<Edge> *adjList,  int numVertex, bool *visited, int parent) { 
    visited[vertex] = true;

    sort(adjList[vertex].begin(), adjList[vertex].end());

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;
        
        if (!visited[adjVertex]) {
            if (DFSCiclo(adjVertex, adjList, numVertex, visited, vertex)) {
                return true;
            }
        } else if (adjVertex != parent) {
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
vector<int> kahnOrdernacaoTopologica(vector<Edge> *adjList, int numVertex) {
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

    // adiciona todos os vértices com grau de entrada zero na fila
    for (int i = 0; i < numVertex; i++) {
        if (inDegree[i] == 0) {
            zeroDegreeQueue.push(i);
        }
    }

    while (!zeroDegreeQueue.empty()) {
        int u = zeroDegreeQueue.top();
        zeroDegreeQueue.pop();
        top.push_back(u);

        // reduz o grau de entrada dos vizinhos
        for (int i = 0; i < newAdjList[u].size(); i++) {
            int adjVertex = newAdjList[u][i].vertex;

            inDegree[adjVertex]--;

            if (inDegree[adjVertex] == 0) {
                zeroDegreeQueue.push(adjVertex);
            }
        }
    }

    return top;
}

int numeroVerticesEntrada(int vertex, vector<Edge> *adjList, int numVertex) {
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

void imprimirListaAdjacencia(vector<Edge> *adjList, int numVertex) {
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
    vector<int> dist(numVertex, INT_MAX); 
    vector<bool> visited(numVertex, false);

    dist[org] = 0; // A distância da origem para ela mesma é 0

    // Usando uma priority_queue para obter o vértice com a menor distância
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push(make_pair(0, org));

    while (!pq.empty()) {
        int currentVertex = pq.top().second;
        pq.pop();

        if (visited[currentVertex]) 
            continue;

        visited[currentVertex] = true;

        for (const Edge& edge : adjList[currentVertex]) {
            int adjVertex = edge.vertex;
            int weight = edge.weight;
            int cost = dist[currentVertex] + weight;
            
            // Verifica se existe um caminho mais curto para adjVertex através de currentVertex
            if (cost < dist[adjVertex]) {
                dist[adjVertex] = cost;
                pq.push(make_pair(dist[adjVertex], adjVertex));
            }
        }
    }

    // Verifica se há um caminho para o último vértice
    if (dist[numVertex - 1] == INT_MAX) {
        cout << "-1" << endl;
    } else {
        cout << dist[numVertex - 1] << endl;
    }
}

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


void DFSBridges(int u, int parent, vector<Edge> *adjList, int numVertex, vector<int>& visitedOrder, vector<int>& low, vector<Edge>& bridges) {
    static int time = 0;
    visitedOrder[u] = low[u] = ++time;

    for (int i = 0; i < adjList[u].size(); i++) {
        int v = adjList[u][i].vertex;

        if (visitedOrder[v] == -1) { // se v não foi visitado
            DFSBridges(v, u, adjList, numVertex, visitedOrder, low, bridges);
            low[u] = min(low[u], low[v]);

            // Se a menor ordem de visita de v é maior que a ordem de visita de u, então (u, v) é uma ponte
            if (low[v] > visitedOrder[u]) {
                bridges.push_back(Edge(v, adjList[u][i].weight, adjList[u][i].id)); // Aqui estamos assumindo que o ID é o mesmo para ambos os sentidos
            }
        } else if (v != parent) { // Atualiza o valor de low[u] se v foi visitado e não é o pai de u
            low[u] = min(low[u], visitedOrder[v]);
        }
    }
}

void getBridges(vector<Edge> *adjList, int numVertex) {
    vector<int> visitedOrder(numVertex, -1);
    vector<int> low(numVertex, -1);
    vector<Edge> bridges;

    for (int i = 0; i < numVertex; i++) {
        if (visitedOrder[i] == -1) {
            DFSBridges(i, -1, adjList, numVertex, visitedOrder, low, bridges);
        }
    }

    for (const Edge& bridge : bridges) {
        cout << bridge.id << " ";
    }
    cout << endl;
}

int find(int u, vector<int>& parent) {
    if (u != parent[u])
        parent[u] = find(parent[u], parent);
    return parent[u];
}

void merge(int u, int v, vector<int>& parent, vector<int>& rank) {
    int rootU = find(u, parent);
    int rootV = find(v, parent);

    if (rootU != rootV) {
        if (rank[rootU] < rank[rootV])
            parent[rootU] = rootV;
        else if (rank[rootU] > rank[rootV])
            parent[rootV] = rootU;
        else {
            parent[rootV] = rootU;
            rank[rootU]++;
        }
    }
}

// Função de comparação para o algoritmo de Kruskal
bool compareEdges(const Edge& a, const Edge& b) {
    return a.weight < b.weight;
}

int kruskal(int numVertex, vector<vector<int>>& capacity) {
    vector<Edge> edges;

    for (int u = 0; u < numVertex; ++u) {
        for (int v = 0; v < numVertex; ++v) {
            if (capacity[u][v] > 0) {
                edges.push_back(Edge(u, capacity[u][v], v));
            }
        }
    }

    sort(edges.begin(), edges.end(), compareEdges);

    vector<int> parent(numVertex);
    vector<int> rank(numVertex, 0);

    for (int i = 0; i < numVertex; ++i)
        parent[i] = i;

    int mstWeight = 0;

    for (Edge& edge : edges) {
        int u = edge.vertex;
        int v = edge.id;

        if (find(u, parent) != find(v, parent)) {
            mstWeight += edge.weight;
            merge(u, v, parent, rank);
        }
    }

    return mstWeight;
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