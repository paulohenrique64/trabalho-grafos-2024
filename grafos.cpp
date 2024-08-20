#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <map>
#include <algorithm>
#include <sstream>

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
};

// dfs e bfs
void DFS(int vertex, vector<Edge> *adjList, int numVertex, bool* visited);
void DFSTarjan(int vertex, vector<Edge> *adjList, int numVertex, int *pre, int *lo, stack<int>& stack, bool *stackMember);
void BFSTree(int initialVertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
void DFSTree(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
bool DFSCycleDirectedGraph(int vertex, vector<Edge> *adjList,  int numVertex,vector<string>& state);

// funções essenciais
bool eulerian(vector<Edge> *adjList, int numVertex, string direction);
map<int, vector<int>> tarjanStronglyComponents(vector<Edge> *adjList, int numVertex);
void printDephTree(vector<Edge> *adjList, int numVertex);
vector<int> kahnTopologicalSort(vector<Edge> *adjList, int numVertex);
vector<Edge>* transitiveClosure(vector<Edge> *adjList, int numVertex);

// funções auxiliares
int getVertexIncomingDegree(int vertex, vector<Edge> *adjList, int numVertex);
bool containsCycleDirectedGraph(vector<Edge> *adjList, int numVertex);
vector<Edge>* transpose(vector<Edge> *adjList, int numVertex);
bool stronglyConnected(vector<Edge> *adjList, int numVertex);
void printAdjList(vector<Edge> *adjList, int numVertex);

// funcoes do paulo silveira
vector<int> getArticulacoes(vector<Edge> *adjList, int numVertex);
int DFSArticulacoes(int u, vector<Edge> *adjList, int numVertex, vector<int>& visitedOrder, vector<int>& ans);

// funções do gabriel
bool checkBipartite(vector<Edge> *adjList, int numVertex);
int countConnectedComponents(int numVertex, vector<Edge> *adjList);
void DFSConnectedComponents(int vertex, vector<Edge> *adjList, vector<bool>& visited);

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
            case 1: {
                // funcao 1 - paulo silveira - verificar se o grafo é conexo
                break;
            }
            case 2: {
                // funcao 2 - gabriel 
                cout << "2 - bipartite:" << checkBipartite(adjList, numVertex) << endl;
                break;
            }
            case 3: {
                // funcao 3 - paulo alves - eulerian
                cout << "3 - eulerian: " << eulerian(adjList, numVertex, direction) << endl;
                break;
            }
            case 4: {
                // funcao 4 - paulo silveira
                cout << "4 - verify cycle: " << eulerian(adjList, numVertex, direction) << endl;
                break;
            }
            case 5: {
                // funcao 5 - gabriel 
                break;
            }
            case 6: {
                // funcao 6 - paulo alves - strongly components
                map<int, vector<int>> stronglyComponents = tarjanStronglyComponents(adjList, numVertex);
                cout << "6 - amount of strongly components: " << stronglyComponents.size() << endl;
                break;
            }
            case 7: {
                // funcao 7 - paulo silveira
                cout << "7 - ariculacoes: ";
                vector<int> articulacoes = getArticulacoes(adjList, numVertex);
                for (int i = 0; i < articulacoes.size(); i++) 
                    cout << articulacoes[i] << " ";
                cout << endl;
                break;
            }
            case 8: {
                // funcao 8 - gabriel 
                cout << "5 - connected components:" << countConnectedComponents(numVertex, adjList) << endl;
                break;
            }
            case 9: {
                // deep search tree
                cout << "9 - deph search tree: ";
                printDephTree(adjList, numVertex);
                break;
            }
            case 10: {
                // funcao 10 - paulo silveira
                break;
            }
            case 11: {
                // funcao 11 - gabriel 
                break;
            }
            case 12: {
                // funcao 12 - paulo alves - ordenação topologica
                cout << "12 - topologial sort: ";
                if (direction.compare("direcionado") == 0 and !containsCycleDirectedGraph(adjList, numVertex)) {
                    // se o grafo for direciondo e nao possuir ciclos
                    vector<int> top = kahnTopologicalSort(adjList, numVertex);

                    for (int i = 0; i < top.size(); i++) 
                        cout << top[i] << " ";
                    cout << endl;
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 13: {
                // funcao 13 - paulo silveira
                break;
            }
            case 14: {
                // funcao 14 - gabriel 
                break;
            }
            case 15: {
                // funcao 15 - paulo alves - fecho transitivo
                cout << "15 - fecho transitivo: ";
                if (direction.compare("direcionado") == 0) {
                    vector<Edge> *newAdjList = transitiveClosure(adjList, numVertex);
                    printAdjList(newAdjList, numVertex);
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
        }
    }

    return 0;
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
        vector<int> v;
        stronglyComponents.insert_or_assign(lo[i], v);
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
    bool visited[numVertex];
    
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

/// dfs 
void DFS(int vertex, vector<Edge> *adjList, int numVertex, bool* visited) {
    visited[vertex] = true;

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (!visited[adjVertex]) 
            DFS(adjVertex, adjList, numVertex, visited);
    }
}

void printDephTree(vector<Edge> *adjList, int numVertex) {
    int initialVertex = 0;
    bool visited[numVertex];
    vector<int> order;
    
    for (int i = 0; i < numVertex; i++) 
        visited[i] = false;

    DFSTree(initialVertex, adjList, numVertex, visited, order);

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
bool containsCycleDirectedGraph(vector<Edge> *adjList, int numVertex) {
    vector<string> state(numVertex, "WHITE");

    for (int i = 0; i < numVertex; ++i) 
        if (state[i].compare("WHITE") == 0) 
            if (DFSCycleDirectedGraph(i, adjList, numVertex, state))  
                return true;

    return false;
}

// dfs para encontrar ciclos
bool DFSCycleDirectedGraph(int vertex, vector<Edge> *adjList,  int numVertex, vector<string>& state) {
    state[vertex] = "GRAY";  // Marca o vértice como "em processo"

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (state[adjVertex].compare("GRAY") == 0) 
            return true;
        

        if (state[adjVertex].compare("WHITE") == 0) 
            if (DFSCycleDirectedGraph(adjVertex, adjList, numVertex, state)) 
                return true;
    }

    state[vertex] = "BLACK"; 
    return false;
}

// algoritmo de kahn para encontrar ordem topológica
vector<int> kahnTopologicalSort(vector<Edge> *adjList, int numVertex) {
    vector<int> top; // armazenar a ordenação topologica
    bool *onTopList = new bool[numVertex];
    vector<Edge> *newAdjList = new vector<Edge>[numVertex]; // clone da lista de adjacência
    
    for (int i = 0; i < numVertex; i++) {
        newAdjList[i] = adjList[i];
        onTopList[i] = false;
    }   

    while (top.size() < numVertex) {
        for (int i = 0; i < numVertex; i++) {
            if (!onTopList[i] and getVertexIncomingDegree(i, newAdjList, numVertex) == 0) {
                top.push_back(i);
                onTopList[i] = true;
                newAdjList[i].clear();           
            }
        }
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

vector<Edge>* transitiveClosure(vector<Edge> *adjList, int numVertex) {
    vector<Edge> *newAdjList = new vector<Edge>[numVertex];

    for (int i = 0; i < numVertex; i++) {
        bool *visited = new bool[numVertex];

        for (int j = 0; j < numVertex; j++)
            visited[j] = false;

        DFS(i, adjList, numVertex, visited);

        for (int j = 0; j < numVertex; j++) {
            if (j == i) continue;

            if (visited[j])
                newAdjList[i].push_back(Edge(j, 0));
        }
    }

    return newAdjList;
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
    vector<int> visitedOrder(numVertex, 0), ans;
    int initialVertex = 0;
    time_s = 1;

    DFSArticulacoes(initialVertex, adjList, numVertex, visitedOrder, ans);
    
    return ans;
}

// dfs para o algoritmo de encontrar articulacoes
int DFSArticulacoes(int u, vector<Edge> *adjList, int numVertex, vector<int>& visitedOrder, vector<int>& ans) {  
    visitedOrder[u] = time_s++;
    int menor = visitedOrder[u];
    int filhos = 0;

    // u -> vértice atual sendo explorado pela DFS
    // menor -> mantém o menor valor de visitedOrder alcançado na subárvore DFS do vértice u
    // filhos -> conta o número de filhos diretos de u na árvore DFS
    // adjList[u] -> acessar os filhos (arestas) do vertice u
    // adjList[u][1] -> acessar o primeiro filho (aresta) do vertice u
    
    for (int i = 0; i < adjList[u].size(); i++) {
        int ajdVertex = adjList[u][i].vertex;

        if (visitedOrder[ajdVertex] == 0) {
            // caso o vertice adjacente nao tenha sido visitado
            filhos++;
             
            int m = DFSArticulacoes(ajdVertex, adjList, numVertex, visitedOrder, ans);
            
            menor = min(menor, m);

            if (visitedOrder[u] <= m && (u!=0 || filhos>=2))
                ans.push_back(u);
       }else {
          menor = min(menor, visitedOrder[ajdVertex]);
       }
    } 

    return menor;      
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

/*
PARA RESOLVER A QUESTÂO 8

void DFSParaPontes(int vertice, int verticePai, vector<int> &pontes)
    {
        foiVisitado[vertice] = true;
        descoberta[vertice] = menorTempo[vertice] = tempoAtual++;

        for (auto &adjacente : listaAdjacente[vertice])
        {
            int vizinho = adjacente.first;
            int idAresta = adjacente.second.second;
            if (!foiVisitado[vizinho])
            {
                DFSParaPontes(vizinho, vertice, pontes);
                menorTempo[vertice] = min(menorTempo[vertice], menorTempo[vizinho]);

                // Se menorTempo[vizinho] > descoberta[vertice], então (vertice, vizinho) é uma ponte
                if (menorTempo[vizinho] > descoberta[vertice])
                {
                    pontes.push_back(idAresta);
                }
            }
        else if (vizinho != verticePai)
        {
                menorTempo[vertice] = min(menorTempo[vertice], descoberta[vizinho]);
        }
    }
}

void encontrarPontes()
    {
        vector<int> pontes;
        for (int i = 0; i < numeroVertices; i++)
        {
            if (!foiVisitado[i])
            {
                DFSParaPontes(i, -1, pontes);
            }
        }

        sort(pontes.begin(), pontes.end()); // Ordenar os IDs das pontes

        for (int id : pontes)
        {
            cout << id << " ";
        }

        if(pontes.empty())
            cout << "-1";

        cout << endl;

        
    }

*/


/*
PARA RESOLVER QUESTÃO 11

// Função para encontrar o representante do conjunto de um elemento (usado em Kruskal)
    int encontrarConjunto(int vertice, vector<int> &pai)
    {
        if (vertice != pai[vertice])
            pai[vertice] = encontrarConjunto(pai[vertice], pai);
        return pai[vertice];
    }

    // Função para unir dois conjuntos (usado em Kruskal)
    void unirConjuntos(int vertice1, int vertice2, vector<int> &pai, vector<int> &rank)
    {
        vertice1 = encontrarConjunto(vertice1, pai);
        vertice2 = encontrarConjunto(vertice2, pai);
        if (rank[vertice1] < rank[vertice2])
            pai[vertice1] = vertice2;
        else if (rank[vertice1] > rank[vertice2])
            pai[vertice2] = vertice1;
        else
        {
            pai[vertice2] = vertice1;
            rank[vertice1]++;
        }
    }
    

int calcularKruskalMST()
    {
        sort(arestas.begin(), arestas.end());
        vector<int> pai(numeroVertices);
        vector<int> rank(numeroVertices, 0);

        for (int i = 0; i < numeroVertices; i++)
            pai[i] = i;

        int valorTotal = 0;
        for (auto &aresta : arestas)
        {
            int peso = aresta.first;
            int origem = aresta.second.first;
            int destino = aresta.second.second;

            if (encontrarConjunto(origem, pai) != encontrarConjunto(destino, pai))
            {
                valorTotal += peso;
                unirConjuntos(origem, destino, pai, rank);
            }
        }
        return valorTotal;
    }
*/

/*
PARA RESOLVER A QUESTÃO 14
// Função para realizar a busca em largura (BFS) e encontrar um caminho aumentante
         bool bfs(vector<vector<int>> &rGraph, int s, int t, vector<int> &parent)
        {
            vector<bool> visitado(numeroVertices, false);
            queue<int> fila;
            fila.push(s);
            visitado[s] = true;
            parent[s] = -1;

            while (!fila.empty())
            {
                int u = fila.front();
                fila.pop();

                for (int v = 0; v < numeroVertices; v++)
                {
                    if (!visitado[v] && rGraph[u][v] > 0)
                    {
                        if (v == t)
                        {
                            parent[v] = u;
                            return true;
                        }
                        fila.push(v);
                        parent[v] = u;
                        visitado[v] = true;
                    }
                }
            }
            return false;
        }

        
// Função para calcular o valor do fluxo máximo
    int calcularFluxoMaximo(int origem, int destino)
    {
        vector<vector<int>> capacidade(numeroVertices, vector<int>(numeroVertices, 0));

        for (auto &aresta : arestas)
        {
            int origem = aresta.second.first;
            int destino = aresta.second.second;
            int capacidadeAresta = aresta.first;
            capacidade[origem][destino] += capacidadeAresta;
        }

        vector<vector<int>> rGraph = capacidade;
        vector<int> parent(numeroVertices);
        int fluxoMaximo = 0;
        
        // Se não for possível chegar ao destino
        if (!bfs(rGraph, origem, destino, parent)) 
            return -1;

        while (bfs(rGraph, origem, destino, parent))
        {
            int fluxo = numeric_limits<int>::max();
            for (int v = destino; v != origem; v = parent[v])
            {
                int u = parent[v];
                fluxo = min(fluxo, rGraph[u][v]);
            }

            for (int v = destino; v != origem; v = parent[v])
            {
                int u = parent[v];
                rGraph[u][v] -= fluxo;
                rGraph[v][u] += fluxo;
            }

            fluxoMaximo += fluxo;
        }

        return fluxoMaximo;
    }
    */