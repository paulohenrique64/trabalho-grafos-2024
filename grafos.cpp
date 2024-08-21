// UFLA        Universidade Federal de Lavras
// DCC         Departamento de Ciência da Computação
// Disciplina  GCC218 - Algoritmos em Grafos
// Professor   Mayron

// Trabalho de Grafos
// Grupo: G
// Integrantes:
// Gabriel Jardim de Souza           - Matrícula: 202310530 - Turma 10A
// Paulo Henrique dos Anjos Silveira - Matrícula: 202310533 - Turma 10A
// Paulo Henrique Ribeiro Alves      - Matrícula: 202310171 - Turma 10A

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

//Estrutura que armazena as informações da aresta
struct Aresta {
    int vertice;
    int peso;
    int id;

    //construtor
    Aresta(int vertice, int peso, int id = -1) {
        this->vertice = vertice;//armazena o vértice
        this->peso = peso;//armazena o peso
        this->id = id;//armazena o id da aresta
    }
    //sobrecargas
    bool operator<(const Aresta& other) const {
        return vertice < other.vertice; 
    }

    bool operator>(const Aresta& other) const {
        return vertice > other.vertice;
    }
};

// funções e métodos de dfs e bfs
void DFSFecho(int vertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado, vector<int>& reached);
void DFS(int vertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado);
void DFSTarjan(int vertice, vector<Aresta> *listaAdj, int numVertices, int *pre, int *lo, stack<int>& stack, bool *stackMember);
void BFSArvore(int initialvertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado, vector<int>& ordemVisitacao);
void DFSArvore(int vertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado, vector<int>& ordemVisitacao);
bool DFSCicloGrafoDirecionado(int vertice, vector<Aresta> *listaAdj,  int numVertices,vector<string>& state);
bool DFSCiclo(int vertice, vector<Aresta> *listaAdj,  int numVertices, bool *visitado, int pai);
void DFSComponentesConectados(int vertice, vector<Aresta> *listaAdj, vector<bool>& visitado);
void DFSPontes(int u, int pai, vector<Aresta> *listaAdj, int numVertices, vector<int>& ordemVisitacao, vector<int>& low, vector<Aresta>& pontes);
void DFSArticulacoes(int u, vector<Aresta>* listaAdj, vector<int>& ordemVisitacao, vector<int>& low, vector<int>& pai, vector<int>& ans);

// funções essenciais
bool euleriano(vector<Aresta> *listaAdj, int numVertices, string tipoGrafo);
map<int, vector<int>> tarjanComponentesFortementeConectados(vector<Aresta> *listaAdj, int numVertices);
void imprimirArvoreProfundidade(vector<Aresta> *listaAdj, int numVertices);
void imprimirArvoreLargura(vector<Aresta> *listaAdj, int numVertices);
vector<int> kahnOrdenacaoTopologica(vector<Aresta> *listaAdj, int numVertices);
void fechoTransitivo(vector<Aresta> *listaAdj, int numVertices);
int kruskal(int numVertices, vector<vector<int>>& capacity);
int edmondsKarp(const vector<vector<int>>& capacity, int source, int sink);
void retornarPontes(vector<Aresta> *listaAdj, int numVertices);
bool checarBipartido(vector<Aresta> *listaAdj, int numVertices);
int contarComponentesConectados(int numVertices, vector<Aresta> *listaAdj);
bool conectado(vector<Aresta> *listaAdj, int numVertices, string tipoGrafo);
vector<int> retornarArticulacoes(vector<Aresta> *listaAdj, int numVertices);
void dijkstra(int org, vector<Aresta>* listaAdj, int numVertices);
bool contemCiclo(vector<Aresta> *listaAdj, int numVertices, string tipoGrafo);

// funções auxiliares
int numeroVerticesEntrada(int vertice, vector<Aresta> *listaAdj, int numVertices);
vector<Aresta>* transposta(vector<Aresta> *listaAdj, int numVertices);
bool fortementeConectado(vector<Aresta> *listaAdj, int numVertices);
void imprimirListaAdjacencia(vector<Aresta> *listaAdj, int numVertices);
int find(int u, vector<int>& pai);
bool compararArestasPorPeso(const Aresta& a, const Aresta& b);

// variaveis globais auxiliares
int tempo; // para o algoritomo de encontrar articulações e para oalgoritmo de tarjan para componentes conexas

int main() {
    int comando, numVertices, numArestas, arestaId, verticeU, verticeV, peso;
    vector<Aresta> *listaAdj = nullptr;
    vector<int> comandos;
    string linha, tipoGrafo;

    // ler a lista de comandos passada na entrada padrao   
    getline(cin, linha); 

    stringstream buffer(linha);
    while (buffer >> comando)
        comandos.push_back(comando);

    // ler o numero de vertices, numero de arestas e se o grafo e direcionado
    cin >> numVertices >> numArestas >> tipoGrafo; 
    listaAdj = new vector<Aresta>[numVertices]; 

    // Vetor usado em fluxo máximo e kruskal
    vector<vector<int>> capacity(numVertices, vector<int>(numVertices, 0)); 

    for (int i = 0; i < numArestas; i++) {
        // preenche a lista de adjacência
        cin >> arestaId >> verticeU >> verticeV >> peso;

        // usado em fluxo máximo e kruskal
        capacity[verticeU][verticeV] = peso; 

        listaAdj[verticeU].push_back(Aresta(verticeV, peso, arestaId));

        if (tipoGrafo.compare("nao_direcionado") == 0) 
            listaAdj[verticeV].push_back(Aresta(verticeU, peso, arestaId));
    }

    for (int i = 0; i < comandos.size(); i++) {
        switch(comandos[i]) {
            case 0: {
                cout << conectado(listaAdj, numVertices, tipoGrafo) << endl;
                break;
            }
            case 1: {
                cout << checarBipartido(listaAdj, numVertices) << endl;
                break;
            }
            case 2: {
                cout << euleriano(listaAdj, numVertices, tipoGrafo) << endl;
                break;
            }
            case 3: {
                cout << contemCiclo(listaAdj, numVertices, tipoGrafo) << endl;
                break;
            }
            case 4: {
                if (tipoGrafo.compare("nao_direcionado") == 0)
                    cout << contarComponentesConectados(numVertices, listaAdj) << endl;
                else
                    cout << "-1" << endl;
                break;
            }
            case 5: {
                if (tipoGrafo.compare("direcionado") == 0) {
                    map<int, vector<int>> componentes = tarjanComponentesFortementeConectados(listaAdj, numVertices);
                    cout << componentes.size() << endl;
                } else {
                    cout << "-1" << endl;
                }           
                break;
            }
            case 6: {
                if (tipoGrafo.compare("nao_direcionado") == 0) {
                    vector<int> articulacoes = retornarArticulacoes(listaAdj, numVertices);

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
                if (tipoGrafo.compare("nao_direcionado") == 0)
                    retornarPontes(listaAdj, numVertices);
                else
                    cout << "-1" << endl;
                break;
            }
            case 8: {
                imprimirArvoreProfundidade(listaAdj, numVertices);
                break;
            }
            case 9: {
                imprimirArvoreLargura(listaAdj, numVertices);
                break;
            }
            case 10: {
                if (tipoGrafo.compare("nao_direcionado") == 0) {
                    int mstpeso = kruskal(numVertices, capacity);
                    cout << mstpeso << endl;
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 11: {
                if (tipoGrafo.compare("direcionado") == 0 && !contemCiclo(listaAdj, numVertices, tipoGrafo)) {
                    vector<int> top = kahnOrdenacaoTopologica(listaAdj, numVertices);
                    for (int i = 0; i < top.size(); i++) 
                        cout << top[i] << " ";
                    cout << endl;
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
            case 12: {
                dijkstra(0, listaAdj, numVertices);
                break;
            }
            case 13: {
                if (tipoGrafo.compare("nao_direcionado") == 0)
                    cout << "-1" << endl;
                else
                    cout << edmondsKarp(capacity, 0, numVertices - 1) << endl;
                break;
            }
            case 14: {
                if (tipoGrafo.compare("direcionado") == 0) {
                    fechoTransitivo(listaAdj, numVertices);
                } else {
                    cout << "-1" << endl;
                }
                break;
            }
        }
    }

    delete[] listaAdj;

    return 0;
}
//Verifica se o grafo é conectado para direcionado e não direcionado
bool conectado(vector<Aresta> *listaAdj, int numVertices, string tipoGrafo) {
    bool *visitado = new bool[numVertices];

    if (tipoGrafo.compare("direcionado") == 0) {
        // converter o grafo para nao direcionado e chamar a DFS nele
        vector<Aresta> *novaListaAdj = new vector<Aresta>[numVertices]; // armazenar a nova versao nao direcionada
    
        for (int i = 0; i < numVertices; i++) 
            novaListaAdj[i] = listaAdj[i];
        
        for (int i = 0; i < numVertices; i++) {
            for (int j = 0; j < novaListaAdj[i].size(); j++) {
                Aresta aresta = novaListaAdj[i][j];
                novaListaAdj[aresta.vertice].push_back(Aresta(i, aresta.peso, aresta.id));
            }
        }

        // chamar a dfs para o novo grafo direcionado convertido para nao direcionado
        DFS(0, novaListaAdj, numVertices, visitado);

        delete[] novaListaAdj;
    } 

    // chamar a dfs para o grafo nao direcionado
    DFS(0, listaAdj, numVertices, visitado);
    
    for (int i = 0; i < numVertices; i++) {
        if (!visitado[i]) {
            delete[] visitado;
            return false;
        }  
    }
        
    delete[] visitado;

    return true;
}


// verificar se o grafo é euleriano
bool euleriano(vector<Aresta> *listaAdj, int numVertices, string tipoGrafo) {
    int numVerticesGrauImpar = 0;

    if (tipoGrafo.compare("nao_direcionado") == 0) {
        // caso o grafo seja não direcionado
        for (int i = 0; i < numVertices; i++) {
        int vertice = i;
        int cont = 0;

        for (int j = 0; j < numVertices; j++) {
            for (int k = 0; k < listaAdj[j].size(); k++) {
                if (listaAdj[j][k].vertice == vertice)
                    cont++;
            }
        }

        if (cont % 2 != 0)
            numVerticesGrauImpar++;
        }

        // se o numero de vértices com grau ímpar for igual a 0
        // o grafo é eleriano
        return numVerticesGrauImpar == 0;
    } 
    
    // caso o grafo seja direcionado
    if (!fortementeConectado(listaAdj, numVertices))
        // se o grafo nao for fortemente conectado, ele nao é eulerianoo
        return false;

    // se o grafo for fortemente conectado, verifica se em cada vértice
    // os graus de entrada são iguais aos graus de saída
    for (int i = 0; i < numVertices; i++) {
        int vertice = i;
        int grausEntrada = 0;
        int grausSaida = 0;

        for (int j = 0; j < numVertices; j++) {
            for (int k = 0; k < listaAdj[j].size(); k++) {
                if (listaAdj[j][k].vertice == vertice)
                    grausEntrada++;
            }
        }

        grausSaida = listaAdj[vertice].size();

        if (grausEntrada != grausSaida)
            return false;
    }

    // se o grafo direcionado for eulerianoo
    return true;
}

// algoritmo de tarjan para encontrar as componentes fortementes conectadas em grafos direcionados
map<int, vector<int>> tarjanComponentesFortementeConectados(vector<Aresta> *listaAdj, int numVertices) {
    int *pre = new int[numVertices];
    int *lo = new int[numVertices];
    bool *stackMember = new bool[numVertices];
    stack<int> stack;
    map<int, vector<int>> componentes;
    tempo = 0;

    for (int i = 0; i < numVertices; i++) {
        pre[i] = -1;
        lo[i] = -1;
        stackMember[i] = false;
    }
        
    for (int i = 0; i < numVertices; i++) {
        if (pre[i] == -1)
            DFSTarjan(i, listaAdj, numVertices, pre, lo, stack, stackMember);
    }

    for (int i = 0; i < numVertices; i++) {
        componentes[lo[i]].push_back(i);
    }

    delete[] pre;
    delete[] lo;
    delete[] stackMember;

    return componentes;
}

// dfs para o algoritmo de tarjan
void DFSTarjan(int vertice, vector<Aresta> *listaAdj, int numVertices, int *pre, int *lo, stack<int>& sc, bool *stackMember) {
    pre[vertice] = tempo++;
    lo[vertice] = pre[vertice];
    sc.push(vertice);
    stackMember[vertice] = true;

    sort(listaAdj[vertice].begin(), listaAdj[vertice].end());

    for (int i = 0; i < listaAdj[vertice].size(); i++) {
        int adjvertice = listaAdj[vertice][i].vertice;

        if (pre[adjvertice] == -1) {
            // se encontrou um vértice adjacente ainda não visitado
            DFSTarjan(adjvertice, listaAdj, numVertices, pre, lo, sc, stackMember); 
            lo[vertice] = min(lo[vertice], lo[adjvertice]);
        } else if (stackMember[adjvertice] == true) { 
            // se encontrou um vértice adjacente que já foi visitado
            lo[vertice] = min(lo[vertice], pre[adjvertice]); 
        } 
    }
    
    int v = 0;

    if (lo[vertice] == pre[vertice]) {
        // cout << "finded new head " << vertice << endl;
        // encontrou uma "head"

        while (sc.top() != vertice) {
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
bool fortementeConectado(vector<Aresta> *listaAdj, int numVertices) {
    int initialvertice = 0;
    bool* visitado = new bool[numVertices];
    vector<Aresta> *listaAdjTransposta = nullptr;
    
    for (int i = 0; i < numVertices; i++) 
        visitado[i] = false;

    DFS(initialvertice, listaAdj, numVertices, visitado);

    for (int i = 0; i < numVertices; i++) {
        if (!visitado[i]) 
            return false;

        visitado[i] = false;
    }

    // obter a transposta do grafo
    listaAdjTransposta = transposta(listaAdj, numVertices);

    DFS(initialvertice, listaAdjTransposta, numVertices, visitado);

    delete[] listaAdjTransposta;

    for (int i = 0; i < numVertices; i++) 
        if (!visitado[i]) 
            return false;

    return true;
}

//Dfs para calcular fecho transitivo
void DFSFecho(int vertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado, vector<int>& reached) {
    visitado[vertice] = true;
    reached.push_back(vertice);

    sort(listaAdj[vertice].begin(), listaAdj[vertice].end());

    for (int i = 0; i < listaAdj[vertice].size(); i++) {
        int adjvertice = listaAdj[vertice][i].vertice;

        if (!visitado[adjvertice]) 
            DFSFecho(adjvertice, listaAdj, numVertices, visitado, reached);
    }
}

// Dfs generica
void DFS(int vertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado) {
    visitado[vertice] = true;

    sort(listaAdj[vertice].begin(), listaAdj[vertice].end());

    for (int i = 0; i < listaAdj[vertice].size(); i++) {
        int adjvertice = listaAdj[vertice][i].vertice;

        if (!visitado[adjvertice]) 
            DFS(adjvertice, listaAdj, numVertices, visitado);
    }
}

//Chama DFS e imprime árvore de profundidade 
void imprimirArvoreProfundidade(vector<Aresta> *listaAdj, int numVertices) {
    int initialvertice = 0;
    bool* visitado = new bool[numVertices];
    vector<int> ordemVisitacao;
    
    for (int i = 0; i < numVertices; i++) 
        visitado[i] = false;

    DFSArvore(initialvertice, listaAdj, numVertices, visitado, ordemVisitacao);

    for (int i = 0; i < ordemVisitacao.size(); i++)
        cout << ordemVisitacao[i] << " ";
    cout << endl;

    delete[] visitado;
}
//Chama BFS e imprime árvore de largura 
void imprimirArvoreLargura(vector<Aresta> *listaAdj, int numVertices) {
    int initialvertice = 0;
    bool* visitado = new bool[numVertices];
    vector<int> ordemVisitacao;
    
    for (int i = 0; i < numVertices; i++) 
        visitado[i] = false;

    BFSArvore(initialvertice, listaAdj, numVertices, visitado, ordemVisitacao);

    for (int i = 0; i < ordemVisitacao.size(); i++)
        cout << ordemVisitacao[i] << " ";
    cout << endl;

    delete[] visitado;
}

//Realiza bfs para árvore de largura
void BFSArvore(int initialvertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado, vector<int>& ordemVisitacao) {
    queue<int> q;

    visitado[initialvertice] = true;
    q.push(initialvertice);

    while (!q.empty()) {
        int vertice = q.front();
        q.pop();

        sort(listaAdj[vertice].begin(), listaAdj[vertice].end());

        for (int i = 0; i < listaAdj[vertice].size(); i++) {
            int adjvertice = listaAdj[vertice][i].vertice;

            if (!visitado[adjvertice]) {
                ordemVisitacao.push_back(listaAdj[vertice][i].id);
                q.push(adjvertice);
                visitado[adjvertice] = true;
            }
        }
    }
}

//Realiza dfs para árvore de profundidade
void DFSArvore(int vertice, vector<Aresta> *listaAdj, int numVertices, bool* visitado, vector<int>& ordemVisitacao) {
    visitado[vertice] = true;

    sort(listaAdj[vertice].begin(), listaAdj[vertice].end());

    for (int i = 0; i < listaAdj[vertice].size(); i++) {
        int adjvertice = listaAdj[vertice][i].vertice;

        if (!visitado[adjvertice]) {
            ordemVisitacao.push_back(listaAdj[vertice][i].id);
            DFSArvore(adjvertice, listaAdj, numVertices, visitado, ordemVisitacao);
        }
    }
}
// transposta de uma lista de adj
vector<Aresta>* transposta(vector<Aresta> *listaAdj, int numVertices) {
    vector<Aresta> *trans = new vector<Aresta>[numVertices];

    for (int i = 0; i < numVertices; i++) 
        for (int j = 0; j < listaAdj[i].size(); j++) 
            trans[j].push_back(Aresta(i, listaAdj[i][j].peso));

    return trans;
}

// verificar se o grafo contem ciclo
bool contemCiclo(vector<Aresta> *listaAdj, int numVertices, string tipoGrafo) {
    if (tipoGrafo.compare("nao_direcionado") == 0) {
        // caso o grafo seja nao direcionado
        bool *visitado = new bool[numVertices];
    
        for (int i = 0; i < numVertices; i++)
            visitado[i] = false;

        for (int i = 0; i < numVertices; i++) {
            if (!visitado[i]) {
                if (DFSCiclo(i, listaAdj, numVertices, visitado, i)) {
                    return true;
                }
            }    
        }

        return false;
    } 

    // caso o grafo seja direcionado
    vector<string> state(numVertices, "WHITE");

    for (int i = 0; i < numVertices; i++) {
        if (state[i].compare("WHITE") == 0) {
            if (DFSCicloGrafoDirecionado(i, listaAdj, numVertices, state)) {
                return true;
            }
        }
    }

    return false;
}

// dfs para encontrar ciclos
// grafos nao direcionados
bool DFSCiclo(int vertice, vector<Aresta> *listaAdj,  int numVertices, bool *visitado, int pai) { 
    visitado[vertice] = true;

    sort(listaAdj[vertice].begin(), listaAdj[vertice].end());

    for (int i = 0; i < listaAdj[vertice].size(); i++) {
        int adjvertice = listaAdj[vertice][i].vertice;
        
        if (!visitado[adjvertice]) {
            if (DFSCiclo(adjvertice, listaAdj, numVertices, visitado, vertice)) {
                return true;
            }
        } else if (adjvertice != pai) {
            return true; // Se adjvertice foi visitado e não é o pai, há um ciclo
        }
    }

    return false; 
}

// dfs para encontrar ciclos 
// grafos direcionados
bool DFSCicloGrafoDirecionado(int vertice, vector<Aresta> *listaAdj,  int numVertices, vector<string>& state) {
    state[vertice] = "GRAY";  // Marca o vértice como "em processo"

    sort(listaAdj[vertice].begin(), listaAdj[vertice].end());

    for (int i = 0; i < listaAdj[vertice].size(); i++) {
        int adjvertice = listaAdj[vertice][i].vertice;

        if (state[adjvertice].compare("GRAY") == 0) 
            return true;
        

        if (state[adjvertice].compare("WHITE") == 0) 
            if (DFSCicloGrafoDirecionado(adjvertice, listaAdj, numVertices, state)) 
                return true;
    }

    state[vertice] = "BLACK"; 

    return false;
}

// algoritmo de kahn para encontrar ordem topológica
vector<int> kahnOrdenacaoTopologica(vector<Aresta> *listaAdj, int numVertices) {
    vector<int> top; // armazenar a ordenação topológica
    vector<int> grausEntrada(numVertices, 0); // grau de entrada dos vértices
    vector<Aresta> *novaListaAdj = new vector<Aresta>[numVertices]; // armazenar a nova versao nao direcionada
    stack<int> zeroGrausQueue;

    for (int i = 0; i < numVertices; i++) 
        novaListaAdj[i] = listaAdj[i];

    // calcula o grau de entrada de cada vértice
    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < listaAdj[i].size(); j++) {
            int adjvertice = listaAdj[i][j].vertice;
            grausEntrada[adjvertice]++; 
        }
    }

    // adiciona todos os vértices com grau de entrada zero na fila
    for (int i = 0; i < numVertices; i++) {
        if (grausEntrada[i] == 0) {
            zeroGrausQueue.push(i);
        }
    }

    while (!zeroGrausQueue.empty()) {
        int u = zeroGrausQueue.top();
        zeroGrausQueue.pop();
        top.push_back(u);

        // reduz o grau de entrada dos vizinhos
        for (int i = 0; i < novaListaAdj[u].size(); i++) {
            int adjvertice = novaListaAdj[u][i].vertice;

            grausEntrada[adjvertice]--;

            if (grausEntrada[adjvertice] == 0) {
                zeroGrausQueue.push(adjvertice);
            }
        }
    }

    delete[] novaListaAdj;

    return top;
}

// Conta o número de vértices de entrada
int numeroVerticesEntrada(int vertice, vector<Aresta> *listaAdj, int numVertices) {
    int cont = 0; 

    for (int i = 0; i < numVertices; i++) 
        for (int j = 0; j < listaAdj[i].size(); j++) 
            if (listaAdj[i][j].vertice == vertice)
                cont++;
        
    return cont;
}

//Realiza e imprime o fecho
void fechoTransitivo(vector<Aresta> *listaAdj, int numVertices) {
    bool *visitado = new bool[numVertices];
    vector<int> fechoTransitivo;

    for (int j = 0; j < numVertices; j++)
        visitado[j] = false;

    DFSFecho(0, listaAdj, numVertices, visitado, fechoTransitivo);
 
    for (int j = 0; j < fechoTransitivo.size(); j++) {
        if (fechoTransitivo[j] != 0)
            cout << fechoTransitivo[j] << " ";
    }

    delete[] visitado;
}

// Imprime a lista de adjacência
void imprimirListaAdjacencia(vector<Aresta> *listaAdj, int numVertices) {
    for (int i = 0; i < numVertices; i++) {
        cout << "[" << i << "] -> ";
        for (int j = 0; j < listaAdj[i].size(); j++)
            cout << "(" << listaAdj[i][j].vertice << ", " << listaAdj[i][j].peso << ") ";
        cout << endl;
    }
}

// retorna um vetor contendo todos vertices articulacoes
vector<int> retornarArticulacoes(vector<Aresta> *listaAdj, int numVertices) {
    vector<int> ordemVisitacao(numVertices, -1), low(numVertices, -1), pai(numVertices, -1), ans;
    tempo = 0;

    for(int i =0; i < numVertices; i++) {
        if(ordemVisitacao[i] == -1) {
            DFSArticulacoes(i, listaAdj, ordemVisitacao, low, pai, ans);
        }
    }
    
    return ans;
}

// dfs para o algoritmo de encontrar articulacoes
void DFSArticulacoes(int u, vector<Aresta>* listaAdj, vector<int>& ordemVisitacao, vector<int>& low, vector<int>& pai, vector<int>& ans) {  
    ordemVisitacao[u] = low[u] = tempo++;
    int filhos = 0;

    // u -> vértice atual sendo explorado pela DFS
    // menor -> mantém o menor valor de ordemVisitacao alcançado na subárvore DFS do vértice u
    // filhos -> conta o número de filhos diretos de u na árvore DFS
    // listaAdj[u] -> acessar os filhos (arestas) do vertice u
    // listaAdj[u][1] -> acessar o primeiro filho (aresta) do vertice u
    
    for (int i = 0; i < listaAdj[u].size(); i++) {
        int adjvertice = listaAdj[u][i].vertice;

         if (ordemVisitacao[adjvertice] == -1) {  // Vértice adjacente ainda não visitado
            filhos++;
            pai[adjvertice] = u;
             
            DFSArticulacoes(adjvertice, listaAdj, ordemVisitacao, low, pai, ans);
            
            low[u] = min(low[u], low[adjvertice]);

            // Verifica se u é um ponto de articulação
            if ((pai[u] == -1 && filhos > 1) || (pai[u] != -1 && low[adjvertice] >= ordemVisitacao[u])) {
                ans.push_back(u);  // u é um ponto de articulação
            }
        } else if (adjvertice != pai[u]) {  // Atualiza low[u] para back Aresta
            low[u] = min(low[u], ordemVisitacao[adjvertice]);
        }
    }  
}

//Realiza o caminho mínimo usando dijkstra
void dijkstra(int org, vector<Aresta>* listaAdj, int numVertices) {
    vector<int> dist(numVertices, INT_MAX); 
    vector<bool> visitado(numVertices, false);

    dist[org] = 0; // A distância da origem para ela mesma é 0

    // Usando uma priority_queue para obter o vértice com a menor distância
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push(make_pair(0, org));

    while (!pq.empty()) {
        int currentvertice = pq.top().second;
        pq.pop();

        if (visitado[currentvertice]) 
            continue;

        visitado[currentvertice] = true;

        for (const Aresta& Aresta : listaAdj[currentvertice]) {
            int adjvertice = Aresta.vertice;
            int peso = Aresta.peso;
            int cost = dist[currentvertice] + peso;
            
            // Verifica se existe um caminho mais curto para adjvertice através de currentvertice
            if (cost < dist[adjvertice]) {
                dist[adjvertice] = cost;
                pq.push(make_pair(dist[adjvertice], adjvertice));
            }
        }
    }

    // Verifica se há um caminho para o último vértice
    if (dist[numVertices - 1] == INT_MAX) {
        cout << "-1" << endl;
    } else {
        cout << dist[numVertices - 1] << endl;
    }
}

//Função para verificar se o grafo é bipartido
bool checarBipartido(vector<Aresta> *listaAdj, int numVertices) {
    vector<int> color(numVertices, -1);

    for (int i = 0; i < numVertices; i++){

        //Se o vértice não foi visitado
        if (color[i] == -1){
            queue<int> queue;
            queue.push(i);
            color[i] = 0;

            while (!queue.empty()){
                int u = queue.front();
                queue.pop();
                
                //Atribui a cor oposta ao vértice adjacente
                for (const auto &Aresta : listaAdj[u]){
                    int v = Aresta.vertice;
                    
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

// DFS (usadacontarComponentesConectados
void DFSComponentesConectados(int vertice, vector<Aresta> *listaAdj, vector<bool>& visitado) {
    visitado[vertice] = true;

    for (const Aresta& Aresta : listaAdj[vertice]) {
        if (!visitado[Aresta.vertice]) {
            DFSComponentesConectados(Aresta.vertice, listaAdj, visitado);
        }
    }
}

// Função para contar componentes conexas
int contarComponentesConectados(int numVertices, vector<Aresta> *listaAdj) {
    vector<bool> visitado(numVertices, false);
    int conectadoComponents = 0;

    for (int i = 0; i < numVertices; i++) {
        if (!visitado[i]) {
            DFSComponentesConectados(i, listaAdj, visitado);
                conectadoComponents++;
        }
    }
    return conectadoComponents;
}

// Realiza uma DFS para o algoritmo de pontes
void DFSPontes(int u, int pai, vector<Aresta> *listaAdj, int numVertices, vector<int>& ordemVisitacao, vector<int>& low, vector<Aresta>& pontes) {
    static int time = 0;
    ordemVisitacao[u] = low[u] = ++time;

    for (int i = 0; i < listaAdj[u].size(); i++) {
        int v = listaAdj[u][i].vertice;

        if (ordemVisitacao[v] == -1) { // se v não foi visitado
            DFSPontes(v, u, listaAdj, numVertices, ordemVisitacao, low, pontes);
            low[u] = min(low[u], low[v]);

            // Se a menor ordem de visita de v é maior que a ordem de visita de u, então (u, v) é uma ponte
            if (low[v] > ordemVisitacao[u]) {
                pontes.push_back(Aresta(v, listaAdj[u][i].peso, listaAdj[u][i].id)); // Aqui estamos assumindo que o ID é o mesmo para ambos os sentidos
            }
        } else if (v != pai) { // Atualiza o valor de low[u] se v foi visitado e não é o pai de u
            low[u] = min(low[u], ordemVisitacao[v]);
        }
    }
}

// Realiza o algoritmo pontos e imprime as pontes
void retornarPontes(vector<Aresta> *listaAdj, int numVertices) {
    vector<int> ordemVisitacao(numVertices, -1);
    vector<int> low(numVertices, -1);
    vector<Aresta> pontes;

    for (int i = 0; i < numVertices; i++) {
        if (ordemVisitacao[i] == -1) {
            DFSPontes(i, -1, listaAdj, numVertices, ordemVisitacao, low, pontes);
        }
    }

    sort(pontes.begin(), pontes.end(), [](const Aresta &a, const Aresta &b) {
        return a.id < b.id;
    });

    for (const Aresta& bridge : pontes) {
        cout << bridge.id << " ";
    }
    cout << endl;
}
//Função para encontrar o pai de um conjunto
int find(int u, vector<int>& pai) {
    if (u != pai[u])
        pai[u] = find(pai[u], pai);
    return pai[u];
}
//Função para unir subconjuntos
void merge(int u, int v, vector<int>& pai, vector<int>& rank) {
    int rootU = find(u, pai);
    int rootV = find(v, pai);

    if (rootU != rootV) {
        if (rank[rootU] < rank[rootV])
            pai[rootU] = rootV;
        else if (rank[rootU] > rank[rootV])
            pai[rootV] = rootU;
        else {
            pai[rootV] = rootU;
            rank[rootU]++;
        }
    }
}

// Função de comparação para o algoritmo de Kruskal
bool compararArestasPorPeso(const Aresta& a, const Aresta& b) {
    return a.peso < b.peso;
}

//Algoritmo de kruskal que retorna o peso 
int kruskal(int numVertices, vector<vector<int>>& capacity) {
    vector<Aresta> Arestas;

    for (int u = 0; u < numVertices; ++u) {
        for (int v = 0; v < numVertices; ++v) {
            if (capacity[u][v] > 0) {
                Arestas.push_back(Aresta(u, capacity[u][v], v));
            }
        }
    }

    sort(Arestas.begin(), Arestas.end(), compararArestasPorPeso);

    vector<int> pai(numVertices);
    vector<int> rank(numVertices, 0);

    for (int i = 0; i < numVertices; ++i)
        pai[i] = i;

    int mstpeso = 0;

    for (Aresta& Aresta : Arestas) {
        int u = Aresta.vertice;
        int v = Aresta.id;

        if (find(u, pai) != find(v, pai)) {
            mstpeso += Aresta.peso;
            merge(u, v, pai, rank);
        }
    }

    return mstpeso;
}

// Realiza BFS para fluxo máximo
bool bfsMaxFlow(const vector<vector<int>>& capacity, const vector<vector<int>>& residual, int source, int sink, vector<int>& pai) {
    int numVertices = capacity.size();
    vector<bool> visitado(numVertices, false);
    queue<int> q;

    q.push(source);
    visitado[source] = true;
    pai[source] = -1; // Fonte não tem pai

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v = 0; v < numVertices; ++v) {
            if (!visitado[v] && residual[u][v] > 0) { // Se o vértice não foi visitado e a capacidade residual > 0
                q.push(v);
                pai[v] = u;
                visitado[v] = true;
                
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
    int numVertices = capacity.size();
    vector<vector<int>> residual(numVertices, vector<int>(numVertices, 0));
    
    // Inicializa a rede residual com as capacidades originais
    for (int u = 0; u < numVertices; ++u) {
        for (int v = 0; v < numVertices; ++v) {
            residual[u][v] = capacity[u][v];
        }
    }

    vector<int> pai(numVertices);
    int maxFlow = 0;

    // Enquanto existir um caminho de aumento
    while (bfsMaxFlow(capacity, residual, source, sink, pai)) {
        int pathFlow = numeric_limits<int>::max();

        // Encontrar a capacidade mínima do caminho encontrado
        for (int v = sink; v != source; v = pai[v]) {
            int u = pai[v];
            pathFlow = min(pathFlow, residual[u][v]);
        }

        // Atualizar capacidades residuais da rede
        for (int v = sink; v != source; v = pai[v]) {
            int u = pai[v];
            residual[u][v] -= pathFlow;
            residual[v][u] += pathFlow;
        }

        maxFlow += pathFlow; // Adiciona o fluxo do caminho ao fluxo máximo total
    }

    return maxFlow;
}