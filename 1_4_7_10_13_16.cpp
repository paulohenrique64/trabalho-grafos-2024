#include <iostream>
#include <vector>
#include <utility>
#include <queue>
#include <stack>
#include <algorithm>

using namespace std;

class Grafo {
    private:
        int V;
        int A;
        bool ehDirecionado;
        vector<vector<pair<int, pair<int, int>>>> adj;
        vector<int> grauEntrada;
        vector<int> grauSaida;

        void removeAresta(int v1, int v2) {
            adj[v1].erase(remove_if(adj[v1].begin(), adj[v1].end(), [v2](const pair<int, pair<int, int>>& p) { return p.first == v2; }), adj[v1].end());

            if (!ehDirecionado) {
                adj[v2].erase(remove_if(adj[v2].begin(), adj[v2].end(), [v1](const pair<int, pair<int, int>>& p) { return p.first == v1; }), adj[v2].end());
            }
        }

        bool dfsCiclo(int v, vector<int>& estado) {
            estado[v] = 1; 

            // Explora todos os vértices adjacentes ao vértice 'v'
            for (auto& u : adj[v]) {
                if (estado[u.first] == 1) return true;

                if (estado[u.first] == 0 && dfsCiclo(u.first, estado)) return true;
            }

            estado[v] = 2;
            return false;
        }

        void DFS(int v, vector<bool>& visitado, int idFunc) {
            visitado[v] = true;
            //cout << "Visitando vértice " << v << endl;

            sort(adj[v].begin(), adj[v].end(), [](auto& a, auto& b) {
                return a.first < b.first;  // Compara os índices dos vértices
            });

            // Explorar todos os vértices adjacentes ao vértice 'v'
            for (auto &u : adj[v]) {

                if (!visitado[u.first]) {
                    //imprime o id_aresta de cada vértice
                    if(idFunc == 10) cout << u.second.first << " ";
                    DFS(u.first, visitado, idFunc);
                }
            }
        }

        void ordemTopologicaAux(int v, vector<bool>& visitado, stack<int>& ordem) {
            visitado[v] = true;

            // Explora todos os vértices adjacentes ao vértice 'v'
            for (auto& u : adj[v]) {
                if (!visitado[u.first]) {
                    ordemTopologicaAux(u.first, visitado, ordem);
                }
            }
            ordem.push(v);
        }

        // Função para verificar se a aresta u-v é uma ponte
        bool ehPonte(int u, int v) {
            // 1: Verifica quantos vértices são acessíveis antes de remover a aresta (conexo)
            vector<bool> visitado(V, false);
            int idFunc = 7;
            DFS(u, visitado, idFunc);

            int contAntes = count(visitado.begin(), visitado.end(), true);

            // 2: Remove a aresta u-v e verifica quantos vértices são acessíveis depois de remover a aresta
            removeAresta(u, v);
            fill(visitado.begin(), visitado.end(), false);
            DFS(u, visitado, idFunc);
            int contDepois = count(visitado.begin(), visitado.end(), true);

            // 3: Recoloca a aresta u-v no grafo
            adj[u].push_back({v, {0, 0}}); // id_aresta e custo são placeholders
            if (!ehDirecionado) adj[v].push_back({u, {0, 0}});

            // Se o número de vértices acessíveis diminuir, u-v é uma ponte
            return (contAntes > contDepois);
        }


    public:
        // construtor
        Grafo(int V, int A, bool ehDirecionado) {
            this->V = V; // atribui o número de vértices
            this->A = A; // atribui o número de arestas
            this->ehDirecionado = ehDirecionado; //atribui 1 para direcionado e 0 para nao_direcionado
            
            adj.resize(V + 1); // redimensiona o vetor de adjacência
            grauEntrada.resize(V, 0);
            grauSaida.resize(V, 0);
            
        }
        //construtor de copia
        Grafo(const Grafo& other) : adj(other.adj) {}

        // adiciona uma aresta ao grafo de v1 à v2
        void addAresta(int id_aresta, int v1, int v2, int custo) {
            adj[v1].push_back({v2, {id_aresta, custo}});
            grauSaida[v1]++;
            grauEntrada[v2]++;

            if (!ehDirecionado) {
                adj[v2].push_back({v1, {id_aresta, custo}});
                grauSaida[v2]++;
                grauEntrada[v1]++;
            }
            
            
        }

        // método para imprimir o grafo
        void imprime() {
            for (int i = 0; i < V; ++i) {
                cout << "\nVertice " << i << ":\n";
                for (auto &aresta : adj[i]) {
                    cout << "  Conectado a: " << aresta.first 
                         << " (Aresta ID: " << aresta.second.first 
                         << ", Peso: " << aresta.second.second << ")\n";
                }
                cout << endl;
            }
        }

        int bfs(int inicio) {
            vector<bool> visitado(V, false);
            queue<int> fila;

            // Iniciando BFS a partir do vértice 0
            fila.push(inicio);
            visitado[inicio] = true;

            int qtdVisitados = 0;
            while (!fila.empty()) {
                int v = fila.front();
                fila.pop();
                cout << "Visitando vértice " << v << endl;

                qtdVisitados++;
                // Explorar todos os vértices adjacentes ao vértice 'v'
                for (auto& u : adj[v]) {
                    if (!visitado[u.first]) {
                        fila.push(u.first);
                        visitado[u.first] = true;
                    }
                }
            }
            return qtdVisitados;
        }

        /*void ordenarAdjacencias() {
            for (int i = 0; i < V; ++i) {
                sort(adj[i].begin(), adj[i].end(), [](auto& a, auto& b) {
                    return a.first < b.first;  // Ordena em ordem crescente pelo índice
                });
            }
        }*/

        void dfs(int inicio, int idFunc) {
            vector<bool> visitado(V, false);
            DFS(inicio, visitado, idFunc);
        }

        bool conexo(int verticesVisitados) {
            if(verticesVisitados == V) return 1;
            else return 0;
        }

        bool possuiCiclo() {
            vector<int> estado(V, 0);
            
            // Faz uma DFS para cada vértice
            for (int i = 0; i < V; ++i) {
                if (estado[i] == 0) {
                    if (dfsCiclo(i, estado)) {
                        return true; 
                    }
                }
            }
            return false;
        }

        // Encontra e imprime a trilha euleriana
        void trilhaEuleriana(int u) {
            for (auto& v : adj[u]) {
                int vAtual = v.first;

                // Verifica se a aresta u-vAtual é válida para a trilha euleriana
                if (adj[u].size() == 1 || !ehPonte(u, vAtual)) {
                    cout << u << " ";

                    removeAresta(u, vAtual);
                    trilhaEuleriana(vAtual);
                    
                }
            }
        }

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

        /*bool verificaTrilhaEuleriana(int& inicio) {
            int qtdInicio = 0, qtdFim = 0;

            for (int i = 0; i < V; ++i) {
                if (grauSaida[i] - grauEntrada[i] == 1) {
                    inicio = i;
                    qtdInicio++;
                } else if (grauEntrada[i] - grauSaida[i] == 1) {
                    qtdFim++;
                } else if (grauEntrada[i] != grauSaida[i]) {
                    return false;
                }
            }

            return (qtdInicio == 1 && qtdFim == 1) || (qtdInicio == 0 && qtdFim == 0);
        }*/

        // Função para encontrar o vértice inicial da trilha euleriana
        // Inicio obrigatoriamente deve ser impar
        /*int encontrarInicioEuleriano() {
            int inicio = 0;
            for (int i = 0; i < V; i++) {
                if (adj[i].size() % 2 == 1) {
                    inicio = i;
                    break;
                }
            }
            return inicio;
        }*/

        void ordemTopologica() {
            stack<int> ordem;
            vector<bool> visitado(V, false);

            for (int i = 0; i < V; ++i) {
                if (!visitado[i]) {
                    ordemTopologicaAux(i, visitado, ordem);
                }
            }
            while (!ordem.empty()) {
                cout << ordem.top() << " ";
                ordem.pop();
            }
            cout << endl;
        }
        // Função auxiliar de DFS para encontrar fecho transitivo
        void dfsFecho(int v, vector<bool>& visitado, vector<int>& fecho) {
            visitado[v] = true;
            fecho.push_back(v); // Adiciona o vértice atual ao fecho transitivo

            for (auto& u : adj[v]) {
                if (!visitado[u.first]) {
                    dfsFecho(u.first, visitado, fecho);
                }
            }
        }
        // Função para calcular o fecho transitivo
        void fechoTransitivo() {
            if(ehDirecionado) {
                vector<vector<int>> fechoTransitivo(V); // Para armazenar o fecho transitivo de cada vértice

                for (int i = 0; i < V; ++i) {
                    vector<bool> visitado(V, false); // Marca todos os vértices como não visitados
                    dfsFecho(i, visitado, fechoTransitivo[i]); // Executa DFS para encontrar o fecho transitivo de i
                }

                // Imprime o fecho transitivo
                cout << "Fecho Transitivo:\n";
                for (int i = 0; i < V; ++i) {
                    cout << "Fecho transitivo de " << i << ": ";
                    for (int v : fechoTransitivo[i]) {
                        cout << v << " ";
                    }
                    cout << endl;
                }
            }
            else {
                cout << "-1" << endl;
            }
        }

};

int main () {

    int n, m;
    cin >> n >> m;

    if(((n < 1) || (n > 300)) && (m < (n - 1) || m > (5 * n))) {
        cerr << "Cada grafo G = (V,E) teve ter 1 ≤ |V| ≤ 300, e |V|-1 ≤ |E| ≤ 5 * |V|.";
        return 1;
    }

    string direcionado_ou_nao_direcionado;
    cin >> direcionado_ou_nao_direcionado;
    bool ehDirecionado = (direcionado_ou_nao_direcionado == "direcionado") ? true : false;

    Grafo grafo(n, m, ehDirecionado);

    for (int i = 0; i < m; i++){
        int id_aresta, u, v, peso;
        cin >> id_aresta >> u >> v >> peso;

        grafo.addAresta(id_aresta, u, v, peso);
    }

    grafo.imprime();
    //grafo.bfs(0);
    cout << grafo.conexo(grafo.bfs(0)) << endl;

    cout << grafo.possuiCiclo() << endl;

    //Copia para não alterar a lista adj original(precisa de reparos)
    /*Grafo copia = grafo;
    int inicio = 0;
    if(copia.conexo(copia.bfs(0)) && copia.verificaTrilhaEuleriana(inicio)){
        copia.trilhaEuleriana(inicio);
    }
    else {
        cout << "-1" << endl;
    }*/

    //grafo.imprime();
    cout << endl << "/////////////////////////" << endl;
    int idFunc = 10;
    grafo.dfs(0, idFunc);
    
    cout << endl << "/////////////////////////" << endl;
    if(ehDirecionado && !grafo.possuiCiclo()) grafo.ordemTopologica();
    else cout << "-1" << endl;

    grafo.fechoTransitivo();

    return 0; 
}