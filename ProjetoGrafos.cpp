/*
Saída nos casos testes:
EX0:     2-> 0 (Não pede)     5-> 1 (Correto)     8-> -1 (Não pede)     11-> 539 (Não pede)     14 -> X (Falta implementar)    
EX1:     2-> -1 (EX errado)     5-> -1 (Correto)     8-> -1 (Correto)     11-> -1 (Correto)     14 -> X (Falta implementar)
EX2:     2-> -1 (EX errado)     5-> -1 (Correto)     8-> -1 (Não pede)     11-> -1 (Não pede)     14 -> X (Falta implementar)
EX3:     2-> 0 (Correto)     5-> 1 (Correto)     8-> 198 (Não pede)     11-> 773 (Correto)     14 -> X (Falta implementar)
EX4:     2-> 0 (Correto)     5-> 1 (Não pede)     8-> -1 (Correto)     11-> 895 (Incorreto [-1 ????])     14 -> X (Falta implementar)
EX5:     2-> 0 (Correto)     5-> 1 (Não pede)     8-> 56 101 164 372 496 685 (Correto)     11-> 5272 (Não pede)     14 -> X (Falta implementar)
EX6:     2-> 0 (Correto)     5-> 1 (Correto)     8-> 124 151 (Não pede)     11-> 868 (Incorreto [-1 ????])     14 -> X (Falta implementar)
EX7:     2-> 0 (Incorreto[1])     5-> 2 (Não pede)     8-> 20 26 27 (Correto)     11-> 912 (Não pede)     14 -> X (Falta implementar)
EX8:     2-> 0 (Não pede)     5-> 1 (Não pede)     8-> -1 (Não pede)     11-> 2521 (Correto)     14 -> X (Falta implementar)
EX9:     2-> -1 (EX errado[0])     5-> -1 (Correto)     8-> -1 (Não pede)     11-> -1 (Correto)     14 -> X (Falta implementar)
EX10:    2-> -1 (Não pede)     5-> -1 (Correto)     8-> -1 (Não pede)     11-> -1 (Correto)     14 -> X (Falta implementar)
EX11:    2-> -1 (Não pede)     5-> -1 (Correto)     8-> -1 (Não pede)     11-> -1 (Não pede)     14 -> X (Falta implementar)
EX12:    2-> -1 (EX errado[0])     5-> -1 (Não pede)     8-> -1 (Correto)     11-> -1 (Correto)     14 -> X (Falta implementar)
EX13:    2-> 1 (Correto)     5-> 1 (Correto)     8-> 0 1 2 (Não pede)     11-> 3 (Correto)     14 -> X (Falta implementar)
EX14:    2-> 1 (Não pede)     5-> 1 (Não pede)     8-> 0 (Não pede)     11-> 1 (Não pede)     14 -> X (Falta implementar)
*/


#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <string>
#include <limits>
#include <algorithm>
using namespace std;

class Grafo
{
private:
    int numeroVertices;
    int numeroArestas;
    bool ehDirecionado;
    vector<list<pair<int, pair<int, int>>>> listaAdjacente;
    vector<pair<int, pair<int, int>>> arestas; // (peso, (origem, destino))
    vector<int> descoberta, menorTempo;
    vector<bool> foiVisitado;
    int tempoAtual;

    // Função DFS usada para calcular os valores de low-link
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

    // Função auxiliar DFS para encontrar componentes conexas
    void DFSParaComponentesConexas(int vertice, vector<bool> &foiVisitado)
    {
        foiVisitado[vertice] = true;
        for (auto &adjacente : listaAdjacente[vertice])
        {
            if (!foiVisitado[adjacente.first])
                DFSParaComponentesConexas(adjacente.first, foiVisitado);
        }
    }

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

public:
    Grafo(int numVertices, int numArestas = 0, string tipoGrafo = "")
    {
        numeroVertices = numVertices;
        numeroArestas = numArestas;
        ehDirecionado = (tipoGrafo == "direcionado");

        listaAdjacente.resize(numeroVertices);
        descoberta.resize(numeroVertices, -1);
        menorTempo.resize(numeroVertices, -1);
        foiVisitado.resize(numeroVertices, false);
        tempoAtual = 0;
    }

    // Função para adicionar uma aresta ponderada
    void adicionarAresta(int origem, int destino, int peso, int idAresta)
    {
        listaAdjacente[origem].push_back(make_pair(destino, make_pair(peso, idAresta)));
        arestas.push_back({peso, {origem, destino}});
        if (!ehDirecionado)
            listaAdjacente[destino].push_back(make_pair(origem, make_pair(peso, idAresta)));
    }

    // Função para verificar se o grafo é bipartido usando BFS
    bool verificarBipartido()
    {
        vector<int> corVertices(numeroVertices, -1);

        for (int i = 0; i < numeroVertices; i++)
        {
            if (corVertices[i] == -1)
            {
                queue<int> fila;
                fila.push(i);
                corVertices[i] = 0;

                while (!fila.empty())
                {
                    int vertice = fila.front();
                    fila.pop();

                    for (const auto &aresta : listaAdjacente[vertice])
                    {
                        int vizinho = aresta.first;
                        if (corVertices[vizinho] == -1)
                        {
                            corVertices[vizinho] = 1 - corVertices[vertice];
                            fila.push(vizinho);
                        }
                        else if (corVertices[vizinho] == corVertices[vertice])
                            return false; // Se dois vizinhos têm a mesma cor, não é bipartido
                    }
                }
            }
        }
        return true;
    }

    // Função para contar as componentes conexas
    int contarComponentesConexas()
    {
        vector<bool> visitados(numeroVertices, false);
        int quantidadeComponentes = 0;

        for (int i = 0; i < numeroVertices; i++)
        {
            if (!visitados[i])
            {
                quantidadeComponentes++;
                DFSParaComponentesConexas(i, visitados);
            }
        }
        return quantidadeComponentes;
    }

    // Função para encontrar as arestas ponte em um grafo não direcionado
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

    // Função para calcular o valor total da MST usando o algoritmo de Kruskal
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
};


int main()
{
    int numeroVertices, numeroArestas;
    string tipoGrafo;

    cin >> numeroVertices >> numeroArestas;
    cin >> tipoGrafo;

    Grafo G(numeroVertices, numeroArestas, tipoGrafo);

    for (int i = 0; i < numeroArestas; ++i)
    {
        int idAresta, origem, destino, peso;
        cin >> idAresta >> origem >> destino >> peso;
        G.adicionarAresta(origem, destino, peso, idAresta);
    }

    // 2. Verifica se o grafo é bipartido
    if (tipoGrafo == "direcionado")
        cout << "2: -1" << endl;
    else
        cout << "2: " << G.verificarBipartido() << endl;

    // 5. Conta as componentes conexas
    if (tipoGrafo == "direcionado")
        cout << "5: -1" << endl;
    else
        cout << "5: " << G.contarComponentesConexas() << endl;

    // 8. Exibe as arestas ponte
    if (tipoGrafo == "direcionado")
        cout << "8: -1" << endl;
    else
    {
        cout << "8: ";
        G.encontrarPontes();
    }

    // 11. Calcular e exibir o valor total da MST usando Kruskal
    if (tipoGrafo == "direcionado")
        cout << "11: -1" << endl;
    else
        cout << "11: " << G.calcularKruskalMST() << endl;

    
    if (tipoGrafo == "nao_direcionado")
        cout << "14: -1" << endl;
    else
    {
        cout << "14: " << G.calcularFluxoMaximo(0, numeroVertices- 1) << endl;
    }

    return 0;
}