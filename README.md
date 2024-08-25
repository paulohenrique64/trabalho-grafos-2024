## Trabalho de Grafos
Trabalho de grafos da disciplina Algoritmo em Grafos do curso de Ciência da Computação da Universidade Federal de Lavras.

## Como rodar
Compile o projeto
```
g++ grafos.cpp -o grafos
```
Para executar
```
./grafos
```

## Exemplo de entrada para o programa
Considerando um grafo G = (V,E) não-direcionado e não-ponderado, V = {0,1,2,3}
e E = {(0,1), (1,2), (1,3), (2,3)}, tem-se:
```
1 3 6 9
4 4
nao_direcionado
0 0 1 1
1 1 2 1
2 1 3 1
3 2 3 1
```
Essa estrutura segue este padrao
> funcao_1 funcao_3 funcao_6 funcao_9
> vertices arestas <br>
> direcionado ou nao_direcionado <br>
> id_aresta vértice_u vértice_v peso_da_aresta <br>

Sendo

```funcao 0 - conectado```<br>
```funcao 1 - bipartido```<br>
```funcao 2 - euleriano```<br>
```funcao 3 - contem ciclos```<br>
```funcao 4 - quantidade de componentes conectadas```<br>
```funcao 5 - quantidade de componentes fortemente conectadas```<br>
```funcao 6 - articulacoes```<br>
```funcao 7 - pontes```<br>
```funcao 8 - árvore de busca em profundidade```<br>
```funcao 9 - árvore de busca em largura```<br>
```funcao 10 - kruskal árvore geradora mínima```<br>
```funcao 11 - kahn ordenação topologica```<br>
```funcao 12 - dijkstra caminhos mínimos```<br>
```funcao 13 - edmondsKarp```<br>
```funcao 14 - fechoTransitivo```<br>
