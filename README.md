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
4 4
nao_direcionado
0 0 1 1
1 1 2 1
2 1 3 1
3 2 3 1
```
Essa estrutura segue este padrao
> vertices arestas <br>
> direcionado ou nao_direcionado <br>
> id_aresta vértice_u vértice_v peso_da_aresta <br>
