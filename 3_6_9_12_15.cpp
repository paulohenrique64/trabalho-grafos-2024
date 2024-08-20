#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <map>

using namespace std;

int order; // tarjan algorithm

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

// dfs and bfs
void DFS(int vertex, vector<Edge> *adjList, int numVertex, bool* visited);
void DFSTarjan(int vertex, vector<Edge> *adjList, int numVertex, int *pre, int *lo, stack<int>& stack, bool *stackMember);
void BFSTree(int initialVertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
void DFSTree(int vertex, vector<Edge> *adjList, int numVertex, bool* visited, vector<int>& order);
bool DFSCycle(int vertex, vector<Edge> *adjList,  int numVertex,vector<string>& state);

// options
bool eulerian(vector<Edge> *adjList, int numVertex, string direction);
map<int, vector<int>> tarjanStronglyComponents(vector<Edge> *adjList, int numVertex);
void printDephTree(vector<Edge> *adjList, int numVertex);
vector<int> kahnTopologicalSort(vector<Edge> *adjList, int numVertex);
vector<Edge>* transitiveClosure(vector<Edge> *adjList, int numVertex);

// aux
int getVertexIncomingDegree(int vertex, vector<Edge> *adjList, int numVertex);
bool containsCycle(vector<Edge> *adjList, int numVertex);
vector<Edge>* transpose(vector<Edge> *adjList, int numVertex);
bool stronglyConnected(vector<Edge> *adjList, int numVertex);
void printAdjList(vector<Edge> *adjList, int numVertex);

int main() {
    int numVertex, numEdges, edgeId, vertexU, vertexV, weight;
    vector<Edge> *adjList = nullptr;
    string direction;

    cin >> numVertex >> numEdges >> direction;
    adjList = new vector<Edge>[numVertex]; 

    for (int i = 0; i < numEdges; i++) {
        // fill adjList
        cin >> edgeId >> vertexU >> vertexV >> weight;

        adjList[vertexU].push_back(Edge(vertexV, weight, edgeId));

        if (direction.compare("nao_direcionado") == 0) 
            adjList[vertexV].push_back(Edge(vertexU, weight, edgeId));
    }

    // eulerian
    cout << "3 - eulerian: " << eulerian(adjList, numVertex, direction) << endl;

    // strongly components
    map<int, vector<int>> stronglyComponents = tarjanStronglyComponents(adjList, numVertex);
    cout << "6 - amount of strongly components: " << stronglyComponents.size() << endl;

    // topological sort
    cout << "12 - topologial sort: ";
    if (direction.compare("direcionado") == 0 and !containsCycle(adjList, numVertex)) {
        // if the graph is directed and does not contain cycles
        vector<int> top = kahnTopologicalSort(adjList, numVertex);

        for (int i = 0; i < top.size(); i++) 
            cout << top[i] << " ";

        cout << endl;
    } else {
        cout << "-1" << endl;
    }

    // transitive closure
    cout << "15 - transitive closure: ";
    if (direction.compare("direcionado") == 0) {
        vector<Edge> *newAdjList = transitiveClosure(adjList, numVertex);
        printAdjList(newAdjList, numVertex);
    } else {
        cout << "-1" << endl;
    }

    return 0;
}



bool eulerian(vector<Edge> *adjList, int numVertex, string direction) {
    int numOddDegreeVertex = 0;

    if (direction.compare("nao_direcionado") == 0) {
        // non directed graph
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

        return numOddDegreeVertex == 0;
    } 
    
    // directed graph
    map<int, vector<int>> stronglyComponents = tarjanStronglyComponents(adjList, numVertex);

    if (!stronglyConnected(adjList, numVertex))
        // verify if graph is strongly connected
        return false;

    // verify is incoming degree is equal to outcoming degree
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


    // if the directed graph is eulerian
    return true;
}

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


void DFSTarjan(int vertex, vector<Edge> *adjList, int numVertex, int *pre, int *lo, stack<int>& sc, bool *stackMember) {
    pre[vertex] = order++;
    lo[vertex] = pre[vertex];
    sc.push(vertex);
    stackMember[vertex] = true;

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (pre[adjVertex] == -1) {
            // if adjVertex are not visited
            DFSTarjan(adjVertex, adjList, numVertex, pre, lo, sc, stackMember); 
            lo[vertex] = min(lo[vertex], lo[adjVertex]);
        } else if (stackMember[adjVertex] == true) { 
            // if find a visited adjVertex 
            lo[vertex] = min(lo[vertex], pre[adjVertex]); 
        } 
    }
    
    int v = 0;

    if (lo[vertex] == pre[vertex]) {
        // cout << "finded new head " << vertex << endl;
        // it's a head

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

// find cycles in directed graphs
bool containsCycle(vector<Edge> *adjList, int numVertex) {
    vector<string> state(numVertex, "WHITE");

    for (int i = 0; i < numVertex; ++i) 
        if (state[i].compare("WHITE") == 0) 
            if (DFSCycle(i, adjList, numVertex, state))  
                return true;

    return false;
}

// find cycles in directed graphs
bool DFSCycle(int vertex, vector<Edge> *adjList,  int numVertex, vector<string>& state) {
    state[vertex] = "GRAY";  // Marca o vértice como "em processo"

    for (int i = 0; i < adjList[vertex].size(); i++) {
        int adjVertex = adjList[vertex][i].vertex;

        if (state[adjVertex].compare("GRAY") == 0) 
            return true;
        

        if (state[adjVertex].compare("WHITE") == 0) 
            if (DFSCycle(adjVertex, adjList, numVertex, state)) 
                return true;
    }

    state[vertex] = "BLACK"; 
    return false;
}

vector<int> kahnTopologicalSort(vector<Edge> *adjList, int numVertex) {
    vector<int> top; // store topological sort
    bool *onTopList = new bool[numVertex];
    vector<Edge> *newAdjList = new vector<Edge>[numVertex]; // clone of adj list
    
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
