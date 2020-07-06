

// Copyright srcmake.com 2018.
// C++ Example Dijkstra Algorithm For Shortest Path (With PQ/Min-Heap)

/* The Dijkstra algorithm:
    // Initialize the graph adjacency list. adjList[i] = pair<int, int> where first is vertex, second is edge weight.
    // Initialize all source->vertex as infinite.
    // Create a PQ.
    // Add source to pq, where distance is 0.
    // While pq isn't empty...
        // Get min distance vertex from pq. (Call it u.)
        // Visit all of u's friends. For each one (called v)....
            // If the distance to v is shorter by going through u...
                // Update the distance of v.
                // Insert v into the pq.
*/

// The example graph: https://www.srcmake.com/uploads/5/3/9/0/5390645/spgraph_orig.jpg

#include <queue>
#include <iostream>
#include <fstream>
#include <array>
#include <stdlib.h>     /* exit, EXIT_FAILURE */
#include <vector>
#include <cmath>
#include <iterator>
using namespace std;


#define X_MAX 606
#define X_STEP 1
#define Y_MAX 606
#define Y_STEP 1
#define FLT_MAX 1000
#define MAX_SLOPE 0.25
//using namespace std;



int indexfromsub(int x, int y){
  int ind = y*X_MAX + x ;
  return ind;
}

static bool isValid(int x, int y) { //If our Node is an obstacle it is not valid

        //int id = x + y * (X_MAX / X_STEP);
        // int x = m.x;
        // int y = m.y;
        //if (slopeMap[x][y] < MAX_SLOPE) {
        if (x < 0 || y < 0 || x >= (X_MAX / X_STEP) || y >= (Y_MAX / Y_STEP)) {
            return false;
        }
        return true;
        //}
        //return false;
    }

// An adjacency list. Each adjList[i] holds a all the friends of node i.
// The first int is the vertex of the friend, the second int is the edge weight.
vector< vector<pair<int, int> > > FormAdjList(float map[X_MAX][Y_MAX])
    {
    // Our adjacency list.
    vector< vector<pair<int, int> > > adjList;

    // We have 7 vertices, so initialize 7 rows.
    const int n = X_MAX*Y_MAX;

    for(int i = 0; i < n; i++)
        {
        // Create a vector to represent a row, and add it to the adjList.
        vector<pair<int, int> > row;
        adjList.push_back(row);
        }


    // Now let's add our actual edges into the adjacency list.
    // See the picture here: https://www.srcmake.com/uploads/5/3/9/0/5390645/spadjlist_orig.jpg

    for (int x = 0; x < (X_MAX / X_STEP); x++) {
       for (int y = 0; y < (Y_MAX / Y_STEP); y++) {
         for (int newX = -1; newX <= 1; newX++) {
            for (int newY = -1; newY <= 1; newY++) {
                 if (isValid(x + newX ,y + newY)){
                  // you can change the edge cost in this line. currently it is the difference between slopes at the two vertices of the edge
                   adjList[indexfromsub(x,y)].push_back(make_pair(indexfromsub(x + newX,y + newY),round(std::abs((map[x][y] - map[x + newX][y + newY])*100)) ));
                 }
               }
             }
           }
         }

    // Our graph is now represented as an adjacency list. Return it.
    return adjList;
    }

// Given an Adjacency List, find all shortest paths from "start" to all other vertices.
vector<int> DijkstraSP(vector< vector<pair<int, int> > > &adjList, int &start)
    {
    cout << "\nGetting the shortest path from " << start << " to all other nodes.\n";
    vector<int> dist;

    // Initialize all source->vertex as infinite.
    int n = adjList.size();
    for(int i = 0; i < n; i++)
        {
        dist.push_back(1000000007); // Define "infinity" as necessary by constraints.
        }

    // Create a PQ.
    priority_queue<pair<int, int>, vector< pair<int, int> >, greater<pair<int, int> > > pq;

    // Add source to pq, where distance is 0.
    pq.push(make_pair(start, 0));
    dist[start] = 0;

    // While pq isn't empty...
    while(pq.empty() == false)
        {
        // Get min distance vertex from pq. (Call it u.)
        int u = pq.top().first;
        pq.pop();

        // Visit all of u's friends. For each one (called v)....
        for(int i = 0; i < adjList[u].size(); i++)
            {
            int v = adjList[u][i].first;
            int weight = adjList[u][i].second;

            // If the distance to v is shorter by going through u...
            if(dist[v] > dist[u] + weight)
                {
                // Update the distance of v.
                dist[v] = dist[u] + weight;
                // Insert v into the pq.
                pq.push(make_pair(v, dist[v]));
                }
            }
        }

    return dist;
    }

void PrintShortestPath(vector<int> &dist, int &start)
    {
    cout << "\nPrinting the shortest paths for node " << start << ".\n";
    for(int i = 0; i < dist.size(); i++)
        {
        cout << "The distance from node " << start << " to node " << i << " is: " << dist[i] << endl;
        }
    }

int main()
    {
    cout << "Program started.\n";

    float slopeMap[X_MAX][Y_MAX];
    ifstream loadfile;
    loadfile.open("slopemap.txt");

    for (int i = 0; i < X_MAX ; i++ ){
      for (int j = 0; j < X_MAX; j++){
        loadfile >> slopeMap[i][j];
      }
    }
    // Construct the adjacency list that represents our graph.
    vector< vector<pair<int, int> > > adjList = FormAdjList(slopeMap);

    // Get a list of shortest path distances for node **.
    int node = 20000;  //choose your index [map coordinates need to be changed to indices].
    vector<int> dist = DijkstraSP(adjList, node);

    // Print the list.
    //PrintShortestPath(dist, node);
    //cout << dist[3000] << endl;
    cout << "Program ended.\n";
    std::ofstream output_file("cost2goal.txt");
    std::ostream_iterator<int> output_iterator(output_file, "\n");
    std::copy(dist.begin(), dist.end(), output_iterator);
    return 0;
    }
