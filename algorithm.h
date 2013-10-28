#include "graph.h"

class ShortestPath
{
 public:
  // vector of previous v.id() in a shortest path
  vector<sz> dijkstraSimple(Graph &g, sz from, sz to);
  vector<sz> dijkstra(Graph &g, sz from, sz to); 
  vector<sz> aStar(Graph &g, sz from, sz to);
    
  void printPath(vector<sz> const& p, sz from, sz to);
};



  
