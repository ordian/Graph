#include "algorithm.h"
#include <iostream>

using std::cin;
using std::cout;
using std::endl;

int main(int argc, char** argv)
{
  if (argc < 3)
    {
      cout << "Usage: ./main USA-road-d.***.co USA-road-d.***.gr" << endl;
      return 1;
    }
  cout << "Reading graph..." << endl;
  Graph g(argv[1], argv[2]);
  cout << "Vertices: " << g.num_v() << endl
	    << "Edges: " << g.num_e() << endl;
 
  cout << "First vertex neighbours: " << endl;
  for (sz i = 0; i < g.neighbours(0).size(); ++i)
    cout << g.neighbours(0)[i] << " with weight " << g.edgeWeight(0, i) << endl;
  sz from = 0;
  sz to = 0;
  cout << "Enter source vertex from 0 to " << g.num_v() - 1 << ":" << endl;
  cin >> from;
  if (from >= g.num_v())
    return 1;
  cout << "Enter destination vertex (same range)" << ":" << endl;
  cin >> to;
  if (to >= g.num_v())
    return 1;
  
  cout << "Running Dijkstra..." << endl;
  ShortestPath d;
  
  vector<sz> p = d.dijkstra(g, from, to);
  if (INFINITY != g.vertex(to).getDistance())
    d.printPath(p, from, to);
  cout << endl;
  cout << "Weight: " << g.vertex(to).getDistance() << endl;

  g.visit(1);
  g.visit(2);
  if (g.notVisited(1) || g.notVisited(2))
    cout << "Need to test Visited" << endl;
  g.unvisitAll();
  if (!(g.notVisited(1) && g.notVisited(2)))
    cout << "Need to test Visited" << endl;
  
  return 0;
}
