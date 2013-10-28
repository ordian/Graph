#include "algorithm.h"
#include "priority_queue.h"
//#include <queue>
#include <iostream> // printPath

struct cmp {
    bool operator() (Vertex const &u, Vertex const &v) {
      return u.getDistance() < v.getDistance();
    }
};

vector<sz> ShortestPath::dijkstraSimple(Graph &g, sz from, sz to)
{
  vector<sz> prev;
  for (sz i = 0; i < g.num_v(); ++i)
    prev.push_back(i);

  
  priority_queue<Vertex, cmp> Q;
  for (size_t i = 0; i < g.num_v(); ++i)
    g.distance(i) = INFINITY; // cmath
  
  g.distance(from) = 0;
  Q.push(g.vertex(from));
  
  sz u, v;
  double w;
  g.unvisitAll();
  while(!Q.empty())
    {
      // extract min
      u = Q.top().id();
      Q.pop();
      
      if (!g.notVisited(u)) continue;
      if (u == to) break;
      
      // explore u's neighbours
      vector<sz> neighbours = g.neighbours(u);
      
      for (sz i = 0; i != neighbours.size(); ++i)
	{
	  v = g.vertex(neighbours[i]).id();
	  w = g.edgeWeight(u, i);
	  if (g.notVisited(v) && 
	      g.distance(u) + w < g.distance(v))
	    {
	      g.distance(v) = g.distance(u) + w;
	      Q.push(Vertex(v, g.distance(v)));
	      prev[v] = u;
	    }
	}
      // visit u
      g.visit(u);
    }
  
  // cleaning up
  g.unvisitAll();
  
  return prev;
}

vector<sz> ShortestPath::dijkstra(Graph &g, sz from, sz to)
{
  Visited black(g.num_v());
  g.unvisitAll();
  vector<sz> vertex_queue_positions(g.num_v(), 0);
  
  vector<sz> prev;
  for (sz i = 0; i < g.num_v(); ++i)
    prev.push_back(i);
  priority_queue<Vertex, cmp> Q(&vertex_queue_positions);
  for (size_t i = 0; i < g.num_v(); ++i)
    g.distance(i) = INFINITY; // cmath
  
  g.distance(from) = 0;
  Q.push(g.vertex(from));
  
  g.visit(from); // !!!
  
  sz u, v;
  double w;
  
  while(!Q.empty())
    {
      // extract min
      u = Q.top().id();
      Q.pop();
      
      if (u == to) break;
      
      // explore u's neighbours
      vector<sz> neighbours = g.neighbours(u);
      
      for (sz i = 0; i != neighbours.size(); ++i)
	{
	  v = g.vertex(neighbours[i]).id();
	  w = g.edgeWeight(u, i);
	  if (g.notVisited(v) && 
	      g.distance(u) + w < g.distance(v))
	    {
	      g.distance(v) = g.distance(u) + w;
	      prev[v] = u;
	      if (g.notVisited(v))
		{
		  g.visit(v);
		  Q.push(Vertex(v, g.distance(v)));
		}
	      else if (black.notVisited(v))
		Q.change_key(vertex_queue_positions[v], 
			     Vertex(v, g.distance(v)));
	    }
	}
      black.visit(u);
    }
  
  // cleaning up
  g.unvisitAll();
  
  return prev;
}

vector<sz> ShortestPath::aStar(Graph &g, sz from, sz to)
{
  Visited black(g.num_v());
  vector<double> f(g.num_v());
  g.unvisitAll();
  vector<sz> vertex_queue_positions(g.num_v(), 0);
  vector<sz> prev;
  for (size_t i = 0; i < g.num_v(); ++i)
    {
      f[i] = g.distance(i) = INFINITY; // cmath
      prev.push_back(i);
    }     
  g.distance(from) = 0;
  f[from] = g.heuristic(from, to);

  
  priority_queue<Vertex, cmp> Q(&vertex_queue_positions);
  Q.push(g.vertex(from));
  
  g.visit(from); // !!!
  
  sz u, v;
  double w;
  
  while(!Q.empty())
    {
      // extract min
      u = Q.top().id();
      Q.pop();
      
      if (u == to) break;
      
      // explore u's neighbours
      vector<sz> neighbours = g.neighbours(u);
      
      for (sz i = 0; i != neighbours.size(); ++i)
	{
	  v = g.vertex(neighbours[i]).id();
	  w = g.edgeWeight(u, i);
	  if (g.notVisited(v) && 
	      g.distance(u) + w < g.distance(v))
	    {
	      g.distance(v) = g.distance(u) + w;
	      prev[v] = u;
	      if (g.notVisited(v))
		{
		  g.visit(v);
		  Q.push(Vertex(v, g.distance(v)));
		}
	      else if (black.notVisited(v))
		Q.change_key(vertex_queue_positions[v], 
			     Vertex(v, g.distance(v)));
	    }
	}
      black.visit(u);
    }
  
  // cleaning up
  g.unvisitAll();
  
  return prev;
}

void ShortestPath::printPath(vector<sz> const& p, sz src, sz dst) 
{
  if(dst == src) 
    {
      std::cout << dst << " ";
      return; 
    }
  printPath(p, src, p[dst]);
  std::cout << dst << " ";
}
