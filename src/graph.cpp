#include "../include/graph.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <limits>
#include <cassert>


long int const LONG_MAX = 
  std::numeric_limits<long int>::max();


Graph::Graph(char const* coordinates, char const* graph)
{
  
  std::ifstream co(coordinates);
  std::string line;
  std::string dummy = "";
  
  sz id = 0;
  long int x_min = LONG_MAX;
  long int y_min = LONG_MAX;

  
  while (std::getline(co, line) && line[0] != 'p')
    ;
  /* p aux sp co num_v */
  {
    std::istringstream iss(line);

    iss >> dummy >> dummy >> dummy >> dummy
	>> num_v_;
  }
  while (std::getline(co, line) && line[0] != 'v')
    ;
  /* v id x y */
  long int x, y;
  {
    std::istringstream iss(line);
    iss >> dummy >> id >> x >> y;
    x_min = x;
    y_min = y;
    vertices_.push_back(Vertex(id - 1, x, y)); 
  }

  while (co >> dummy >> id >> x >> y)
   {
      vertices_.push_back(Vertex(id - 1, x, y));
      
      if (x < x_min)
	x_min = x;
      if (y < y_min)
	y_min = y;    
   }
  
  assert(num_v_ == id);

  vector<Vertex>::iterator vit;
  for (vit = vertices_.begin(); vit != vertices_.end(); ++vit)
    {
      (*vit).setX((*vit).x() - x_min);
      (*vit).setY((*vit).y() - y_min);
    }

  co.close();
  assert(num_v_ == vertices_.size());

   
  std::ifstream gr(graph);
  
  while (std::getline(gr, line) && line[0] != 'p')
    ;
  /* p sp num_v num_e */
  {
    std::istringstream iss(line);
    iss >> dummy >> dummy >> num_v_ >> num_e_;
    assert(num_v_ == vertices_.size());
  }
  
  while (std::getline(gr, line) && line[0] != 'a')
    ;
  /* a from to weight */
  sz u, v;
  {
    std::istringstream iss(line);
    iss >> dummy >> u >> v >> dummy;
    addEdge(u - 1, v - 1);
    addWeight(u - 1, 
    Distance::euclidianDistance(vertex(u - 1), vertex(v - 1)));
  }

  while (gr >> dummy >> u >> v >> dummy)
    {
      Vertex from = vertex(u - 1);
      Vertex to   = vertex(v - 1);
      addEdge(u - 1, v - 1);
      addWeight(u - 1, Distance::euclidianDistance(from, to));   
    }
}

Vertex const& Graph::vertex(sz id) const
{
  return vertices_[id];
}

vector<sz> const& Graph::neighbours(sz id) const
{
  return adj_[id];
}

void Graph::addEdge(sz u, sz v)
{
  /* lazy add */
  if (adj_.size() <= u)
    for (sz i = adj_.size(); i <= u; ++i)
      adj_.push_back(vector<sz>());
  adj_[u].push_back(v);   
}

void Graph::addWeight(sz u, double weight)
{
  if (weights_.size() <= u)
    for (sz i = weights_.size(); i <= u; ++i)
      weights_.push_back(vector<double>());
  weights_[u].push_back(weight);   
}

double const& Graph::edgeWeight(sz from, sz nnum) const
{
  return weights_[from][nnum];
}

