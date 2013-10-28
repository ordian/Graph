#include "graph.h"
#include <boost/algorithm/string/predicate.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <cassert>

double Graph::heuristic(sz from, sz to) const
{
  return Distance::euclidianDistance(vertex(from).x(), vertex(from).y(),
				     vertex(to).x(), vertex(to).y());
}

bool Visited::getBit(sz bit)
{
  return static_cast<bool>(visit_mask_ & (1 << bit));
}

void Visited::setBit(sz bit)
{
  visit_mask_ |= 1 << bit;
}

bool Visited::notVisited(sz id)
{
  // if (id < size_)
  return !getBit(id);
}

void Visited::visit(sz id)
{
  // if (id < size_)
  setBit(id);
}

void Visited::unvisitAll()
{
  visit_mask_ = 0;
}

Graph::Graph(char const* coordinates, char const* graph)
{
  // for better performance
  //std::ios_base::sync_with_stdio(false);

  std::ifstream co(coordinates);
  char const prefix_vertex[] = "v";
  std::string line;
  
  // find min x and y coordinates
  sz id = 0;
  long int x_min = 1000000000; // "infinity"
  long int y_min = 1000000000;

  while (std::getline(co, line))
    { 
      long int x, y;
      std::istringstream iss(line);
      if (boost::starts_with(line, prefix_vertex))
	{
	  iss.ignore(1, '1'); // "v"
	  if (!(iss >> id >> x >> y)) break; // Error
	  if (x < x_min)
	    x_min = x;
	  if (y < y_min)
	    y_min = y; 
	}
    }
  num_v_ = id;    
  // reopen coordinates
  co.close(); 
  co.open(coordinates);
 
 while (std::getline(co, line))
    {
      std::istringstream iss(line);
      long int x, y;
      if (boost::starts_with(line, prefix_vertex))
	{
	  iss.ignore(1, '1'); // "v"
	  if (!(iss >> id >> x >> y)) break; // Error
	  sz x_ = static_cast<sz>(x - x_min);
	  sz y_ = static_cast<sz>(y - y_min);
	  vertices_.push_back(Vertex(id - 1, 0.0, x_, y_)); // id - 1 !!!
	}
    }
  assert(num_v_ == id);
  assert(num_v_ == vertices_.size());
  co.close();
  
  
  // read adjacency list
  std::ifstream gr(graph);
  char const prefix_num_edges[] = "p sp";
  char const prefix_arc[] = "a";
  
  while (std::getline(gr, line))
    {
      std::istringstream iss(line);
      sz u, v;
      if (boost::starts_with(line, prefix_arc))
	{
	  iss.ignore(1, '1'); // "a"
	  if (!(iss >> u >> v >> id)) break; // Error
	  addEdge(u - 1, v - 1); // 0-based index
	  Vertex from = vertex(u - 1);
	  Vertex to   = vertex(v - 1);
	  addWeight(u - 1, Distance::euclidianDistance(from.x(), from.y(), to.x(), to.y())); 
	}
      else if (boost::starts_with(line, prefix_num_edges))
	{
	  iss.ignore(4, '1'); // "p sp"
	  iss >> u >> v;
	  assert(num_v_ == u);
	  num_e_ = v;
	}
    }
  visited_ = Visited(num_v_);
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
  // lazy add
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

void Graph::visit(sz id)
{
  visited_.visit(id);
}

bool Graph::notVisited(sz id)
{
  return visited_.notVisited(id);
}

void Graph::unvisitAll()
{
  visited_.unvisitAll();
}

double& Graph::distance(sz id)
{
  return vertices_[id].distance();
}
