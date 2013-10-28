#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <cstddef> // size_t
#include <cmath>   // sqrt()
using std::vector;

typedef size_t sz;
typedef vector< vector<sz>     > adj_vect;
typedef vector< vector<double> > Weights;

struct Vertex
{
Vertex(sz id = 0, double distance = 0.0, sz x = 0, sz y = 0)
    : id_(id)
    , distance_(distance)
    , x_(x)
    , y_(y)
  {}
  sz id() const { return id_; }
  // distance from initial vertex
  // for graph search algorithms
  double& distance() { return distance_; }
  double const& getDistance() const { return distance_; }
  
  
  sz x() const { return x_; }
  sz y() const { return y_; }
private:
  sz id_;
  double distance_;
  // coordinates
  sz x_;
  sz y_;
};

class Visited
{
 public:
 Visited(sz size = 0)
    : size_(size)
    , visit_mask_(0)
    {}
  bool notVisited(sz vertex);
  void visit(sz vertex);
  void unvisitAll();
  sz size();
 private:
  sz size_;
  sz visit_mask_;
  void setBit(sz bit);
  bool getBit(sz bit);
};

class Graph
{
 public:
  // graph construction
  Graph(char const* coordinates, char const* graph);
  
  void addEdge(sz u, sz v);
  void addWeight(sz from, double weight);
  
  void visit(sz id);
  bool notVisited(sz id);
  void unvisitAll();

  double& distance(sz id);
  double heuristic (sz from, sz to) const;

  Vertex const& vertex(sz id) const;
  vector<sz> const& neighbours(sz id) const;
  double const& edgeWeight(sz from, sz neighbour_num) const;
  
  sz num_v() const { return num_v_; }
  sz num_e() const { return num_e_; }
 private:
  sz num_v_;
  sz num_e_;

  vector<Vertex> vertices_;
  adj_vect adj_;
  Weights weights_;
  Visited visited_;

  Graph& operator=(Graph const&);
  Graph(Graph const&); 
};

// helper functions

class Distance
{
 public:

  static double euclidianDistance(sz x1, sz y1, sz x2, sz y2)
  {
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
  }
};

 
#endif // GRAPH_H_
