#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <cstddef> /* size_t */
#include <cmath>   /* sqrt() */


using std::vector;

typedef size_t sz;
typedef vector< vector<sz>     > adj_vect;
typedef vector< vector<double> > Weights;


struct Vertex
{
Vertex(sz id = 0, long x = 0, long y = 0)
    : id_(id)
    , x_(x)
    , y_(y)
  {}

  sz  id() const { return id_; }
  long x() const { return x_; }
  long y() const { return y_; }

  void setX(long x) { x_ = x; }
  void setY(long y) { y_ = y; }

private:
  sz id_;

  long x_;
  long y_;
};

class Graph
{
 public:
  Graph(char const* coordinates, char const* graph);
  
  void addEdge(sz u, sz v);
  void addWeight(sz from, double weight);

  double& distance(sz id);

  Vertex     const& vertex    (sz id) const;
  vector<sz> const& neighbours(sz id) const;
  double     const& edgeWeight(sz from, sz neighbour_num) const;
  
  sz num_v() const { return num_v_; }
  sz num_e() const { return num_e_; }

 private:
  sz num_v_;
  sz num_e_;

  vector<Vertex> vertices_;
  adj_vect adj_;
  Weights weights_;

 private:
  Graph& operator=(Graph const&);
  Graph(Graph const&); 
};



class Distance
{
 public:

  static double euclidianDistance(Vertex const &v1, 
                                  Vertex const &v2)
  {
    return sqrt(sqr(v1.x() - v2.x()) + sqr(v1.y() - v2.y()));
  }
  
  static double sqr(double const &x)
  {
    return x * x;
  }
};

 
#endif /* GRAPH_H_ */
