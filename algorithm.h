#include "graph.h"

struct Node
{
  Node(sz id, double distance)
    : id_(id)
    , distance_(distance)
  {}
  sz id() const { return id_; }

  double const & 
  distance() const { return distance_; }

  double & 
  distance() { return distance_; }

private:
  sz id_;
  double distance_;
};

class ShortestPath
{
 public:
 ShortestPath(sz from, sz to, Graph& g)
    : from_(from)
    , to_(to)
    , graph_(g)
    , prev_(g.num_v(), from)
    , visited_(g.num_v(), '\0')
    , distance_(g.num_v(), INFINITY)
  {
    for (sz i = 0; i < g.num_v(); ++i)
      nodes_.push_back(Node(g.vertex(i).id(), INFINITY));
  }
   

  struct AStarComparator 
  {
    AStarComparator(sz to, Graph &g)
    : to_(to)
    , g_(g)
    {}

    bool operator() (Node const &u, Node const &v) 
    {
      Vertex const &U  = g_.vertex(u.id());
      Vertex const &V  = g_.vertex(v.id());
      Vertex const &To = g_.vertex(to_);
      double fromU = Distance::euclidianDistance(U, To);
      double fromV = Distance::euclidianDistance(V, To);      
      return u.distance() + fromU < v.distance() + fromV;
    }

  private:
    sz     to_;
    Graph & g_;
  };
  
  struct DijkstraComparator 
  {
    DijkstraComparator() {}
    bool operator() (Node const &u, Node const &v) 
    {
      return u.distance() < v.distance();
    }
  };

  
  double dijkstra(); 
  double aStar();
     
  void printPath(sz src, sz dst);

 private:

  sz from_;
  sz to_;
  Graph & graph_;
  vector<sz> prev_;
  
  vector<char>   visited_;
  vector<double> distance_;
  vector<Node> nodes_;

 private:
  void unvisitAll();

 private:
  ShortestPath const& 
    operator=(ShortestPath const&);

  ShortestPath(ShortestPath const&);
};



  
