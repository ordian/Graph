#include "graph.hpp"
#include <boost/optional.hpp>
double const EPS = 1e-9;

struct Node
{
	Node(sz id, double distance)
		: id_(id)
		, distance_(distance)
	{}

	sz id() const 
	{ 
		return id_; 
	}

	double const & distance() const 
	{ 
		return distance_; 
	}

	double & distance()       
	{ 
		return distance_; 
	}

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
		, prev_(g.num_v(), boost::optional<sz>())
	{}
	 
	struct Comparator 
	{
		bool operator() (Node const &u, Node const &v) 
		{
			return u.distance() + EPS < v.distance();
		}
	};

	
	double dijkstra(); 
	double aStar();
		 
	void printPath(sz src, sz dst);

 private:
	sz from_;
	sz to_;
	Graph & graph_;
	vector<boost::optional<sz> > prev_;
	
 private:
	ShortestPath const& 
		operator=(ShortestPath const&);

	ShortestPath(ShortestPath const&);
};
