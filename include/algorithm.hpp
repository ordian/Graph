#include "graph.hpp"
#include <boost/optional.hpp>
#include <fstream>

double const EPS = 1e-5;

struct Node
{
    Node(sz id, double distance)
	: id_(id)
	, distance_(distance)
    {}
    
    sz                   id() const { return id_; }    
    double const & distance() const { return distance_; }
    double &       distance()       { return distance_; }
    
private:
    sz           id_;
    double distance_;
};


/* for ALT preprocessing */
struct landmark
{
    landmark(sz i = 0, vector<double> d = vector<double>())
	: id(i)
	, distances(d)
    {}
    sz id;
    vector<double> distances;
};


class ShortestPath
{
public:
    ShortestPath(sz from, sz to, Graph& g, char ALT = 3, sz num_landmarks = 16)
	: from_(from)
	, to_(to)
	, graph_(g)
	, prev_(g.num_v(), boost::optional<sz>())
	, visited(g.num_v(), '\0')
	, statistics_()
	, dijkstra_(g.num_v(), INFINITY)
	, ALTPreprocessing_(ALTPreprocess(num_landmarks, ALT))
    {}
   
    struct Comparator 
    {
	bool operator() (Node const &u, Node const &v) 
	{
	    return u.distance() + EPS < v.distance();
	}
    };
    
    struct statistics 
    {
	statistics()
	    : pushed(0)
	    , decreased(0)
	{}
	sz pushed;
	sz decreased;
    };
    
    /* pointer to method */
    typedef vector<landmark> (ShortestPath::*Preprocessing)();
    typedef double (ShortestPath::*Heuristic)(Vertex const &, vector<landmark> const &, bool);
    
    vector<landmark> ALTPreprocessing();
    vector<landmark> doNothingPreprocessing();
    double dijkstraHeuristic(Vertex const &, vector<landmark> const &, bool);
    double    aStarHeuristic(Vertex const &, vector<landmark> const &, bool);
    double  biAStarHeuristic(Vertex const &, vector<landmark> const &, bool);
    double      ALTHeuristic(Vertex const &, vector<landmark> const &, bool);
    double    biALTHeuristic(Vertex const &, vector<landmark> const &, bool);

    double unidirectionalAlgorithm(Preprocessing, Heuristic);
    double bidirectionalAlgorithm (Preprocessing, Heuristic); 

    double dijkstra();
    double aStar();
    double biDijkstra();
    double biAStar();
    double   ALT();
    double biALT();

    vector<landmark>       ALTPreprocess(sz num_landmarks = 16, 
					 char OPTION = 3);
    vector<landmark> planarALTPreprocess(sz num_landmarks = 16);
    vector<landmark> randomALTPreprocess(sz num_landmarks = 16);
    vector<landmark>  avoidALTPreprocess(sz num_landmarks = 16);

    void dijkstraAll();
    
    void printPath(sz src, sz dst) const;
    void printStatistics() const;
    statistics const & stats() { return statistics_; }
    void writeBMP(vector<char> const & visited_, 
		  vector<boost::optional<sz> > const & prev,
		  char const * filename) const;
    vector<char> const & getVisited() const { return visited; }
    vector<boost::optional<sz> > const & getPrev() const { return prev_; }

    sz source()      const { return from_; }
    sz destination() const { return to_; }

    void setSource   (sz source) { from_ = source; }
    void setDestination(sz dest) { to_ = dest; }

private:
    sz from_;
    sz to_;
    Graph & graph_;
    vector<boost::optional<sz> > prev_;
    vector<char> visited;
    statistics statistics_;
    vector<double> dijkstra_;
    vector<landmark> ALTPreprocessing_;

private:
    ShortestPath const& 
    operator=(ShortestPath const&);

    ShortestPath(ShortestPath const&);
};


bool choose();
sz find_center(Graph const&);
bool file_exists(const std::string&);
bool areClockwise(Vertex const&, Vertex const&);
  
