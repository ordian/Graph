#include "graph.hpp"
#include <boost/optional.hpp>
#include <fstream>
#include <set>

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

/* for grid TNR */
struct LocalTransitNode
{
    LocalTransitNode(sz i = 0, double d = 0.0)
	: id(i)
	, distance(d)
    {}
    sz id;
    double distance;
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
	, numSelectedLandmarks_(num_landmarks)
	  //, localTransitNodes_(g.num_v())
    {
	//gridTNRPreprocess(128);
    }
   
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
    typedef void (ShortestPath::*Preprocessing)();
    typedef double (ShortestPath::*Heuristic)(Vertex const &, bool);
    
    void       ALTPreprocessing();
    void doNothingPreprocessing();
    double dijkstraHeuristic(Vertex const & V, bool forward = true);
    double    aStarHeuristic(Vertex const & V, bool forward = true);
    double  biAStarHeuristic(Vertex const & V, bool forward);
    double      ALTHeuristic(Vertex const & V, bool forward = true);
    double    biALTHeuristic(Vertex const & V, bool forward);

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

    // std::set<sz> dijkstraLocal(sz from, 
    // 			       long x_cell, 
    // 			       long y_cell,
    // 			       std::set<sz> & transits);
    // void gridTNRPreprocess(int grid = 256);

    void dijkstraAll(sz from);
    
    void printPath(sz src, sz dst) const;
    void printStatistics() const;
    statistics const & stats() { return statistics_; }
    void writeBMP(char const * filename) const;
    
    
    sz source()      const { return from_; }
    sz destination() const { return to_; }

    void setSource   (sz source) { from_ = source; }
    void setDestination(sz dest) { to_ = dest; }

private:
    sz from_;
    sz to_;
    Graph & graph_;
    vector< boost::optional<sz> > prev_;
    vector<char> visited;
    statistics statistics_;
    vector<double> dijkstra_;
    vector<landmark> ALTPreprocessing_;
    sz numSelectedLandmarks_;
    // vector<sz> transitNodes_;
    // vector< vector<LocalTransitNode> > localTransitNodes_;
    // vector< vector<double> > transitTable_;
    // vector< vector< boost::optional<sz> > > transitTablePrev_;
    

private:
    ShortestPath const& 
    operator=(ShortestPath const&);

    ShortestPath(ShortestPath const&);
};


bool choose();
sz find_center(Graph const&);
bool file_exists(const std::string&);
bool areClockwise(Vertex const&, Vertex const&);
  
