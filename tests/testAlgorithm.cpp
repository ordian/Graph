#include "../include/algorithm.hpp"
#include "../include/timer.hpp"
#include <cstdlib>
#include <string>
#include <iomanip>


#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE testAlgorithm
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#define timeIt(funcName)             \
    tmr.reset();		     \
    double funcName = sp.funcName(); \
    t = tmr.elapsed();		     
   

#define getrandom(min, max) \
    ((rand()%(int)(((max) + 1)-(min)))+ (min))


char const * CO = "INPUT/USA-road-d.NY.co";
char const * GR = "INPUT/USA-road-d.NY.gr";
size_t const NUM_V = 264346;
size_t const NUM_E = 733846;
int const NUM_RAND_TESTS = 100;

 
struct Fixture
{
    Fixture()
	: tmr()
	, stats()
	, g(CO, GR)
	, sp(0, 0, g)
    {	
	double t = tmr.elapsed();
	std::cout << "Read graph in "
		  << std::setprecision(10)
		  << t
		  << "ms"
		  << std::endl;
    }

    struct Stat
    {
	Stat(std::string name, 
	     ShortestPath::statistics st,
	     double t,
	     double d,
	     sz src,
	     sz dst)
	    : algorithm_name(name)
	    , statistics(st)
	    , running_time(t)
	    , distance(d)
	    , from(src)
	    , to(dst)
	{
	    srand(time(NULL));
	}
	std::string algorithm_name;
	ShortestPath::statistics statistics;
	double running_time;
	double distance;
	sz from;
	sz to;
	
	void print() const
	{
	    std::cout << algorithm_name
		      << ":" 
		      << std::endl
		      << "From " << from << " to " << to 
		      << std::endl
		      << "\tVisited: " << statistics.pushed 
		      << std::endl
		      << "\tMore than once: " << statistics.decreased 
		      << std::endl
		      << std::setprecision(10)
		      << "\tRunning time: " << running_time << "ms" 
		      << std::endl
		      << "\tDistance: " << distance 
		      << std::endl;
	}
    };

    void addStat(Stat stat)
    {
	stats.push_back(stat);
    }

    ~Fixture()
    {
	vector<Stat>::const_iterator it;
	for (it = stats.begin(); it != stats.end(); ++it)
	    (*it).print();
    }
    
    Timer tmr;
    vector<Stat> stats;
    Graph g;
    ShortestPath sp;
};


BOOST_FIXTURE_TEST_CASE(testGraph, Fixture)
{
    BOOST_CHECK_EQUAL(g.num_v(), NUM_V);
    BOOST_CHECK_EQUAL(g.num_e(), NUM_E);
}
 
BOOST_FIXTURE_TEST_CASE(testAlgorithm, Fixture)
{
    sp.setSource(10);
    sp.setDestination(100500);
    BOOST_CHECK_EQUAL(sp.source(), 10);
    BOOST_CHECK_EQUAL(sp.destination(), 100500);
}
 
BOOST_FIXTURE_TEST_CASE(testAlgorithmFar, Fixture)
{
    sz from = 0;
    sz to = NUM_V - 2;
    sp.setSource(from);
    sp.setDestination(to);
    double t;
    vector<double> times;
    timeIt(dijkstra);
    addStat(
	Stat("Dijkstra",
	     sp.stats(),
	     t,
	     dijkstra,
	     from, 
	     to)
	);
    timeIt(aStar);
    addStat(
	Stat("A *",
	     sp.stats(),
	     t,
	     aStar,
	     from, 
	     to)
	);
    timeIt(biDijkstra);
    addStat(
	Stat("Bi-Dijkstra",
	     sp.stats(),
	     t,
	     biDijkstra,
	     from, 
	     to)
	);
    timeIt(biAStar);
    addStat(
	Stat("bi-A*",
	     sp.stats(),
	     t,
	     biAStar,
	     from, 
	     to)
	);
    timeIt(ALT);
    addStat(
	Stat("ALT",
	     sp.stats(),
	     t,
	     ALT,
	     from, 
	     to)
	);
    timeIt(biALT);
    addStat(
	Stat("bi-ALT",
	     sp.stats(),
	     t,
	     biALT,
	     from, 
	     to)
	);
    BOOST_CHECK_CLOSE(dijkstra, aStar, EPS);
    BOOST_CHECK_CLOSE(biDijkstra, aStar, EPS);
    BOOST_CHECK_CLOSE(biAStar, aStar, EPS);
    BOOST_CHECK_CLOSE(biALT, aStar, EPS);
    BOOST_CHECK_CLOSE(ALT, aStar, EPS);
}

BOOST_FIXTURE_TEST_CASE(testAlgorithmClose, Fixture)
{
    sz from = 12;
    sz to = 20;
    sp.setSource(from);
    sp.setDestination(to);
    double t;
    vector<double> times;
    timeIt(dijkstra);
    addStat(
	Stat("Dijkstra",
	     sp.stats(),
	     t,
	     dijkstra,
	     from, 
	     to)
	);
    timeIt(aStar);
    addStat(
	Stat("A *",
	     sp.stats(),
	     t,
	     aStar,
	     from, 
	     to)
	);
    timeIt(biDijkstra);
    addStat(
	Stat("Bi-Dijkstra",
	     sp.stats(),
	     t,
	     biDijkstra,
	     from, 
	     to)
	);
    timeIt(biAStar);
    addStat(
	Stat("bi-A*",
	     sp.stats(),
	     t,
	     biAStar,
	     from, 
	     to)
	);
    
    timeIt(ALT);
    addStat(
	Stat("ALT",
	     sp.stats(),
	     t,
	     ALT,
	     from, 
	     to)
	);
    timeIt(biALT);
    addStat(
	Stat("bi-ALT",
	     sp.stats(),
	     t,
	     biALT,
	     from, 
	     to)
	);
    BOOST_CHECK_CLOSE(dijkstra, aStar, EPS);
    BOOST_CHECK_CLOSE(biDijkstra, aStar, EPS);
    BOOST_CHECK_CLOSE(biAStar, aStar, EPS);
    BOOST_CHECK_CLOSE(biALT, aStar, EPS);
    BOOST_CHECK_CLOSE(ALT, aStar, EPS);
}

BOOST_FIXTURE_TEST_CASE(testAlgorithmRandom, Fixture)
{
    for (int i = 0; i < NUM_RAND_TESTS; ++i)
    {
	sz from = getrandom(0, NUM_V - 1);
	sz to   = getrandom(0, NUM_V - 1);
	sp.setSource(from);
	sp.setDestination(to);
	double t;
	vector<double> times;
	timeIt(dijkstra);
	addStat(
	    Stat("Dijkstra",
		 sp.stats(),
		 t,
		 dijkstra,
		 from, 
		 to)
	    );
	timeIt(aStar);
	addStat(
	    Stat("A *",
		 sp.stats(),
		 t,
		 aStar,
		 from, 
		 to)
	    );
	timeIt(biDijkstra);
	addStat(
	    Stat("Bi-Dijkstra",
		 sp.stats(),
		 t,
		 biDijkstra,
		 from, 
		 to)
	    );
	timeIt(biAStar);
	addStat(
	    Stat("bi-A*",
		 sp.stats(),
		 t,
		 biAStar,
		 from, 
		 to)
	    );
	
	timeIt(ALT);
	addStat(
	    Stat("ALT",
		 sp.stats(),
		 t,
		 ALT,
		 from, 
		 to)
	    );
	timeIt(biALT);
	addStat(
	    Stat("bi-ALT",
		 sp.stats(),
		 t,
		 biALT,
		 from, 
		 to)
	    );
	BOOST_CHECK_CLOSE(dijkstra, aStar, EPS);
	BOOST_CHECK_CLOSE(biDijkstra, aStar, EPS);
	BOOST_CHECK_CLOSE(biAStar, aStar, EPS);
	BOOST_CHECK_CLOSE(biALT, aStar, EPS);
	BOOST_CHECK_CLOSE(ALT, aStar, EPS);
    }
}



BOOST_AUTO_TEST_CASE(testAlgorithmTiny)
{
    Graph g("INPUT/tiny.co", "INPUT/tiny.gr");
    sz from = 0;
    sz to   = 2;
    ShortestPath sp(from, to, g);
    double dijkstra   = sp.dijkstra();
    double aStar      = sp.aStar();
    double biDijkstra = sp.biDijkstra();
    double biAStar    = sp.biAStar();
    double ALT        = sp.ALT();
    double biALT      = sp.biALT();
    std::cout << "Tiny graph: from 1 to 3"
	      << std::endl
	      << "Dijkstra: " << dijkstra
	      << std::endl
	      << "A *: " << aStar
	      << std::endl
	      << "Bi-Dijkstra: " << biDijkstra
	      << std::endl
	      << "Bi-A *: " << biAStar
	      << std::endl
	      << "ALT: " << ALT
	      << std::endl
	      << "Bi-ALT: " << biALT
	      << std::endl;
    BOOST_CHECK_CLOSE(dijkstra, 12, EPS);
    BOOST_CHECK_CLOSE(dijkstra, aStar, EPS);
    BOOST_CHECK_CLOSE(biDijkstra, aStar, EPS);
    BOOST_CHECK_CLOSE(biAStar, aStar, EPS);
    BOOST_CHECK_CLOSE(biALT, aStar, EPS);
    BOOST_CHECK_CLOSE(ALT, aStar, EPS);
}

