#include "../include/algorithm.hpp"
#include <iostream>
#include <string>
#include <ctime>

using std::cin;
using std::cout;
using std::endl;

void option()
{
    cout << endl << "OPTION is:"
	 << endl << "    0 - exit"
	 << endl << "    1 - Dijkstra (default)"
	 << endl << "    2 - A*"
	 << endl << "    3 - BiDijkstra"
	 << endl << "    4 - ..."
	 << endl;
}

void usage()
{
    cout << "Usage: ./main "
	 << "INPUT/USA-road-d.***.co "
	 << "INPUT/USA-road-d.***.gr "
	 << "[OPTION]";
    option();
}

/* Unix */
class Timer
{
public:
    Timer() { clock_gettime(CLOCK_REALTIME, &beg_); }

    double elapsed() {
        clock_gettime(CLOCK_REALTIME, &end_);
        return (end_.tv_sec - beg_.tv_sec) * 1000 +
            (end_.tv_nsec - beg_.tv_nsec) / 1000000.;
    }
    
    void reset() { clock_gettime(CLOCK_REALTIME, &beg_); }
    
private:
    timespec beg_, end_;
};



void runAlgorithm(Graph &g, int OPTION, Timer & tmr)
{
    while (OPTION)
    {
	sz from = 0;
	sz to = 0;
	
	cout << "Enter source vertex from 0 to "
	     << g.num_v() - 1
	     << ":"
	     << endl;

	cin >> from;
	if (from >= g.num_v())
	    return;
	
	cout << "Enter destination vertex (same range)"
	     << ":"
	     << endl;
	
	cin >> to;
	if (to >= g.num_v())
	    return;
	
	tmr.reset();	
	ShortestPath d(from, to, g);
	double weight = 0;
	std::string msg = "Running ";
      

	switch (OPTION) 
	{
	case 1:
	    weight = d.dijkstra();
	    msg += "Dijkstra...";
	    break;
	case 2:
	    weight = d.aStar();
	    msg += "A*...";
	    break;
	case 3:
	    weight = d.biDijkstra();
	    msg += "Bi-Dijkstra...";
	    break;
	case 4:
	    cout << "Not implemented yet..." << endl;
	    return;
	default:
	    return;
        }
	
	cout << msg << endl;	
	d.printPath(from, to);
	d.printStatistics();

	cout << endl 
	     << "Weight: " 
	     << weight 
	     << endl;

	double t = tmr.elapsed();
	cout << "Elapsed time: "
	     << t 
	     << "ms"
	     << endl;
    
	cout << endl 
	     << "Choose OPTION:";
	
	option();
	cin >> OPTION;
    }
}

int main(int argc, char** argv)
{
    std::ios_base::sync_with_stdio(false);

    if (argc < 3)
    {
        usage();
        return 1;
    }
    
    int OPTION = 1;
    Timer tmr;
    
    cout << "Reading graph..." << endl;

    Graph g(argv[1], argv[2]);

    cout << "Vertices: " << g.num_v() << endl
         << "Edges: "    << g.num_e() << endl;

    double t = tmr.elapsed();
    cout << "Elapsed time: "
	 << t 
	 << "ms"
	 << endl;
    
    if (argc == 4)
      OPTION = argv[3][0] - '0';

    runAlgorithm(g, OPTION, tmr);
    
    return 0;
}
