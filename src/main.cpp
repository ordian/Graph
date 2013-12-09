#include "../include/algorithm.hpp"
#include "../include/timer.hpp"
#include <iostream>
#include <iomanip>
#include <string>


using std::cin;
using std::cout;
using std::endl;
using std::setprecision;

void option();
void usage();
void runAlgorithm(Graph&, int OPTION, Timer&);

void option()
{
    cout << endl << "OPTION is:"
	 << endl << "    0 - exit"
	 << endl << "    1 - Dijkstra (default)"
	 << endl << "    2 - A*"
	 << endl << "    3 - BiDijkstra"
	 << endl << "    4 - BiAStar"
	 << endl << "    5 - ALT"
	 << endl << "    6 - BiALT"
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


void runAlgorithm(Graph &g, int OPTION, Timer & tmr)
{
    ShortestPath d(0, 0, g);
    //char const * graphBMP = "OUTPUT/graph.bmp";
    char const * algorithmBMP = "OUTPUT/algorithm.bmp";
  
    //d.writeBMP(d.getVisited(), d.getPrev(), graphBMP);

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
	d.setSource(from);
	d.setDestination(to);
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
	    weight = d.biAStar();
	    msg += "BiAStar...";
	    break;
	case 5:
	    weight = d.ALT();
	    msg += "ALT...";
	    break;
	case 6:
	    weight = d.biALT();
	    msg += "BiALT...";
	    break;
	default:
	    return;
        }
	
	d.writeBMP(d.getVisited(), d.getPrev(), algorithmBMP);
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
	 << setprecision(10)
	 << t 
	 << "ms"
	 << endl;
    
    if (argc == 4)
      OPTION = argv[3][0] - '0';

    runAlgorithm(g, OPTION, tmr);
    
    return 0;
}
