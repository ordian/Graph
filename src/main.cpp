#include "../include/algorithm.hpp"
#include <iostream>
#include <string>

using std::cin;
using std::cout;
using std::endl;

void option()
{
    cout << "OPTION is:"
	 << endl << "0 - exit"
	 << endl << "1 - Dijkstra (default)"
	 << endl << "2 - A*"
	 << endl << "3 - BiDijkstra"
	 << endl << "4 - BiA*"
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

int runAlgorithm(Graph &g, int OPTION)
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
	    return 1;
	
	cout << "Enter destination vertex (same range)"
	     << ":"
	     << endl;
	
	cin >> to;
	if (to >= g.num_v())
	    return 1;
	
    
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
          return 1;
      default:
          return 1;
        }
      
      cout << msg << endl;

      d.printPath(from, to);

      cout << endl << "Weight: " << weight << endl;
      
      cout << endl << "Choose OPTION:";
      option();
      cin >> OPTION;
  }
  return 0;
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

    cout << "Reading graph..." << endl;
    Graph g(argv[1], argv[2]);
    cout << "Vertices: " << g.num_v() << endl
         << "Edges: " << g.num_e() << endl;

    if (argc == 4)
      OPTION = argv[3][0] - '0';

    return runAlgorithm(g, OPTION);
}
