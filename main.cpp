#include "algorithm.h"
#include <iostream>

using std::cin;
using std::cout;
using std::endl;

void usage()
{
    std::cerr
            << "Usage: ./main USA-road-d.***.co USA-road-d.***.gr"
            << endl;
}

void runDijkstra(Graph &g)
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

    cout << "Running Dijkstra..." << endl;

    ShortestPath d(from, to, g);

    double res = d.dijkstra();

    d.printPath(from, to);

    cout << endl << "Weight: " << res << endl;

}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        usage();
        return 1;
    }

    cout << "Reading graph..." << endl;
    Graph g(argv[1], argv[2]);
    cout << "Vertices: " << g.num_v() << endl
         << "Edges: " << g.num_e() << endl;

    cout << "First vertex coordinates: " << endl;
    cout << "X: "
         << g.vertex(0).x()
         << " Y: "
         << g.vertex(0).y()
         << std::endl;

    cout << "First vertex neighbours: " << endl;
    for (sz i = 0; i < g.neighbours(0).size(); ++i)
        cout << g.neighbours(0)[i]
                << " with weight "
                << g.edgeWeight(0, i)
                << endl;



    runDijkstra(g);

    return 0;
}
