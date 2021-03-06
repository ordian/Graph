#include "../include/algorithm.hpp"
#include "../include/priority_queue.hpp"
#include "../include/bitmap_image.hpp"
#include <algorithm>
#include <limits>
#include <sys/stat.h>
#include <cassert>
#include <cmath>
#include <string>
#include <sstream>
#include <cstdlib>
#include <ctime>
#include <stack>
#include <queue>
//#include "../include/timer.hpp"

 
#define getrandom(min, max) \
    ((rand()%(int)(((max) + 1)-(min)))+ (min))


/* for planar ALT */
long int const LONG_MAX = 
  std::numeric_limits<long int>::max();
long int const LONG_MIN = 
  std::numeric_limits<long int>::min();


/* edge is (u, graph.neighbours(u)[i]) */
struct edge
{
    edge(sz v_, sz i_)
	: u(v_)
	, i(i_)
    {}
    sz u;
    sz i;
};

/* bidirectional switch */
bool choose()
{
    static bool choise = false; 
    /* simply alternate */
    return choise = !choise;
}

double ShortestPath::unidirectionalAlgorithm(Preprocessing preprocessing, 
					     Heuristic     heuristic)
{    
    (this->*preprocessing)();

    sz n = graph_.num_v();
    statistics_ = statistics();
    prev_ = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
    visited = vector<char>(n, '\0');

    vector<sz> vertex_queue_positions(n, 0);
    priority_queue<Node, Comparator>
        Q(&vertex_queue_positions);
 
    vector<double>      distance(n, INFINITY);
    vector<double> real_distance(n, INFINITY);
    distance[from_] = 0.0;
    real_distance[from_] = 0.0;
    prev_[from_] = from_;
    
    Q.push(Node(from_, 0.0));
    visited[from_] = 'y';
    
    while(!Q.empty())
    {
	sz u, v;
	double w;

        u = Q.top().id();
        Q.pop();
        
        if (u == to_) break;
        
        vector<sz> const &neighbours = graph_.neighbours(u);
        for (sz i = 0; i != neighbours.size(); ++i)
            if (!visited[neighbours[i]])
            {
                v = neighbours[i];
                w = graph_.edgeWeight(u, i);
                
                if (real_distance[u] + w + EPS < real_distance[v])
                {
                    Vertex const &V = graph_.vertex(v);
                    double fromV = (this->*heuristic)(V, true);  
                    real_distance[v] = real_distance[u] + w;
                    distance[v] = real_distance[u] + w + fromV;
                    prev_[v] = u;
		    
                    if (!vertex_queue_positions[v])
		    {
                        Q.push(Node(v, distance[v]));
			++statistics_.pushed;
                    }
                    else 
		    {
                        Q.change_key(vertex_queue_positions[v], 
                                     Node(v, distance[v]));
			++statistics_.decreased;
		    }
                }
            } /* end for */      

        visited[u] = 'y';          

    } /* end while */
  
    return real_distance[to_];
}


double ShortestPath::bidirectionalAlgorithm(Preprocessing preprocessing, 
					    Heuristic     heuristic)
{
    (this->*preprocessing)();

    sz n = graph_.num_v();
    statistics_ = statistics();

    vector<boost::optional<sz> > 
	backward_prev = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
   
    vector<edge> crossing_edges;

    visited = vector<char>(n, '\0');
    
    vector<sz> forward_vertex_queue_positions (n, 0);
    vector<sz> backward_vertex_queue_positions(n, 0);
    
    priority_queue<Node, Comparator>
        forward_Q (&forward_vertex_queue_positions);

    priority_queue<Node, Comparator>
        backward_Q(&backward_vertex_queue_positions);

    vector<double>  forward_distance (n, INFINITY);
    vector<double>  backward_distance(n, INFINITY);
    forward_distance[from_] = 0.0;
    backward_distance[to_]  = 0.0;
    vector<double>  real_forward_distance (n, INFINITY);
    vector<double>  real_backward_distance(n, INFINITY);
    real_forward_distance[from_] = 0.0;
    real_backward_distance[to_]  = 0.0;

    forward_Q.push (Node(from_, 0.0));
    backward_Q.push(Node(to_, 0.0));

    visited[from_] = 'f';
    visited[to_]   = 'b';

    prev_[from_] = from_;
    backward_prev[to_]  = to_;

    sz u = 0, v = 0;
    double w;

    while(!(forward_Q.empty() || backward_Q.empty()))
    {	
	bool choise = choose();
	
        if (choise)
        {
            u = forward_Q.top().id();
            forward_Q.pop();
        }
        else
        {
            u = backward_Q.top().id();
            backward_Q.pop();
        }


	if ((visited[u] == 'b' && choise) ||
	    (visited[u] == 'f' && !choise)) break;
       
        vector<sz> const &neighbours = graph_.neighbours(u);
        for (sz i = 0; i != neighbours.size(); ++i)
            if (choise)
                if (!visited[neighbours[i]])
                {
                    v = neighbours[i];
                    w = graph_.edgeWeight(u, i);
                    
                    if (real_forward_distance[u] + w + EPS < 
                        real_forward_distance[v])
                    {
			Vertex const &V = graph_.vertex(v);
			double fromV = (this->*heuristic)(V, true);  
			real_forward_distance[v] = 
			    real_forward_distance[u] + w;
			forward_distance[v] = 
			    real_forward_distance[u] + w + fromV;
                        
			prev_[v] = u;
			
			if (!forward_vertex_queue_positions[v])
			{
			    forward_Q.push(
				Node(v, forward_distance[v]));
			    ++statistics_.pushed;
			}
			else 
			{
			    forward_Q.change_key(
				forward_vertex_queue_positions[v] 
				, Node(v, forward_distance[v]));
			    ++statistics_.decreased;
			}
		    }
		}
		else
		{
		    /* add v with i-th neighbour */
		    if (visited[neighbours[i]] == 'b')
			crossing_edges.push_back(edge(u, i));
		}
	    else
		if (!visited[neighbours[i]])
		{
		    v = neighbours[i];
		    w = graph_.edgeWeight(u, i);
		    
		    if (real_backward_distance[u] + w + EPS < 
			real_backward_distance[v])
		    {
			Vertex const &V = graph_.vertex(v);
			double fromV = (this->*heuristic)(V, false);  
			real_backward_distance[v] = 
			    real_backward_distance[u] + w;
			backward_distance[v] = 
			    real_backward_distance[u] + w + fromV;
			
			backward_prev[v] = u;
			
			if (!backward_vertex_queue_positions[v])
			{
			    backward_Q.push(
				Node(v, backward_distance[v]));
			    ++statistics_.pushed;
			}
			else
			{ 
			    backward_Q.change_key(
				backward_vertex_queue_positions[v] 
				, Node(v, backward_distance[v]));
			    ++statistics_.decreased;
			}
		    }
		}
		else
		{
		    /* add v with i-th neighbour */
		    if (visited[neighbours[i]] == 'f')
			crossing_edges.push_back(edge(u, i));
		}

	    /* end for */
	 
	if (choise)
	    visited[u] = 'f';
	else
	    visited[u] = 'b';

    } /* end while */
   
    
    /* process crossing edges*/
    double cur_dist = real_forward_distance[u] + real_backward_distance[u];
    
    vector<edge>::const_iterator it;
    sz b_cross = u; /* vertices of crossing edge */

    for (it = crossing_edges.begin(); it != crossing_edges.end(); ++it)
    {
    /* process edge (v1, v2) */
	
	sz v1 = (*it).u;
	sz i  = (*it).i;
	sz v2 = graph_.neighbours(v1)[i];
	w     = graph_.edgeWeight(v1, i);
	bool forward = (visited[v1] == 'f');
	
	/* v1 is forward, v2 is backward */
	if (!forward)
	    std::swap(v1, v2);

	double dist = 
	    real_forward_distance[v1] +  real_backward_distance[v2] + w;

	if (dist + EPS  <  cur_dist)
	{
	    ++statistics_.decreased;	    
	    cur_dist = dist;
	    prev_[v2] = v1;
	    backward_prev[v1] = v2;
	    b_cross = v2;
	}
    }
   
    while (b_cross != to_)
    {
	sz prev = b_cross;
	b_cross = backward_prev[b_cross].get();
	prev_[b_cross] = prev;	
    }
    
    return cur_dist;
}

/* store distances in dijkstra_ */
void ShortestPath::dijkstraAll(sz from)
{
    sz n = graph_.num_v();
    statistics_ = statistics();
    prev_ = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
    visited = vector<char>(n, '\0');

    vector<sz> vertex_queue_positions(n, 0);
    priority_queue<Node, Comparator>
        Q(&vertex_queue_positions);
 
    if (dijkstra_[from] != INFINITY)
	dijkstra_ = vector<double>(n, INFINITY);

    dijkstra_[from] = 0.0;
    prev_[from] = from;
    
    Q.push(Node(from, 0.0));
    visited[from] = 'y';

    while(!Q.empty())
    {
	sz u, v;
	double w;

        u = Q.top().id();
        Q.pop();       
        
        vector<sz> const &neighbours = graph_.neighbours(u);
        for (sz i = 0; i != neighbours.size(); ++i)
            if (!visited[neighbours[i]])
            {
                v = neighbours[i];
                w = graph_.edgeWeight(u, i);

                if (dijkstra_[u] + w + EPS < dijkstra_[v])
                {
		    dijkstra_[v] = dijkstra_[u] + w;
                    prev_[v] = u;
		    
                    if (!vertex_queue_positions[v])
		    {
                        Q.push(Node(v, dijkstra_[v]));
			++statistics_.pushed;
                    }
                    else 
		    {
                        Q.change_key(vertex_queue_positions[v], 
                                     Node(v, dijkstra_[v]));
			++statistics_.decreased;
		    }
                }
            } /* end for */      

        visited[u] = 'y';          

    } /* end while */
}

struct LandmarkComparator
{
    LandmarkComparator(sz from_, sz to_)
	: from(from_)
	, to(to_)
    {}

    bool operator() (landmark const& a, landmark const& b)
    {
	return (fabs(a.distances[from] - a.distances[to]) >
		fabs(b.distances[from] - b.distances[to]));
    }
    
    sz from;
    sz to;
};

/* BOTTLENECK */
void ShortestPath::ALTPreprocessing()
{
/*    Timer tmr;
    tmr.reset();
    std::sort(ALTPreprocessing_.begin(), 
	      ALTPreprocessing_.end(),
	      LandmarkComparator(from_, to_));
    
    // empirical optimum 
    numSelectedLandmarks_ = 4;
    std::cout << tmr.elapsed() << std::endl;
*/
}

void ShortestPath::doNothingPreprocessing()
{
    return;
}

vector<landmark> ShortestPath::avoidALTPreprocess(sz num_landmarks)
{
    srand(time(NULL));
    vector<landmark> preproc;
    
    sz n = graph_.num_v();
    vector<bool> has_child_landmark(n, false);

    /* first landmark */
    sz first = getrandom(0, n - 1);
    dijkstraAll(first);
    preproc.push_back(landmark(first, dijkstra_));
    has_child_landmark[first] = true;
    
    for (sz counter = 0; counter + 1 < num_landmarks; ++counter)
    {
	/* pick a root r at random */
	sz r = getrandom(0, n - 1);

	/* build the shortest path tree from r */
	dijkstraAll(r);
	vector<vector<sz> > tree(n);
	for (sz i = 0; i < n; ++i)
	    if (prev_[i]) /* should always be true */
		tree[prev_[i].get()].push_back(i);
	
	/* bfs */
	std::queue<sz> q;
	std::stack<sz> order;
	q.push(r);

	while (!q.empty())
	{
	    sz c = q.front();
	    order.push(c);
	    q.pop();
		    
	    for (vector<sz>::const_iterator it = tree[c].begin();
		 it != tree[c].end();
		 ++it)
		if ((*it) != c)
		    q.push(*it);
	}

	vector<double> LB(n, 0);
	vector<double> weight(n, 0);
	vector<double> size(n, 0);	
	sz w_max = r;
	double cur_max = 0;

	while (!order.empty())
	{
	    sz v = order.top();
	    order.pop();
	 	    
	    /* set LB */
	    for (sz i = 0; i < preproc.size(); ++i)
	    {
		double d = fabs(preproc[i].distances[r] - 
				preproc[i].distances[v]);
		if (LB[v] + EPS < d)
		    LB[v] = d;
		if (preproc[i].id == v)
		    has_child_landmark[v] = true;
	    }

	    /* set weight */
	    weight[v] = dijkstra_[v] - LB[v];
	    assert(weight[v] + EPS > 0);
	    
	    /* set size */
	    if (!has_child_landmark[v])
		for (sz i = 0; i < tree[v].size(); ++i)		
		{
		    if (!has_child_landmark[v])
		    {
			size[v] += size[tree[v][i]];
			size[v] += weight[tree[v][i]];
		    }

		    /* not a leaf */
		    if (size[tree[v][i]] < EPS &&
			(tree[tree[v][i]].size() || 
			 (has_child_landmark[tree[v][i]])))
		    {
			size[v] = 0;
			has_child_landmark[v] = true;
		    }
		    
		    if (cur_max + EPS < size[tree[v][i]])
		    {
			w_max = tree[v][i];
			cur_max = size[tree[v][i]];
		    }
		}
	    else
		size[v] = 0;
	}   

	/* starting at w_max, go down tree following the maximum-sized child */
	
	while (!tree[w_max].empty())
	{
	    double w = 0;
	    if (tree[w_max][0] == w_max)
		break;
	    sz cur_w_max = tree[w_max][0];
	    for (sz i = 0; i < tree[w_max].size(); ++i)
		if (w + EPS < size[tree[w_max][i]])
		{
		    w = size[tree[w_max][i]];
		    cur_w_max = tree[w_max][i];
		}
	    w_max = cur_w_max;
	}
	/* pick the leaf at the end of this path as the new landmark */
	dijkstraAll(w_max);
	preproc.push_back(landmark(w_max, dijkstra_));

    }

    assert(preproc.size() == num_landmarks);
    return preproc;
}

vector<landmark> ShortestPath::randomALTPreprocess(sz num_landmarks)
{
    srand(time(NULL));
    vector<landmark> preproc;
    sz l = 0;
    for (sz i = 0; i < num_landmarks; ++i)
    {
	l = getrandom(0, graph_.num_v() - 1);
	dijkstraAll(l);
	preproc.push_back(landmark(l, dijkstra_));
    }
    return preproc;
}

vector<landmark> ShortestPath::planarALTPreprocess(sz num_landmarks)
{
    vector<landmark> preproc;
    sz center = find_center(graph_);
    std::cout << "Found center: " << center << std::endl;
    /* find landmarks */
    
    Vertex const & Center = graph_.vertex(center);
    
    vector<sz>     far_away(num_landmarks);
    vector<double> cur_dist(num_landmarks, 0);
    
    dijkstraAll(center);
    
    double const PI = 3.141592653589793238463;
    
    vector<Vertex> sectors;
    double offset = 360.0 / num_landmarks;
    double angle  = offset;
    sectors.push_back(Vertex(0, 1, 0));
    for (size_t i = 1; i < num_landmarks; ++i, angle += offset) 
    {
	sectors.push_back(Vertex(0, 
				 1000 * cos(angle * PI / 180.0), 
				 1000 * sin(angle * PI / 180.0)));
	
    }
    sectors.push_back(Vertex(0, 1, 0));
    assert(fabs(angle - 360.0) < EPS);
    
    for (size_t i = 0; i < graph_.num_v(); ++i)
    {
	Vertex const & V = graph_.vertex(i);
	Vertex v = Vertex(0, V.x() - Center.x(), V.y() - Center.y()); 
	
	/* find sector v belongs to */	    
	sz sector = 0;
	for (size_t j = 0; j < num_landmarks; ++j)
	    if (!areClockwise(sectors[j], v) && 
		areClockwise(sectors[j + 1], v))
	    {
		sector = j;
		break;
	    }
	
	/* update far_away if needed */	
	double d = dijkstra_[i]; 
        // or Distance::euclidianDistance(V, Center);
	
	if (d > EPS + cur_dist[sector])
	{
	    far_away[sector] = i;
	    cur_dist[sector] = d;
	}
    }
    for (size_t i = 0; i < far_away.size(); ++i)
    {
	dijkstraAll(far_away[i]);
	preproc.push_back(landmark(far_away[i], dijkstra_));
    }
    
    return preproc;
}

bool file_exists(const std::string& name) 
{
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

/* find id of vertex, closest to center */
sz find_center(Graph const &graph)
{
    /* find center of rectangle */
    sz n = graph.num_v();
    long x_min = LONG_MAX;
    long y_min = LONG_MAX;
    long x_max = LONG_MIN;
    long y_max = LONG_MIN;

    for (size_t i = 0; i < n; ++i)
    {
	Vertex const & v = graph.vertex(i);
	if (v.x() < x_min)
	    x_min = v.x();
	if (v.x() > x_max)
	    x_max = v.x();
	if (v.y() < y_min)
	    y_min = v.y();
	if (v.y() > y_max)
	    y_max = v.y();
    }

    assert(x_min == 0);
    assert(y_min == 0);

    long x_center = x_max / 2;
    long y_center = y_max / 2;
    Vertex center(0, x_center, y_center);

    /* find the closest point to center */
    sz cur_center = 0;
    double cur_distance = INFINITY;
    for (size_t i = 0; i < n; ++i)
    {
	Vertex const & v = graph.vertex(i);
	double d = Distance::euclidianDistance(v, center);
	if (d + EPS < cur_distance)
	{
	    cur_center = i;
	    cur_distance = d;
	}
    }
    return cur_center;
}

bool areClockwise(Vertex const & v1, Vertex const & v2) 
{
    return -v1.x() * v2.y() + v1.y() * v2.x() > 0;
}

/* NO PORTABILITY GUARANTEE */
vector<landmark> ShortestPath::ALTPreprocess(sz num_landmarks, char OPTION)
{
    std::ostringstream altfile;
    std::string name = "Avoid";
    if (OPTION == 1)
	name = "Planar";
    else 
	if (OPTION == 2)
	name = "Random";

    altfile << "preprocessing/" << name << "ALT_" << graph_.num_v() << ".txt";
    std::cout << "Start " << name << "ALT preprocessing..." << std::endl;
    std::string ALT(altfile.str());
    vector<landmark> preproc;
    bool recompute = false;

    /* read from file */
    if (file_exists(ALT))
    {
	std::cout << "Reading from file..." << std::endl;
	std::ifstream in(ALT.c_str(), std::ios::binary);
	sz size;
	in.read(reinterpret_cast<char*>(&size), sizeof(sz));

	if (size != num_landmarks)
	    recompute = true;
	else
	{   
	    
	    preproc.resize(num_landmarks);
	    
	    for (sz i = 0; i < size; ++i)
	    {
		sz d_size = 0;
		in.read(reinterpret_cast<char*>(
			    &d_size), sizeof(sz));

		in.read(reinterpret_cast<char*>(
			    &preproc[i].id), sizeof(sz));

		preproc[i].distances.resize(d_size);
		
		for (sz j = 0; j < d_size; ++j)
		    in.read(reinterpret_cast<char*>(
				&preproc[i].distances[j]), sizeof(double));
	    }
	}
    }
    else
	recompute = true;
    
    /* find center */
    if (recompute)
    {
	if (OPTION == 1)
	    preproc = planarALTPreprocess(num_landmarks);
	else
	    if (OPTION == 2)
		preproc = randomALTPreprocess(num_landmarks);
	    else
		preproc = avoidALTPreprocess(num_landmarks);

	/* write to file */
	std::ofstream out(ALT.c_str(), std::ios::binary);
	sz size = preproc.size();
	assert(size == num_landmarks);
	std::cout << "Writing to " 
		  << name
		  << "ALT_"
		  << graph_.num_v() 
		  << ".txt" 
		  << std::endl;
	out.write(reinterpret_cast<char*>(&size), sizeof(size));
	for (sz i = 0; i < size; ++i)
	{
	    sz d_size = preproc[i].distances.size();
	    out.write(reinterpret_cast<char*>(
			  &d_size), sizeof(sz));

	    out.write(reinterpret_cast<char*>(
			  &preproc[i].id), sizeof(sz));
	    
	    for (sz j = 0; j < preproc[i].distances.size(); ++j)
		out.write(reinterpret_cast<char*>(
			      &preproc[i].distances[j]), sizeof(double));
	}
    }

    std::cout << "End " << name << " ALT preprocessing!" << std::endl;
    
    return preproc;
}

double ShortestPath::dijkstraHeuristic(Vertex const &V,  
				       bool forward)
{
    return 0;
}

double ShortestPath::aStarHeuristic(Vertex const &V,
				    bool forward)
{
    Vertex const & From = graph_.vertex(from_);
    Vertex const & To   = graph_.vertex(to_);

    if (forward)
	return Distance::euclidianDistance(V, To);
    else
	return Distance::euclidianDistance(V, From);
}

double ShortestPath::biAStarHeuristic(Vertex const &V,
				      bool forward)
{
    Vertex const & From = graph_.vertex(from_);
    Vertex const & To   = graph_.vertex(to_);
    
    if (forward)
	return (Distance::euclidianDistance(V, To) -
		Distance::euclidianDistance(V, From)) / 2;
    else
	return (Distance::euclidianDistance(V, From) -
		Distance::euclidianDistance(V, To)) / 2;
}


double ShortestPath::ALTHeuristic(Vertex const &V,
				  bool forward)
{
    
    double max = 0;
    sz to = to_;
    if (!forward)	
	to = from_;

    for (sz it = 0;
	 it != numSelectedLandmarks_;
	 ++it)
    {
	double h = fabs(ALTPreprocessing_[it].distances[V.id()] - 
			ALTPreprocessing_[it].distances[to]);
	if (max + EPS < h)		
	    max = h;
    }

    return max;
}

double ShortestPath::biALTHeuristic(Vertex const &V,
				    bool forward)
{
    return (ALTHeuristic(V, forward) -
	    ALTHeuristic(V, !forward)) / 2;
}


double ShortestPath::dijkstra()
{
    return unidirectionalAlgorithm(&ShortestPath::doNothingPreprocessing, 
				   &ShortestPath::dijkstraHeuristic);
}

double ShortestPath::aStar()
{
    return unidirectionalAlgorithm(&ShortestPath::doNothingPreprocessing, 
				   &ShortestPath::aStarHeuristic);
}

double ShortestPath::biDijkstra()
{
    return bidirectionalAlgorithm(&ShortestPath::doNothingPreprocessing, 
				  &ShortestPath::dijkstraHeuristic);
}

double ShortestPath::biAStar()
{
    return bidirectionalAlgorithm(&ShortestPath::doNothingPreprocessing, 
				  &ShortestPath::biAStarHeuristic);
}

double ShortestPath::ALT()
{
    return unidirectionalAlgorithm(&ShortestPath::ALTPreprocessing, 
				   &ShortestPath::ALTHeuristic);
}

double ShortestPath::biALT()
{
    return bidirectionalAlgorithm(&ShortestPath::ALTPreprocessing, 
				  &ShortestPath::biALTHeuristic);
}

/* 
std::set<sz> ShortestPath::dijkstraLocal(sz from, 
					 long x_cell, 
					 long y_cell,
					 std::set<sz> & transitNodes)
{
    sz n = graph_.num_v();
    statistics_ = statistics();
    prev_ = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
    visited = vector<char>(n, '\0');
    
    std::set<sz> new_transits;

    vector<sz> vertex_queue_positions(n, 0);
    priority_queue<Node, Comparator>
        Q(&vertex_queue_positions);
    
    dijkstra_ = vector<double>(n, INFINITY);
    
    dijkstra_[from] = 0.0;
    prev_[from] = from;
    
    Q.push(Node(from, 0.0));
    visited[from] = 'y';

    long x_center_cell = graph_.vertex(from).x() / x_cell;
    long y_center_cell = graph_.vertex(from).y() / y_cell;
    
    while(!Q.empty())
    {
	sz u, v;
	double w;

        u = Q.top().id();
        Q.pop();

	long x_u_cell = std::abs(graph_.vertex(u).x() / x_cell 
				 - x_center_cell);
	long y_u_cell = std::abs(graph_.vertex(u).y() / y_cell 
				 - y_center_cell);


	if ((x_u_cell > 4) || (y_u_cell > 4))
	    break;
        
        vector<sz> const &neighbours = graph_.neighbours(u);
        for (sz i = 0; i != neighbours.size(); ++i)
            if (!visited[neighbours[i]])
            {
                v = neighbours[i];
                w = graph_.edgeWeight(u, i);

		
                if (dijkstra_[u] + w + EPS < dijkstra_[v])
                {
		    dijkstra_[v] = dijkstra_[u] + w;
                    prev_[v] = u;
		    
                    if (!vertex_queue_positions[v])
		    {
                        Q.push(Node(v, dijkstra_[v]));
			++statistics_.pushed;
                    }
                    else 
		    {
                        Q.change_key(vertex_queue_positions[v], 
                                     Node(v, dijkstra_[v]));
			++statistics_.decreased;
		    }
                }
            } 

        visited[u] = 'y';
	
	if (prev_[u])
	{
	    long x_v_cell = std::abs(graph_.vertex(prev_[u].get()).x() / x_cell 
				     - x_center_cell);
	    long y_v_cell = std::abs(graph_.vertex(prev_[u].get()).y() / y_cell 
				     - y_center_cell);
	
            //	cross boundaries 
	    if ((x_v_cell < 3 && x_u_cell > 2) ||
		(x_u_cell < 3 && x_v_cell > 2) ||
		(y_v_cell < 3 && y_u_cell > 2) ||
		(y_u_cell < 3 && y_v_cell > 2))
	    {
		new_transits.insert(std::min(u, prev_[u].get()));
		transitNodes.insert(std::min(u, prev_[u].get()));
	    }
	}


    } 
    
    return new_transits;
}
*/

/* 
void ShortestPath::gridTNRPreprocess(int grid)
{
    char const * transits_file  = "preprocessing/transitNodes.txt";
    char const * transits_table = "preprocessing/transitTable.txt";
    char const * transits_prev  = "preprocessing/transitTablePrev.txt";
    sz num_transits;
    
    if (!file_exists(transits_file))
    {
	sz n = graph_.num_v();
	long x_max = LONG_MIN;
	long y_max = LONG_MIN;
	
	for (size_t i = 0; i < n; ++i)
	{
	    Vertex const & v = graph_.vertex(i);
	    if (v.x() > x_max)
		x_max = v.x();
	    if (v.y() > y_max)
		y_max = v.y();
	}
	
	long x_cell = x_max / grid;
	long y_cell = y_max / grid;
	
	if (x_cell * grid < x_max)
	    ++x_cell;
	if (y_cell * grid < y_max)
	    ++y_cell;

	std::set<sz>::const_iterator it;
	std::set<sz> transitNodes;
	std::cout << "Find transit nodes..." << std::endl;
	for (sz i = 0; i < n; ++i)
	{
	    //std::cout << n - i << std::endl;
	    std::set<sz> nt = dijkstraLocal(i, x_cell, y_cell, transitNodes);
	    
	    for (it = nt.begin(); it != nt.end(); ++it)
	    {
		localTransitNodes_[i].push_back(
		    LocalTransitNode(*it, dijkstra_[*it])
		    );
	    }
	}
	std::cout << "Writing to file " << transits_file << std::endl;
	
	transitNodes_ = vector<sz>(transitNodes.begin(), transitNodes.end());
	num_transits = transitNodes_.size();
	std::cout << num_transits << " nodes" << std::endl;
	std::ofstream out(transits_file);
	out.write(reinterpret_cast<char*>(&num_transits), sizeof(sz));
	for (sz i = 0; i != transitNodes_.size(); ++i)
	    out.write(reinterpret_cast<char*>(&transitNodes_[i]), sizeof(sz));
    }
    else
    {

	std::cout << "Reading from file" << transits_file << std::endl;	


	std::ifstream in(transits_file);
	in.read(reinterpret_cast<char*>(&num_transits), sizeof(sz));
	transitNodes_.resize(num_transits);
	for (sz i = 0; i != num_transits; ++i)
	    in.read(reinterpret_cast<char*>(&transitNodes_[i]), sizeof(sz));

    }

    transitTable_.resize(num_transits);
    transitTablePrev_.resize(num_transits);
    for (sz i = 0; i < num_transits; ++i)
	transitTable_[i].resize(num_transits);
    
    if (!file_exists(transits_table) ||
	!file_exists(transits_prev))
    {
	std::cout << "Wait for a table construction..." << std::endl;

	
	for (sz i = 0; i < num_transits; ++i)
	{
	    std::cout << num_transits - i << std::endl;
	    dijkstraAll(transitNodes_[i]);
	    for (sz j = 0; j < num_transits; ++j)
	    {
		transitTable_[i][j] = dijkstra_[transitNodes_[j]];
	    }
	    transitTablePrev_[i] = prev_;
	}
	
	std::cout << "Writing to files..." << std::endl;
	std::ofstream out_table(transits_table);
	std::ofstream out_prev(transits_prev);
	for (sz i = 0; i < num_transits; ++i)
	    for (sz j = 0; j < num_transits; ++j)
		out_table.write(reinterpret_cast<char*>(&transitTable_[i][j]),
				sizeof(double));
	
	for (sz i = 0; i < num_transits; ++i)
	    out_prev.write(reinterpret_cast<char*>(&transitTablePrev_[i]),
			   sizeof(sz));
	
    }
    else
    {
	std::cout << "Reading from files..." << std::endl;
	std::ifstream in_table(transits_table);
	std::ifstream in_prev(transits_prev);
	for (sz i = 0; i < num_transits; ++i)
	    for (sz j = 0; j < num_transits; ++j)
		in_table.read(reinterpret_cast<char*>(&transitTable_[i][j]),
			      sizeof(double));
	
	for (sz i = 0; i < num_transits; ++i)
	    in_prev.read(reinterpret_cast<char*>(&transitTablePrev_[i]),
			 sizeof(sz));
	
    }
}
*/


void ShortestPath::printPath(sz src, sz dst) const
{
    if (src == dst)
    {
        std::cout << dst << " ";
        return;
    }

    if (prev_[dst])
        printPath(src, prev_[dst].get());
    std::cout << dst << " ";
}

void ShortestPath::printStatistics() const
{
    std::cout << std::endl
	      << "Statistics:" 
	      << std::endl;
    std::cout << "\tVisited nodes: "
	      << statistics_.pushed
	      << std::endl;
    std::cout << "\tVisited more than once: "
	      << statistics_.decreased
	      << std::endl;
}

void ShortestPath::writeBMP(char const * bmpfile) const
{
    int const BORDER = 20;
    int const HEIGHT = 3000 + 2 * BORDER;
    int const WIDTH  = 3000 + 2 * BORDER;
    
    bitmap_image image(WIDTH, HEIGHT);

    /* set background to white */
    image.set_all_channels(255, 255, 255);
   
    image_drawer draw(image);
   
    sz n = graph_.num_v();
    
    /* find scale */
    long int x_max = LONG_MIN, y_max = LONG_MIN;
    for (sz i = 0; i < n; ++i)
    {
	if (graph_.vertex(i).x() > x_max)
	    x_max = graph_.vertex(i).x();
	if (graph_.vertex(i).y() > y_max)
	    y_max = graph_.vertex(i).y();
    }
    double x_scale = x_max / (WIDTH  - 2 * BORDER);
    double y_scale = y_max / (HEIGHT - 2 * BORDER);
    
    /* vertices */
    assert(visited.size() == n);
    for (sz i = 0; i < n; ++i)
    {
	if (!visited[i])
	{
	    draw.pen_width(1);   
	    draw.pen_color(0, 0, 0);
	}
	else if (visited[i] == 'b')
	{
	    draw.pen_width(3);   
	    draw.pen_color(0, 0, 255);
	}
	else
	{
	    draw.pen_width(3);   
	    draw.pen_color(0, 255, 0);
	}

	draw.plot_pen_pixel(BORDER + graph_.vertex(i).x() / x_scale, 
			    BORDER + graph_.vertex(i).y() / y_scale);
    }
       
    /* edges */
    draw.pen_width(1);
    draw.pen_color(0, 0, 0);
    for (sz i = 0; i < n; ++i)
	for (vector<sz>::const_iterator it = graph_.neighbours(i).begin();
	     it != graph_.neighbours(i).end();
	     ++it)
	{
	    Vertex const & u = graph_.vertex(i);
	    Vertex const & v = graph_.vertex((*it));
	    draw.line_segment(BORDER + u.x() / x_scale, 
			      BORDER + u.y() / y_scale, 
			      BORDER + v.x() / x_scale, 
			      BORDER + v.y() / y_scale);   
	}
        
    /* shortest path */
    draw.pen_width(3);
    draw.pen_color(255, 0, 0);
    sz dst = to_;
    /* destination */
    
    draw.circle(BORDER + graph_.vertex(to_).x() / x_scale, 
		BORDER + graph_.vertex(to_).y() / y_scale, BORDER / 2);
    while (dst != from_)
    {
	Vertex const & u = graph_.vertex(dst);
	Vertex const & v = graph_.vertex(prev_[dst].get());
	draw.line_segment(BORDER + u.x() / x_scale, 
			  BORDER + u.y() / y_scale, 
			  BORDER + v.x() / x_scale, 
			  BORDER + v.y() / y_scale);   
	dst = prev_[dst].get();
    }
    /* source */
    
    draw.circle(BORDER + graph_.vertex(from_).x() / x_scale, 
		BORDER + graph_.vertex(from_).y() / y_scale, BORDER / 2);
    

    for (sz it = 0;
	 it != ALTPreprocessing_.size();
	 ++it)
    {
	draw.pen_color(0, 255, 0);
	if (it < numSelectedLandmarks_)
	    draw.pen_color(0, 255, 255);	
	
	draw.circle(BORDER + 
		    graph_.vertex(ALTPreprocessing_[it].id).x() / x_scale, 
		    BORDER + 
		    graph_.vertex(ALTPreprocessing_[it].id).y() / y_scale, 
		    BORDER / 2);
    }

    
    image.save_image(bmpfile);
}
