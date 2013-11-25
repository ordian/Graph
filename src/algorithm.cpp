#include "../include/algorithm.hpp"
#include "../include/priority_queue.hpp"
#include <algorithm>

double ShortestPath::dijkstra()
{
    sz n = graph_.num_v();
    statistics_ = statistics();
    prev_ = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
    vector<double> visited(n, '\0');

    vector<sz> vertex_queue_positions(n, 0);
    priority_queue<Node, Comparator>
        Q(&vertex_queue_positions);

    vector<double>  distance(n, INFINITY);
    distance[from_] = 0.0;
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
            
                if (distance[u] + w + EPS < distance[v])
                {
                    distance[v] = distance[u] + w;
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

    return distance[to_];
}

double ShortestPath::aStar()
{
    Vertex const &To = graph_.vertex(to_);
    sz n = graph_.num_v();
    statistics_ = statistics();
    prev_ = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
    vector<double> visited(n, '\0');
    
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
                    double fromV = Distance::euclidianDistance(V, To);  
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

bool choose(double a, double b)
{
    static bool choise = false; 
    /* simply alternate */
    return choise = !choise;
    /* or choose the one with lower min. dist. */
    // return a < b;
}

struct edge
{
    edge(sz v_, sz i_)
	: u(v_)
	, i(i_)
    {}
    sz u;
    sz i;
};

/* ASSUME: weights are symmetric */
double ShortestPath::biDijkstra()
{
    sz n = graph_.num_v();
    statistics_ = statistics();

    vector<boost::optional<sz> > 
	backward_prev = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
   
    vector<edge> crossing_edges;

    vector<double> visited(n, '\0');
    
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

    forward_Q.push (Node(from_, 0.0));
    backward_Q.push(Node(to_, 0.0));

    visited[from_] = 'f';
    visited[to_]   = 'b';

    prev_[from_] = from_;
    backward_prev[to_]  = to_;

    sz u = 0, v = 0;
    double w;

    while(true)
    {	
	bool choise = choose(0, 0);
	    //forward_distance [forward_Q.top().id()], 
            //backward_distance[backward_Q.top().id()]);
	
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

        if (visited[u] == 'b' && choise ||
            visited[u] == 'f' && !choise) break;
       
        vector<sz> const &neighbours = graph_.neighbours(u);
        for (sz i = 0; i != neighbours.size(); ++i)
            if (choise)
                if (visited[neighbours[i]] != 'f')
                {
                    v = neighbours[i];
                    w = graph_.edgeWeight(u, i);
                    
                    if (forward_distance[u] + w + EPS < 
                        forward_distance[v])
                    {
                        forward_distance[v] = 
                            forward_distance[u] + w;
                        
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
		    crossing_edges.push_back(edge(u, i));
		}
	    else
		if (visited[neighbours[i]] != 'b')
		{
		    v = neighbours[i];
		    w = graph_.edgeWeight(u, i);
		    
		    if (backward_distance[u] + w + EPS < 
			backward_distance[v])
		    {
			backward_distance[v] = 
			    backward_distance[u] + w;
			
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
		    crossing_edges.push_back(edge(u, i));
		}

	    /* end for */

	if (choise)
	    visited[u] = 'f';
	else
	    visited[u] = 'b';

    } /* end while */
    
/* process crossing edges*/
    double cur_dist = forward_distance[u] + backward_distance[u];
    
    vector<edge>::const_iterator it;
    sz f_cross = u, b_cross = u; /* vertices of crossing edge */

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
	    forward_distance[v1] +  backward_distance[v2] + w;

	if (dist + EPS  <  cur_dist)
	{
	    ++statistics_.decreased;	    
	    cur_dist = dist;
	    prev_[v2] = v1;
	    backward_prev[v1] = v2;
	    f_cross = v1;
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
    std::cout << "    Visited nodes: "
	      << statistics_.pushed
	      << std::endl;
    std::cout << "    Visited more than once: "
	      << statistics_.decreased
	      << std::endl;
}
