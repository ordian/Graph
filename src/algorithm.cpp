#include "../include/algorithm.hpp"
#include "../include/priority_queue.hpp"


double ShortestPath::dijkstra()
{
    sz n = graph_.num_v();
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
            
                if (distance[u] + w < distance[v])
                {
                    distance[v] = distance[u] + w;
                    prev_[v] = u;
                
                    if (!vertex_queue_positions[v])
                        Q.push(Node(v, distance[v]));
                
                    else 
                        Q.change_key(vertex_queue_positions[v],
                                     Node(v, distance[v]));
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
                
                if (real_distance[u] + w < real_distance[v])
                {
                    Vertex const &V = graph_.vertex(v);
                    double fromV = Distance::euclidianDistance(V, To);  
                    real_distance[v] = real_distance[u] + w;
                    distance[v] = real_distance[u] + w + fromV;
                    prev_[v] = u;
                    
                    if (!vertex_queue_positions[v])
                        Q.push(Node(v, distance[v]));
                    
                    else 
                        Q.change_key(vertex_queue_positions[v], 
                                     Node(v, distance[v]));
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
    return a < b;
}

double ShortestPath::biDijkstra()
{
    sz n = graph_.num_v();
    prev_ = vector<boost::optional<sz> >
        (n, boost::optional<sz>());
    vector<double> visited(n, '\0');

    vector<sz> forward_vertex_queue_positions(n, 0);
    vector<sz> backward_vertex_queue_positions(n, 0);

    priority_queue<Node, Comparator>
        forward_Q(&forward_vertex_queue_positions);

    priority_queue<Node, Comparator>
        backward_Q(&backward_vertex_queue_positions);

    vector<double>  forward_distance(n, INFINITY);
    vector<double>  backward_distance(n, INFINITY);
    forward_distance[from_] = 0.0;
    backward_distance[to_] = 0.0;

    forward_Q.push(Node(from_, 0.0));
    backward_Q.push(Node(to_, 0.0));

    visited[from_] = 'f';
    visited[to_]   = 'b';

    prev_[from_] = from_;

    sz u = 0, v = 0;
    double w;

    while(true)
    {	
	bool choise = choose(
	    forward_distance[forward_Q.top().id()], 
            backward_distance[backward_Q.top().id()]);

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
                    
                    if (forward_distance[u] + w < 
                        forward_distance[v])
                    {
                        forward_distance[v] = 
                            forward_distance[u] + w;
                        
			prev_[v] = u;
			
			if (!forward_vertex_queue_positions[v])
			    forward_Q.push(
				Node(v, forward_distance[v]));
			
			else 
			    forward_Q.change_key(
				forward_vertex_queue_positions[v] 
				, Node(v, forward_distance[v]));
		    }
		}
		else
		    continue;
	    else
		if (visited[neighbours[i]] != 'b')
		{
		    v = neighbours[i];
		    w = graph_.edgeWeight(u, i);
		    
		    if (backward_distance[u] + w < 
			backward_distance[v])
		    {
			backward_distance[v] = 
			    backward_distance[u] + w;
			
			prev_[u] = v;
			
			if (!backward_vertex_queue_positions[v])
			    backward_Q.push(
				Node(v, backward_distance[v]));
			
			else 
			    backward_Q.change_key(
				backward_vertex_queue_positions[v] 
				, Node(v, backward_distance[v]));
		    }
		}

	    /* end for */

	if (choise)
	    visited[u] = 'f';
	else
	    visited[u] = 'b';

    } /* end while */

    return forward_distance[u] + backward_distance[u];
}




void ShortestPath::printPath(sz src, sz dst) 
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

