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
    
    sz u, v;
    double w;
    
    while(!Q.empty())
    {
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
  
    sz u, v;
    double w;
  
    while(!Q.empty())
    {
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


void ShortestPath::printPath(sz src, sz dst) 
{
    if(src == dst) 
    {
        std::cout << dst << " ";
        return; 
    }

    if (prev_[dst])
        printPath(src, prev_[dst].get());
    std::cout << dst << " ";
}

