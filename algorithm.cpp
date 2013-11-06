#include "algorithm.h"
#include "priority_queue.h"
#include <iostream>


void ShortestPath::unvisitAll()
{
  visited_.assign(graph_.num_v(), '\0');
}
   
double ShortestPath::dijkstra()
{
  sz n = graph_.num_v();
  vector<sz> vertex_queue_positions(n, 0);
  
  priority_queue<Node, DijkstraComparator> 
    Q(&vertex_queue_positions);
 
  distance_[from_] = 0.0;
  Q.push(Node(from_, 0.0));
  
  visited_[from_] = 'y';
  
  sz u, v;
  double w;
  
  while(!Q.empty())
    {

      u = Q.top().id();
      Q.pop();
      
      if (u == to_) break;
      
  
      vector<sz> const &neighbours = graph_.neighbours(u);
      for (sz i = 0; i != neighbours.size(); ++i)
	if (!visited_[neighbours[i]])
	  {
	    v = neighbours[i];
	    w = graph_.edgeWeight(u, i);
	    
	    if (distance_[u] + w < distance_[v])
	      {
		
		distance_[v] = distance_[u] + w;
		prev_[v] = u;
		
		if (!vertex_queue_positions[v])
		  Q.push(Node(v, distance_[v]));
		
		else 
		  Q.change_key(vertex_queue_positions[v], 
			       Node(v, distance_[v]));
	      }
	  }
      
      visited_[u] = 'y';
      
    }
    
  unvisitAll();
  return distance_[to_];
}


void ShortestPath::printPath(sz src, sz dst) 
{
  if(src == dst) 
    {
      std::cout << dst << " ";
      return; 
    }
  printPath(src, prev_[dst]);
  std::cout << dst << " ";
}

