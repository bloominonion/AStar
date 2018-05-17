/*
   AStarTest.cpp

   Implementation of a simple test for AStar algorithm
*/

#include <AStarPathfinding.h>

int main()
{
   using AStar::Node;
   using AStar::Point;

   std::vector<Point> points {
      {0,0,0},
      {0,5,0},
      {5,5,0},
      {10,5,0},
      {10,10,0},
   };

   AStar::DistanceFunction distFunc = 
      [&points](const Node&a, const Node& b) -> double
      {
         if (a.Id < 0 || b.Id < 0)
         {
            return std::numeric_limits<double>::infinity();
         }
         return points[a.Id].distance(points[b.Id]);
      };

   std::vector<Node> nodes(5, &distFunc);

   int i = 0;
   for (auto& node : nodes)
   {
      node.Id = i;
      node.Neighbors.push_back(&nodes[(i+1) % nodes.size()]);
      i++;
   }

   std::cout << nodes[0].Distance(nodes[1]) << std::endl;

   auto path = AStar::FindPath(nodes[0], nodes[nodes.size()-1]);
   if (path.size() > 0)
   {
      std::cout << "Solution found:" << std::endl;
   }
   for (auto& node : path)
   {
      auto& pt = points[node->Id];
      std::cout << "   X: " << pt.x << " Y: " << pt.y << " Z: " << pt.z << std::endl;
   }
   return 0;
}