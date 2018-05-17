#ifndef _ASTARPATHFINDING_H_
#define _ASTARPATHFINDING_H_
/*
   AStarPathfinding.h

   Implementation of the A* Pathfinding algorithm.

*/
#include <iostream>
#include <functional>
#include <algorithm>
#include <vector>
#include <memory>
#include <limits>
#include <cmath>
#include <set>
#include <unordered_set>

namespace AStar{

struct Point
{
   Point()
   {
      x = 0.0;
      y = 0.0;
      z = 0.0;
   }
   Point(double xx, double yy, double zz)
   {
      x = xx;
      y = yy;
      z = zz;
   }

   bool operator==(const Point& other) const
   {
      return (abs(x-other.x) + abs(y-other.y) + abs(z-other.z)) < 0.001;
   }

   double distance(const Point& other) const
   {
      return sqrt(pow((other.x-x), 2) 
                + pow((other.y-y), 2) 
                + pow((other.z-z), 2));
   }

   double x, y, z;
};

struct Node;
typedef std::function<double(const Node&, const Node&)> DistanceFunction;

struct Node
{
   Node()
   {
      Id = -1;
      PrevNode = nullptr;
      DistCallback = nullptr;
      Pcost = 0.0;
      Hcost = std::numeric_limits<double>::infinity();
      Gcost = 0.0;
      Walkable = true;
   }
   Node(DistanceFunction* dFunc)
   {
      Id = -1;
      PrevNode = nullptr;
      DistCallback = dFunc;
      Pcost = 0.0;
      Hcost = std::numeric_limits<double>::infinity();
      Gcost = 0.0;
      Walkable = true;
   }
   ~Node(){}
   bool operator<(const Node& other) const
   {
      return this->FCost() < other.FCost();
   }
   bool operator>(const Node& other) const
   {
      return this->FCost() > other.FCost();
   }

   bool operator==(const Node& other) const
   {
      return this->FCost() == other.FCost() &&
             this->Distance(other) < 0.001;
   }

   bool Is(const Node& other) const
   {
      return this == &other;
   }

   double Distance(const Node& other) const
   {
      if (DistCallback != nullptr)
      {
         return (*DistCallback)(*this, other);
      }
      return std::numeric_limits<double>::infinity();
   }

   double FCost() const
   {
      return Hcost + Gcost;
   }

   int Id;
   Node* PrevNode;
   std::vector<Node*> Neighbors;
   double Hcost; // AStar heuristic value (Cost of travel from this to end)
   double Gcost; // Cost from start to this node
   double Pcost; // Penalty cost
   bool Walkable;
   DistanceFunction* DistCallback;
};

bool SetContains(const std::set<Node*>& set, Node* node)
{
   return set.find(node) != set.end();
}

bool SetContains(const std::unordered_set<Node*>& set, Node* node)
{
   return set.find(node) != set.end();
}

std::vector<Node*> FindPath(Node& start, Node& end)
{
   std::set<Node*, std::greater<Node*>> openSet;
   std::unordered_set<Node*> closedSet;

   start.Gcost = 0.0;
   openSet.insert(&start);

   bool solutionFound = false;
   Node* curNode = nullptr;
   size_t nodesChecked = 0;
   while (!openSet.empty())
   {
      curNode = *std::prev(openSet.end());
      openSet.erase(std::prev(openSet.end()));
      closedSet.insert(curNode);

      if (curNode->Is(end))
      {
         solutionFound = true;
         break;
      }

      for (Node* neighbor : curNode->Neighbors)
      {
         nodesChecked++;
         // If node can't be traversed or is in closed set, skip it
         if (!neighbor->Walkable || SetContains(closedSet, neighbor))
         {
            continue;
         }

         double newCost = curNode->Gcost + curNode->Distance(*neighbor) + neighbor->Pcost;
         bool inOpenSet = openSet.find(neighbor) != openSet.end();
         if (newCost < neighbor->Gcost || !inOpenSet)
         {
            neighbor->Gcost = newCost;
            neighbor->Hcost = neighbor->Distance(end);
            neighbor->PrevNode = curNode;

            if (inOpenSet)
            {
               std::set<Node*>::iterator elem = openSet.find(neighbor);
               std::set<Node*>::iterator hint = elem;
               hint++;
               openSet.erase(elem);
               openSet.insert(hint, neighbor);
            }
            else
            {
               openSet.insert(neighbor);
            }

         }
      }
   }

   std::vector<Node*> res;
   if (solutionFound)
   {
      while (curNode->PrevNode != nullptr)
      {
         res.push_back(curNode);
         curNode = curNode->PrevNode;
      }
      std::reverse(res.begin(), res.end());
   }
   return res;
}


} // End namespace AStar
#endif //End include guard for _ASTARPATHFINDING_H_
