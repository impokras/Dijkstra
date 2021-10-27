#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "input.h"
#include <chrono>

class Dijkstra
{
    ClosedList closed;
    OpenList open;
public:
    Dijkstra() {}
    SearchResult findPath(Input input);
    std::list<Node> reconstructPath(Node current);
};

#endif // DIJKSTRA_H
