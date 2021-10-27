#include "dijkstra.h"

SearchResult Dijkstra::findPath(Input input)
{
    auto t = std::chrono::high_resolution_clock::now();
    SearchResult result;
        input.start.g = 0;
    input.start.f = 0;
    open.addNode(input.start);
    while(open.getSize() > 0)
    {
        Node current = open.getMin();
        closed.addClose(current);
        open.popMin();
        if((current.x == input.goal.x) && (current.y == input.goal.y))
        {
            result.cost = current.g;
            result.pathfound = true;
            result.path = Dijkstra::reconstructPath(current);
            break;
        }
        std::list<Node> neighbours = input.map.getValidMoves(current);
        for (auto neignbour : neighbours)
        {
            if (closed.inClose(neignbour.x, neignbour.y))
                continue;
            neignbour.g = current.g + input.map.getCost(neignbour, current);
            neignbour.f = neignbour.g;
            neignbour.parent = closed.getPointer(current.x, current.y);
            open.addNode(neignbour);
        }
    }

    result.createdNodes = closed.getSize() + open.getSize();
    result.steps = closed.getSize();
    result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t).count();
    return result;
}

std::list<Node> Dijkstra::reconstructPath(Node current)
{
    std::list<Node> path;
    while(current.parent != nullptr)
    {
        path.push_front(current);
        current = *current.parent;
    }
    path.push_front(current);
    return path;
}
