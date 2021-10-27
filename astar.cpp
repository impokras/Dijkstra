#include "astar.h"
#include "dijkstra.h"

SearchResult AStar::findPath(Input input)
{
    auto t = std::chrono::high_resolution_clock::now();
    SearchResult result;
    input.start.g = 0;
    input.start.f = AStar::getHValue(input.start, input.goal, input.map.diagonal_moves_allowed);
    open.addNode(input.start);
    while (open.getSize() != 0)
    {
        Node current = open.getMin();
        cl.addClose(current);
        open.popMin();
        if ((current.x == input.goal.x) && (current.y == input.goal.y))
        {
            result.cost = current.f;
            result.pathfound = true;
            result.path = AStar::reconstructPath(current);
            break;
        }
        std::list<Node> neighbours = input.map.getValidMoves(current);
        for (auto neighbour : neighbours)
        {
            if (cl.inClose(neighbour.x, neighbour.y))
                continue;
            neighbour.g = current.g + input.map.getCost(neighbour, current);
            neighbour.f = neighbour.g + AStar::getHValue(neighbour, input.goal, input.map.diagonal_moves_allowed);
            neighbour.parent = cl.getPointer(current.x, current.y);
            open.addNode(neighbour);
        }
    }
    result.createdNodes = closed.getSize() + open.getSize();
    result.steps = closed.getSize();
    result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t).count();
    return result;
}
double AStar::getHValue(Node current, Node goal, bool dma)
{
    if (dma)
        return abs(abs(goal.x - current.x) - abs(goal.y - current.y)) + sqrt(2) * fmin(abs(goal.x - current.x), abs(goal.y - current.y));
    else
        return abs(goal.x - current.x) + abs(goal.y - current.y);;
}
std::list<Node> AStar::reconstructPath(Node current)
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

