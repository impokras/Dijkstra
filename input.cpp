#include "input.h"

Input::Input(std::string fileName, bool dma)
{
    map.diagonal_moves_allowed = dma;
    if(fileName != "")
    {
        loadInput(fileName);
        return;
    }
    start.x = 0;
    start.y = 0;
    goal.x = 5;
    goal.y = 5;
    map.height = 6;
    map.width = 6;
    map.elements.resize(6, std::vector<int> (6, 0));
    map.elements[3][3] = 1;
}

double Grid::getCost(Node current, Node neighbor)
{
    if ((current.x != neighbor.x) && (current.y != neighbor.y))
        return sqrt(2);
    else
        return 1;
}
std::list<Node> Grid::getValidMoves(Node current)
{
    std::list<Node> result;
    std::list<std::pair<int, int>> d1 = {{0 ,1}, {1, 0}, {-1, 0}, {0, -1}};
    for(auto d:d1)
        if(current.x + d.first < width-1 && current.x + d.first >= 0 && current.y + d.second < height-1 && current.y + d.second >= 0)
            if(elements[current.x + d.first][current.y + d.second] == 0)
                result.push_back(Node(current.x + d.first, current.y + d.second));
    if(diagonal_moves_allowed)
    {
        std::list<std::pair<int, int>> d2 = {{1, 1}, {-1, -1}, {1, -1},  {-1, 1}}; // дельты для диагонального перехода
        for(auto d:d2)
            if(current.x + d.first < width-1 && current.x + d.first >= 0 && current.y + d.second < height-1 && current.y + d.second >= 0) // проверка краёв
                if((elements[current.x + d.first][current.y + d.second] == 0)   // проверка наличия препятствий при диагональном переходе
                        && (elements[current.x + d.first][current.y] == 0)      // проверка наличия препятствий при переходе по x
                        && (elements[current.x][current.y + d.second] == 0))    // проверка наличия препятствий при переходе по y
                {
                    result.push_back(Node(current.x + d.first, current.y + d.second));
                }
    }
    return result; // массив со всеми вершинами
}

void Input::loadInput(std::string fileName)
{
    std::ifstream in;
    in.open(fileName);
    if(!in.is_open())
    {
        std::cout<<"Error! Cannot load the file "<<fileName<<"\n";
        return;
    }
    char c;
    in>>map.height>>map.width;
    map.elements.resize(map.height, std::vector<int>(map.width, 0));
    for(int k=0; k<map.height; k++)
        for(int l=0; l<map.width; l++)
        {
            in>>c;
            if(c == '#')
                map.elements[k][l]=1;
        }
    in>>start.x>>start.y>>goal.x>>goal.y>>true_cardinal_cost>>true_diagonal_cost;
}
