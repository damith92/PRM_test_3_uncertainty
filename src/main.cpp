

#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <tuple>
#include <algorithm>
#include <list>
#include <limits>
#include <set>
#include <tuple>
#include <map>
#include <fstream>
#include <sstream>
#include <regex>
#include <uxhw.h>

int INF = std::numeric_limits<int>::max();


// data

int NODES = 20; 
double RADIUS = 5;
double startX = 0.5; 
double startY = 1.0; 
int startId = 0;
double goalX = 90.0; 
double goalY = 14.0; 
int goalId = NODES;
double obsRadius = 1.0;

struct ObsCoordinate {
    double x;
    double y;
};


struct Node
{

    double x;
    double y;
    int id;
};

class Graph
{

private:
    int V;

    std::list<std::pair<double, int>> *adj;

public:
    std::vector<int> optimalNodes;
    Graph(int v) : V(v)
    {

        std::cout << "initialization of graph for A* ..." << std::endl;

        this->V = v;

        this->adj = new std::list<std::pair<double, int>>[this->V];
    }
    void addEdge(int vStart, int vEnd, double cost);
    void Astar(int vStart, int vGoal, std::vector<bool> visited, std::vector<double> &heuristic);
    void computeStarA(int vStart, int vGoal, std::vector<double> heuristic);
};

//---------------------------------------------------------------------------


double measureNodeDistance(Node a, Node b)
{

    return std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));
}

double measureNodeDistanceObs(Node a, ObsCoordinate b)
{

    return std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));
}

//--------------------------------------------------------------------------------

/*
void printGoalPath(std::vector<Node> nodes, std::vector<std::tuple<Node, Node>> knn_nodes, std::vector<int> optimalNodes)
{

    std::vector<double> optX;
    std::vector<double> optY;

    std::vector<double> nodeX;
    std::vector<double> nodeY;

    std::vector<double> x_knn;
    std::vector<double> y_knn;

    for (auto &ii : optimalNodes)
    {

        for (int jj = 0; jj < knn_nodes.size(); jj++)
        {
            Node a = std::get<0>(knn_nodes[jj]);
            if (ii == a.id)
            {

                optX.push_back(a.x);
                optY.push_back(a.y);
            }
        }
    }

    for (auto &ii : nodes)
    {
        nodeX.push_back(ii.x);
        nodeY.push_back(ii.y);
    }

    for (int ii = 0; ii < knn_nodes.size(); ii++)
    {

        Node a = std::get<0>(knn_nodes[ii]);
        Node b = std::get<1>(knn_nodes[ii]);

        x_knn.push_back(a.x);
        x_knn.push_back(b.x);
        y_knn.push_back(a.y);
        y_knn.push_back(b.y);
    }

    
}
*/
//-------------------------------------------------


bool checkCollisonWithObs(Node a, Node b, std::vector<ObsCoordinate> obscoordinates)
{
    bool collision = false;
    double distToObstacle;

    for (auto& obscoord : obscoordinates) {

        // Calculate the distance from the line segment to the obstacle center
        distToObstacle = abs((b.y - a.y) * obscoord.x - (b.x - a.x) * obscoord.y + b.x * a.y - b.y * a.x)
                            / measureNodeDistance(a, b);

        if (distToObstacle <= obsRadius) {
            collision = true;
            break;
        }

        
    }

    return collision;
}



//-------------------------------------------------

bool checkDistance(Node a, Node b)
{

    double dist = std::sqrt(std::pow((a.x - b.x), 2) + std::pow((a.y - b.y), 2));

    return dist <= RADIUS ? true : false;
}

//------------------------------------------------

std::vector<std::tuple<Node, Node>> KNN(std::vector<Node> nodes, std::vector<ObsCoordinate> obscoordinates)
{

    std::vector<std::tuple<Node, Node>> knn_nodes;

    for (int ii = 0; ii < nodes.size(); ii++)
    {

        for (int jj = 0; jj < nodes.size(); jj++)
        {

            if (ii != jj)
            {
                bool checkColl = checkCollisonWithObs(nodes[ii], nodes[jj], obscoordinates);
                if (checkDistance(nodes[ii], nodes[jj]) && checkColl != true)
                {

                    knn_nodes.push_back({nodes[ii], nodes[jj]});
                }
            }
        }
    }

    return knn_nodes;
}

//-------------------------------------------------

std::vector<Node> nodeObsExclusion(std::vector<ObsCoordinate> obscoordinates, std::vector<Node> vecNodes)
{

    std::vector<Node> vecNodesEx;

    bool col1;
    double obsdistance;

    for (auto &ii : vecNodes)
    {
        col1 = false;

        for (auto& obscoord : obscoordinates) {

            obsdistance = measureNodeDistanceObs(ii, obscoord);

            if (obsdistance <= obsRadius) 
            {
                col1 = true;
                break;
            }

           
        }

        if (col1 == false)
        {

            vecNodesEx.push_back(ii);
        }
    }

    return vecNodesEx;

}

//-------------------------------------------------

std::vector<Node> generateNodes(std::vector<ObsCoordinate> obscoordinates)
{

    std::vector<Node> vecNodes;
    std::vector<Node> vecNodesEx;
    std::random_device rd;
    std::mt19937 gen(rd());
    for (int ii = 1; ii < NODES + 1; ii++)
    {

        std::uniform_int_distribution<> dist(0, 200);

        Node node{node.x = dist(gen) / 10.0, node.y = dist(gen) / 10.0, node.id = ii};
        vecNodes.push_back(node);
    }
    Node nodeS{nodeS.x = startX, nodeS.y = startY, nodeS.id = startId};
    Node nodeG{nodeG.x = goalX, nodeG.y = goalY, nodeG.id = goalId};
    vecNodes.push_back(nodeS);
    vecNodes.push_back(nodeG);
    return nodeObsExclusion(obscoordinates, vecNodes);
}

//-------------------------------------------------------------------------------

void Graph::addEdge(int vStart, int vEnd, double cost)
{

    this->adj[vStart].push_back(std::make_pair(cost, vEnd));
}

//-------------------------------------------------------------------------------
/*
    g = min cost from start vertex to this vertex (cost is changing depending which vertexes have benn visited)
    h = heuristic (eg Manhattan distance) - constance
    (FX) f = g + h
    next step: min f

    */

void Graph::Astar(int vStart, int vGoal, std::vector<bool> visited, std::vector<double> &heuristic)
{

    std::vector<std::tuple<double, int, int>> path;
    std::vector<double> functionFX(V, INF);

    std::set<std::pair<double, int>> AStar_set;

    functionFX[vStart] = 0 + heuristic[vStart];

    AStar_set.insert(std::make_pair(functionFX[vStart], vStart));

    while ((*(AStar_set.begin())).second != vGoal)
    {

        std::pair<double, int> nodeMin = *(AStar_set.begin());

        int nodeGraph = nodeMin.second;

        AStar_set.erase(AStar_set.begin());

        visited[nodeGraph] = true;

        // traverse list for certein vertex

        for (auto i = adj[nodeMin.second].begin(); i != adj[nodeMin.second].end(); i++)
        {

            int nodeGraph_i = (*i).second;
            double nodeGraph_i_functionFX = (*i).first + heuristic[(*i).second];

            // check the cost - functionFX for each neighbors of current vertex
            if (visited[nodeGraph_i] != true)
            {

                if (functionFX[nodeGraph_i] > nodeGraph_i_functionFX)
                {
                    // Remove the current distance if it is in the set
                    if (functionFX[nodeGraph_i] != INF)
                    {
                        AStar_set.erase(AStar_set.find(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i)));
                    }

                    // Update the distance
                    functionFX[nodeGraph_i] = nodeGraph_i_functionFX;
                    AStar_set.insert(std::make_pair(functionFX[nodeGraph_i], nodeGraph_i));

                    path.push_back(std::make_tuple(functionFX[nodeGraph_i], nodeGraph_i, nodeGraph));
                }
            }
        }
    }

    //---------------------------------------------------------------------

    std::multiset<std::tuple<int, int>> init_mSet;
    init_mSet.insert(std::make_tuple(0, 0));
    std::vector<std::multiset<std::tuple<int, int>>> routePath(V, init_mSet);

    for (int pathV = 1; pathV < V; pathV++)
    {

        std::multiset<std::tuple<int, int>> to_routePath;

        for (auto &ii : path)
        {

            int vertexV = std::get<1>(ii);

            if (pathV == vertexV)
            {

                to_routePath.insert(std::make_tuple(std::get<0>(ii), std::get<2>(ii))); //  Path_FX : Xx_xX, previous :Xx_xX
            }
        }

        routePath[pathV] = to_routePath;
    }

    //---------------------------------------------------------------------

    int previous = vGoal;
    std::vector<int> optimalPath;
    optimalPath.push_back(vGoal);

    while (previous != 0)
    {

        std::set<std::tuple<int, int>> minFx;

        for (auto &ii : routePath[previous])
        {

            minFx.insert(std::make_tuple(std::get<0>(ii), std::get<1>(ii)));
        }

        auto it = minFx.begin();
        previous = std::get<1>(*it);

        int min_path_i = std::get<0>(*it);

        optimalPath.push_back(previous);
    }

    optimalNodes = optimalPath;
    //======= PRINT OPTIMAL PATH ===========

    std::cout << "Optimal A* path for given graph : " << std::endl;
    for (auto &ii : optimalPath)
    {

        std::cout << ii << "--";
    }

    std::cout << "\n";

    //======================================
}

//-------------------------------------------------------------------------------

void Graph::computeStarA(int vStart, int vGoal, std::vector<double> heuristic)
{

    std::vector<bool> visited(this->V, false);

    Graph::Astar(vStart, vGoal, visited, heuristic);
}

//-------------------------------------------------------------------------------



int main()
{

    std::vector<ObsCoordinate> obscoordinates;

    std::istringstream iss;
    int numPoints;

    // Open the file
    std::ifstream inputFile("scaled_turbines_xy.yaml");
    
    // Check if the file is opened successfully
    if (!inputFile.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
        return 1;
    }

    std::string line;
    std::regex regex("\\[(-?\\d+\\.\\d+), (-?\\d+\\.\\d+)\\]");

    // Skip the first two lines
    for (int i = 0; i < 2; ++i) {
        if (std::getline(inputFile, line)) {

            if(i == 1){

                iss.str(line) ;
	            iss >> numPoints;
            }
            
        }
        else {
            std::cerr << "Error: Insufficient lines in the file." << std::endl;
            inputFile.close();
            return 1;

        }
    }

    while (std::getline(inputFile, line)) {
        std::smatch match;
        if (std::regex_search(line, match, regex)) {
            ObsCoordinate obscoord;
            //obscoord.x = std::stof(match[1]);
            iss.clear();
	        iss.str(match[1]) ;
	        iss >> obscoord.x ;

            //obscoord.y = std::stof(match[2]);
            iss.clear();
	        iss.str(match[2]) ;
	        iss >> obscoord.y ;

            obscoordinates.push_back(obscoord);
        } else {
            std::cerr << "Error parsing coordinates from line: " << line << std::endl;
        }
    }

    // Close the file
    inputFile.close();

    
    std::cout << "Number of Points: " << numPoints << std::endl;

    // Print the read coordinates
    for (auto& coord : obscoordinates) {
        std::cout << "X: " << coord.x << " | Y: " << coord.y << std::endl;
    }
    

    std::vector<Node> nodes = generateNodes(obscoordinates);
    std::vector<std::tuple<Node, Node>> knn_nodes = KNN(nodes, obscoordinates);

    Node G_h{G_h.x = goalX, G_h.y = goalY, G_h.id = goalId};

    int astar_nodes = knn_nodes.size();
    std::vector<double> heuristic;

    Graph g(astar_nodes);

    for (int ii = 0; ii < knn_nodes.size(); ii++)
    {

        Node a = std::get<0>(knn_nodes[ii]);
        Node b = std::get<1>(knn_nodes[ii]);

        double dist = measureNodeDistance(a, b);

        dist = UxHwDoubleUniformDist(dist, 1.0);

        heuristic.push_back(measureNodeDistance(a, G_h));

        g.addEdge(a.id, b.id, dist);
    }

    g.computeStarA(startId, goalId, heuristic);

    for (auto &ii : g.optimalNodes)
    {

        std::cout << ii << " , ";
    }

    //printGoalPath(nodes, knn_nodes, g.optimalNodes);
}