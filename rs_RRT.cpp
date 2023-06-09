//g++ cpp_RRT.cpp -o t -I/usr/include/python3.8 -lpython3.8
#include <random>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <limits>
#include <omp.h>

#include <chrono>

#define PLOT

#ifdef PLOT
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

using namespace std;
using namespace std::chrono;

int MAX_ITERATION = 16000;
double MAX_DISTANCE = .5;
double GOAL;
int THREADS = 4;

omp_lock_t writelock;

//obstacles for RRT
std::vector<double> obsXmin{3, 11, 25};
std::vector<double> obsYmin{3, 6, 20};
std::vector<double> obsXmax{7, 18,27};
std::vector<double> obsYmax{10, 17, 25};

//------------------------------------------------------------------------------

struct Node
{
    double posX;
    double posY;
    Node *prev;
    Node *next;
};

//------------------------------------------------------------------------------

typedef struct{
    Node *start;
    Node *goal;
    std::vector<Node *> rrtNodes;



} RRT;


void init_RRT(RRT& rrt, double startX, double startY, double goalX, double goalY)
{

    Node *node = new Node;
    rrt.start = node;
    rrt.start->posX = startX;
    rrt.start->posY = startY;
    rrt.start->prev = nullptr;
    rrt.rrtNodes.push_back(node);

    node = new Node;
    rrt.goal = node;
    rrt.goal->posX = goalX;
    rrt.goal->posY = goalY;
}

    //------------------------------------------------------------------------------

#ifdef PLOT

    //void prdoubleGoalPath(Node *path_node)
    void printGoalPath(std::tuple<std::vector<double>, std::vector<double>> path)
    {
        std::vector<double> xX = std::get<0>(path);
        std::vector<double> yY = std::get<1>(path);

        //obstacles for visualisation
        std::vector<int> obsX1{3, 7, 7, 3, 3};
        std::vector<int> obsY1{3, 3, 10, 10, 3};

        std::vector<int> obsX2{11, 18, 18, 11, 11};
        std::vector<int> obsY2{6, 6, 17, 17, 6};

        std::vector<int> obsX3{25, 27, 27, 25, 25};
        std::vector<int> obsY3{20, 20, 25, 25, 20};

        plt::figure_size(600, 600);
        plt::plot(obsX1, obsY1, "darkblue");
        plt::plot(obsX2, obsY2, "darkblue");
        plt::plot(obsX3, obsY3, "darkblue");

        plt::plot(xX, yY, "r--");
        plt::xlabel("xx");
        plt::ylabel("yy");
        plt::xlim(-1, 31);
        plt::ylim(-1, 31);

        plt::show();
    }

    //------------------------------------------------------------------------------

    void plotRRT(std::vector<Node*>& rrtNodes)
    {

        plt::figure_size(600, 600);
        std::vector<int> obsX1{3, 7, 7, 3, 3};
        std::vector<int> obsY1{3, 3, 10, 10, 3};

        std::vector<int> obsX2{11, 18, 18, 11, 11};
        std::vector<int> obsY2{6, 6, 17, 17, 6};


        std::vector<int> obsX3{25, 27, 27, 25, 25};
        std::vector<int> obsY3{20, 20, 25, 25, 20};

        plt::plot(obsX1, obsY1, "darkblue");
        plt::plot(obsX2, obsY2, "darkblue");
        plt::plot(obsX3, obsY3, "darkblue");

        // std::cout << "All nodes:\n";
        for (auto &ii : rrtNodes)
        {
            if(ii->prev == nullptr) continue;
            std::vector<double> xX;
            std::vector<double> yY;
            // xX.push_back(ii->posX);
            // yY.push_back(ii->posY);
            // Node* prev = ii->prev;
            // if(prev == nullptr) continue;
            // xX.push_back(prev->posX);
            // yY.push_back(prev->posY);

            // std::cout << ii->posX << "," << ii->posY << std::endl;

            xX = {ii->posX, ii->prev->posX};
            yY = {ii->posY, ii->prev->posY};

            plt::plot(xX, yY, "red");
        }






        plt::xlabel("xx");
        plt::ylabel("yy");
        plt::xlim(0, 31);
        plt::ylim(0, 31);

        plt::show();
    }
#endif

    //------------------------------------------------------------------------------

    //   https://en.wikipedia.org/wiki/Line_(geometry)
    //   https://en.wikipedia.org/wiki/doubleersection_(Euclidean_geometry)

    bool checkObstacles(double x1, double y1, double x2, double y2, std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>> obs)
    {
        bool decision;
        // std::cout << x1 << " : " << y1 << " : " << x2 << " : " << y2 << std::endl;

        std::vector<double> minX = std::get<0>(obs);
        std::vector<double> minY = std::get<1>(obs);
        std::vector<double> maxX = std::get<2>(obs);
        std::vector<double> maxY = std::get<3>(obs);

        for (int i = 0; i < minX.size(); i++)
        {

            // Completely outside.
            if ((x1 <= minX[i] && x2 <= minX[i]) || (y1 <= minY[i] && y2 <= minY[i]) || (x1 >= maxX[i] && x2 >= maxX[i]) || (y1 >= maxY[i] && y2 >= maxY[i]))
                decision = false;
        }
        for (int i = 0; i < minX.size(); i++)
        {

            double m = (y2 - y1) / (x2 - x1);

            double y = m * (minX[i] - x1) + y1;
            if (y > minY[i] && y < maxY[i])
                decision = true;

            y = m * (maxX[i] - x1) + y1;
            if (y > minY[i] && y < maxY[i])
                decision = true;

            double x = (minY[i] - y1) / m + x1;
            if (x > minX[i] && x < maxX[i])
                decision = true;

            x = (maxY[i] - y1) / m + x1;
            if (x > minX[i] && x < maxX[i])
                decision = true;
        }

        return decision;
    }

    //------------------------------------------------------------------------------

    std::vector<Node*> checkNearestNode(Node *new_node, std::vector<Node*>& rrtNodes, Node* start, Node* goal, std::vector<Node*>& rrtNodesNext)
    {

        std::vector<double> tempX;
        std::vector<double> tempY;

        Node *near_node;
        double minDistance = std::numeric_limits<double>::max();
        double corrX = 0.0;
        double corrY = 0.0;
        bool check_obstacle;

        

        for (auto &ii : rrtNodes)
        {

            double distance = std::sqrt(std::pow((new_node->posX - ii->posX), 2) + std::pow((new_node->posY - ii->posY), 2));
            if (distance < minDistance)
            {

                minDistance = distance;
                near_node = ii;
            }
        }

        double dx = new_node->posX - near_node->posX;
        double dy = new_node->posY - near_node->posY;
        double angle = std::atan2(dy, dx) * 180 / M_PI;

        if (minDistance > MAX_DISTANCE)
        {

            corrX = std::abs(near_node->posX + std::cos(angle) * MAX_DISTANCE);
            corrY = std::abs(near_node->posY + std::sin(angle) * MAX_DISTANCE);
        }

        if (minDistance <= MAX_DISTANCE)
        {

            corrX = new_node->posX;
            corrY = new_node->posY;
        }
        if (rrtNodes.size() > 0)
        {
            check_obstacle = checkObstacles(near_node->posX, near_node->posY, corrX, corrY, std::make_tuple(obsXmin, obsYmin, obsXmax, obsYmax));
        }
        new_node->posX = corrX;
        new_node->posY = corrY;

        // std::cout << " new is x: " << corrX << " y: " << corrY;

        // std::cout << " closest is : x: " << near_node->posX << " y: " << near_node->posY;

        near_node->next = new_node;
        new_node->prev = near_node;

        if (rrtNodes.size() == 0)
        {

            new_node->prev = start;
        }

        if (check_obstacle == 0)
        {
            // std::cout << " added...";

            rrtNodesNext.push_back(new_node);

        }


        if (((double)new_node->posX == (double)goal->posX) && ((double)new_node->posY == (double)goal->posY))
        {

            std::cout << "The GOAL achive && GOLD path is ..." << std::endl;

            GOAL = 1;

            while (new_node->prev != nullptr)

            {
                std::cout << new_node->posX << " :?: " << new_node->posY << std::endl;
                new_node = new_node->prev;
                tempX.push_back(new_node->posX);
                tempY.push_back(new_node->posY);
            }

            // printGoalPath(std::make_tuple(tempX, tempY));
        }
        return rrtNodesNext;

        // return new_node;
    }

    //------------------------------------------------------------------------------

    double lookForPath(std::vector<Node*>& rrtNodes, Node* start, Node* goal)
    {

        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist100(0, 30); 
        int i, tid;
        omp_set_num_threads(64);


        //pragma section
        omp_init_lock(&writelock);

        std::vector<Node*> rrtNodesNext;



        #pragma omp parallel shared(rrtNodes, start, goal) private(i, tid, rrtNodesNext)
        {

        
        tid = omp_get_thread_num();
        std::cout <<"tid: " << tid << std::endl;

        int chunk = MAX_ITERATION/4;

        // #pragma omp parallel shared(rrtNodes,start,goal,chunk) private(tid)
        // #pragma omp for schedule(static,chunk)
        for (i = 0; i < MAX_ITERATION; i++)
        {
            Node *random_node = new Node;
            Node *last_node = new Node;

            double randX = dist100(rng);
            double randY = dist100(rng);
            random_node->posX = randX;
            random_node->posY = randY;

            auto retVal = checkNearestNode(random_node,rrtNodes,start,goal, rrtNodesNext);
            // std::cout <<"tid: " << tid << " wait reinsert" << std::endl;
            #pragma omp barrier
            omp_set_lock(&writelock);

            while(!rrtNodesNext.empty()){
                rrtNodes.push_back(rrtNodesNext.back());
                rrtNodesNext.pop_back();
            }

            omp_unset_lock(&writelock);

            // std::cout <<"tid: " << tid << " wait next loop" << std::endl;

            #pragma omp barrier
        }



        }

        return 1;
    }


//------------------------------------------------------------------------------

int main()
{

    RRT rrt;
    init_RRT(rrt,10, 10, 27, 27);
    auto start = high_resolution_clock::now();



    
    lookForPath(rrt.rrtNodes,rrt.start,rrt.goal);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
     
    // To get the value of duration use the count()
    // member function on the duration object
    double t = duration.count()/1000000.;
    std::cout << "RUNTIME in seconds: " <<  t << std::endl;
   
#ifdef PLOT
    plotRRT(rrt.rrtNodes);
#endif
}
