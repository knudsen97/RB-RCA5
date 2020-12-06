#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "fl/Headers.h"
#include "inc/lidar.h"
#include "inc/fuzzycontrol.h"
#include "inc/location.h"
#include "inc/pose.h"
#include "inc/RoadMap.h"
#include "inc/vision.h"
#include "inc/QLearning.h"
#include "inc/camera.h"

void pathFinder()
{
    RoadMap map;
    std::vector<cv::Point> goalPointsBig = 
    {
      cv::Point(10,7),
      cv::Point(24,7),  
      cv::Point(17,24),  
      cv::Point(9,60),  
      cv::Point(32,61),  
      cv::Point(43,17),  
      cv::Point(67,9),  
      cv::Point(68,26),  
      cv::Point(94,17),  
      cv::Point(110,18),  
      cv::Point(110,46),  
      cv::Point(71,54),  
      cv::Point(104,70),  
      cv::Point(79,70),  
      cv::Point(55,71)  
    };
    cv::Point startBig(65, 35);

    map.loadMap("bigworld.png", "Map", 8);
    map.generateNodes(1000, startBig, goalPointsBig);
    map.generateRM();
    map.explore();
}

std::vector<cv::Point> generateGoal(int n, cv::Mat &map)
{
    std::vector<cv::Point> goals;
    cv::Point goal;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    for(int i = 0; i < n;)
    {
        double n1 = distribution(generator);
        double n2 = distribution(generator);
        goal = {map.cols, map.rows};
        if(map.at<cv::Vec3b>(cv::Point(goal.x * n1, goal.y * n2)) == cv::Vec3b(255,255,255))
        {
            goal.x *= n1;
            goal.y *= n2;
            goals.push_back(goal);
            std::cout << goal << '\n';
            i++;

        }
    }
    return goals;
}

bool qLearning()
{
    srand(time(NULL));

    //Small world map
    cv::Mat map1 = cv::imread("smallworld.png");
    if(map1.empty())
    {
        std::cout << "Could not read the image/map: " << "smallworld.png" << std::endl;
        return false;
    }

    //Big map:
    int dimY = 34;
    int dimX = 34;
    cv::Mat map2(dimY,dimX, CV_8UC3, cv::Scalar(0,0,0));
    cv::rectangle(map2, cv::Point(0,11), cv::Point(33,21),cv::Scalar(255,255,255), -1);
    cv::rectangle(map2, cv::Point(11,0), cv::Point(21,33),cv::Scalar(255,255,255), -1);

    cv::rectangle(map2, cv::Point(0,11), cv::Point(33,21),cv::Scalar(0,0,0));
    cv::rectangle(map2, cv::Point(11,0), cv::Point(21,33),cv::Scalar(0,0,0));

    cv::line(map2, cv::Point(11,16), cv::Point(11,18), cv::Scalar(255,255,255));
    cv::line(map2, cv::Point(21,14), cv::Point(21,16), cv::Scalar(255,255,255));
    cv::line(map2, cv::Point(16,21), cv::Point(18,21), cv::Scalar(255,255,255));
    cv::line(map2, cv::Point(13,11), cv::Point(16,11), cv::Scalar(255,255,255));

    //Medium map
    int dim = 10;
    cv::Mat map3(dim,dim*3, CV_8UC3, cv::Scalar(255,255,255));
    cv::rectangle(map3, cv::Point(0,0), cv::Point((dim*3)-1,dim-1),cv::Scalar(0,0,0));
    cv::line(map3, cv::Point(10,0), cv::Point(10,6), cv::Scalar(0,0,0));
    cv::line(map3, cv::Point(20,9), cv::Point(20,3), cv::Scalar(0,0,0));

    //Big world map
    cv::Mat map4 = cv::imread("bigworld.png");
    if(map4.empty())
    {
        std::cout << "Could not read the image/map: " << "bigworld.png" << std::endl;
        return false;
    }


    std::vector<cv::Point> goalPointsBig = 
    {
      cv::Point(10,7),
      cv::Point(24,7),  
      cv::Point(17,24),  
      cv::Point(9,60),  
      cv::Point(32,61),  
      cv::Point(43,17),  
      cv::Point(67,9),  
      cv::Point(68,26),  
      cv::Point(94,17),  
      cv::Point(110,18),  
      cv::Point(110,46),  
      cv::Point(71,54),  
      cv::Point(104,70),  
      cv::Point(79,70),  
      cv::Point(55,71)  
    };
    
    cv::Point start = {16,16};
    cv::Point startBig(65, 35);

    float discount_factor = 0.7; 
    float e_greedy = 0.7; 
    float learning_rate = 0.8; 
    int total_steps = 4000000;
    float wall_point = -1.0f;
    float step_point = -1.0f;

    int training_attempts = 20000;

    std::vector<cv::Point> goals = generateGoal(10, map2);
    QLearning AI;
    AI.loadMap(map2, "test");
    
    AI.setReward(goals, 100, 1);
    AI.setStartPosition(start);

    // AI.setReward(goalPointsBig, 100, 1);
    // AI.setStartPosition(startBig);

    AI.setVariables(discount_factor, e_greedy, learning_rate, total_steps);
    AI.training(training_attempts);

    return true;
}
