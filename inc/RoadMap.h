#ifndef ROADMAP_H
#define ROADMAP_H
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <array>
#include <cmath>
#include <iostream>

class RoadMap
{
private:
    struct path
    {
        cv::Point start;
        cv::Point end;
        float distance;
        bool visited;
    };

public:
    RoadMap();
    ~RoadMap();

    void quasiSampling(int n);
    bool loadMap(std::string map,std::string name,  int scalingFactor); //Loads map and scales it to scaling factor
    void generateNodes(int nPoints, cv::Point start,                    //Generates points with quasi and inserts start and goal points.
                        std::vector<cv::Point> & goals);
    void findShortestDist();                                            //Find at least 3 nodes for each node
    void generateRM();                                                  //Generates roadmap by drawing lines between nodes
    std::vector<RoadMap::path> weightedPath(cv::Point start);
    std::vector<std::vector<cv::Point>> explore();                      //The explore function is used to implement Dijkstras algorithm

    std::vector<cv::Point> postProcessing(std::vector<cv::Point> &);    //Post processing to skip redundant nodes

    bool valid(cv::Point, cv::Point, cv::Mat);                          //Checks that a line between 2 points doesnt go through any walls(black pixels)
    void print(path p);                                                 //Print for debugging purposes


private:
    int width, height, scalingFactor;

    std::string name;

    cv::Mat img;
    cv::Mat greyImg;

    cv::Point start, startConnection;
    
    std::vector<cv::Point> goals, goalConnections;
    std::vector<cv::Point> randomPoints;
    
    std::array<path, 3> shortestDist;
    std::vector<path> pathHolder;

    int lineWidth;
};

#endif // ROADMAP_H