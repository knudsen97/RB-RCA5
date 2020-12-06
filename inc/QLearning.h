#ifndef QLEARNING_H
#define QLEARNING_H
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <array>
#include <cmath>
#include <iostream>
#include <random>
#include <time.h>
#include <iomanip> 
#include <fstream>
#include <bitset>

#define GOAL        'g'    //goal and punishment
#define SUPERGOAL   'G'
#define EMPTY       255
#define WALL        0

#define COLOR_AGENT         255, 0, 0
#define COLOR_NEXTSTATE     150, 0, 0
#define COLOR_SUPERGOAL     0, 255, 0
#define COLOR_SUPERGOAL_X   0, 100, 0
#define COLOR_GOAL          0, 255, 240
#define COLOR_GOAL_X        0, 235, 220
#define COLOR_PUNISH        0, 0, 255
#define COLOR_WALL          0, 0, 0,

class QLearning
{
private:

    //Variables
    struct state
    {
        int x,y;
        bool validPos;
    };

    struct reward_t
    {
        cv::Point   position;
        float       score;
        bool        superReward;
    };
    
    state TERMINAL_STATE = {-1, -1, false};
    enum action{ LEFT, RIGHT, UP, DOWN};
    
    float alpha, epsilon, gamma;    //Learning rate, epsilon greedy, discount factor
    float wallPoint, stepPoint;
    int totalSteps, steps;
    std::vector<std::vector<std::array<float,4>>> qTable;
    std::vector<std::vector<std::vector<std::array<float,4>>>> qTableList;
    int qTableIndex;
    int rewardIndex;
    std::vector<std::vector<float>> rewardTable, rewardTableTemp;
    std::vector<reward_t> rewardList;
    cv::Mat enviroment, enviromentTemp;

    cv::Mat img, imgDisplay;
    std::string windowName;
    int width, height;
    char rewardCount;

    state agent;
    cv::Point start;

    //Private methods:
    bool validPixel(state s);
    void colourPixel(cv::Mat &map, cv::Point p, char B, char G, char R);
    cv::Vec3b pixelColour(cv::Point p);
    state terminal(state s);
    action nextAction(state s);
    float getReward(action a, state s);  //Get a reward accordingly to the action and state.
    state getNextState(action a, state s);
    void resetVariables();
    void resetImage();

    std::vector<std::vector<std::array<float,4>>> getQTable(char index);
    

public:
    QLearning();
    ~QLearning();

    bool loadMap(cv::Mat &map, std::string windowName);
    void setReward(std::vector<cv::Point> rewards, float score, bool superGoal=0);
    void setStartPosition(cv::Point start);
    void setVariables(float gamma = 0.7, float epsilon=0.7, float alpha=0.6, int totalSteps=5000, float wallPoint=-1.0f, float stepPoint=0.0f);
    void training(int trainingAttempts);
    void plotDebug(int act = -1);


    void printTable();
};


#endif // QLEARNING_H