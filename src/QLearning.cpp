#include "inc/QLearning.h"

QLearning::QLearning()
{
    srand(time(NULL));
    qTableIndex = 0;
    rewardIndex = 0;
};
QLearning::~QLearning(){};


QLearning::state QLearning::terminal(QLearning::state s)
{
    if(rewardCount == 0 || steps == 0 )
        return {s.x, s.y, false};
    else
        return s;   
}

bool QLearning::validPixel(QLearning::state s)
{
    if(this->img.at<cv::Vec3b>(cv::Point(s.x, s.y)) == cv::Vec3b(0,0,0))
        return false;
    else
        return true;        

}

void QLearning::colourPixel(cv::Mat &map, cv::Point p, char B, char G, char R)
{
    cv::Vec3b &colour = map.at<cv::Vec3b>(p);
    colour[0] = B;
    colour[1] = G;
    colour[2] = R;
    map.at<cv::Vec3b>(p) = colour;
}

cv::Vec3b QLearning::pixelColour(cv::Point p){ return this->img.at<cv::Vec3b>(p); }


QLearning::action QLearning::nextAction(QLearning::state s)
{
    std::vector<int> idxs;
    float random = (rand()%100)/100.0f;
    if(epsilon > random)    //Explore with epsilon greedy otherwise pick best action
        return action(rand()%4); //Perform random action
    else
    {
        float temp = qTableList[qTableIndex][s.x][s.y][0];

        for(int i = 0; i < 4; i++)
        {
            if(temp < qTableList[qTableIndex][s.x][s.y][i])
            {
                temp = qTableList[qTableIndex][s.x][s.y][i];
                idxs.clear();
                idxs.push_back(i);
            }
            else if(temp == qTableList[qTableIndex][s.x][s.y][i])
            {
                idxs.push_back(i);
            }
        }
        return action((idxs[rand()%idxs.size()]));//pick best action
    } 
}

float QLearning::getReward(QLearning::action a, QLearning::state s)
{
    float score = 0;
    score = this->rewardTableTemp[s.x][s.y];
    this->rewardTableTemp[s.x][s.y] = 0.0f;
    cv::Point statePosition(s.x, s.y);

    //remove reward if it exist in enviromentTemp
    int i = 0; //Reward index
    for (reward_t reward : this->rewardList) 
    {
        if (reward.position == statePosition)   //found maching reward
            if(this->enviromentTemp.at<uchar>(cv::Point(s.x, s.y)) != EMPTY) //if there is something
            {
                if (reward.superReward)
                    rewardCount--;
                else if(reward.score < 0)
                    rewardCount=0;
                score = this->enviromentTemp.at<uchar>(cv::Point(s.x, s.y));
                this->enviromentTemp.at<uchar>(cv::Point(s.x, s.y)) = EMPTY;
                rewardIndex = i;
            }
        i++;
    }

    return score;
}



QLearning::state QLearning::getNextState(QLearning::action a, QLearning::state s)
{
    switch(a)
    {
        case LEFT:
        s.x -= 1; break;
        
        case RIGHT:
        s.x += 1; break;
        
        case UP:
        s.y -= 1; break;
        
        case DOWN:
        s.y += 1; break;
        
        default:
        std::cout << "default case\n";
        break;
    }

    return s;
}

void QLearning::resetVariables()
{
    QLearning::setStartPosition(this->start);
    this->imgDisplay = this->img.clone();

    int superGoalCount = 0;
    for (QLearning::reward_t reward : this->rewardList)
        superGoalCount = (reward.superReward) ? (superGoalCount+1) : (superGoalCount);
    this->rewardCount = superGoalCount;
    
    this->rewardTableTemp = this->rewardTable;
    this->enviromentTemp = this->enviroment.clone();
    this->qTableIndex = 0;

    QLearning::resetImage(); // reset BGR
}

void QLearning::resetImage()
{
    for (QLearning::reward_t reward : this->rewardList)
    {
        if(reward.score < 0)
        {
            QLearning::colourPixel(this->img, reward.position, 0, 0, 255);   // Negative reward = RED
        }
        else if(reward.score > 0 && !reward.superReward)
        {
            QLearning::colourPixel(this->img, reward.position, 0, 255, 255); // Breadcrumb goal = YELLOW
        }
        else if (reward.superReward)
        {
            QLearning::colourPixel(this->img, reward.position, 0, 255, 0);   // Super goal = GREEN
        }
    }
}

//Public methods ------------------------------------------------------------
bool QLearning::loadMap(cv::Mat &map, std::string windowName)
{
    this->windowName = windowName;

    this->img = map.clone();
    cv::cvtColor(map, this->enviroment, cv::COLOR_BGR2GRAY);
    cv::cvtColor(map, this->enviromentTemp, cv::COLOR_BGR2GRAY);

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 600, 600);

    cv::imshow(windowName, img);
    cv::waitKey(0);

    this->width = img.cols;
    this->height = img.rows;

    rewardTable.resize(this->width, std::vector<float>(this->height));
    rewardTableTemp.resize(this->width, std::vector<float>(this->height));

    qTable.resize(this->width, std::vector<std::array<float,4>>(this->height));
    for (size_t col = 0; col < map.cols; col++)
        for (size_t row = 0; row < map.rows; row++)
            if (map.at<cv::Vec3b>(cv::Point(col, row)) == cv::Vec3b{255, 255, 255} ) //if free space
                rewardTable[col][row] = stepPoint;
            else
                rewardTable[col][row] = wallPoint;
    

    return true;
}

void QLearning::setReward(std::vector<cv::Point> rewards, float score, bool superGoal)
{
    for(cv::Point reward : rewards)
    {
        this->rewardList.push_back({reward, score, superGoal} );
        rewardTable[reward.x][reward.y] = score;
        if(!superGoal)
            this->enviroment.at<uchar>(reward) = GOAL;
        else if(superGoal)
            this->enviroment.at<uchar>(reward) = SUPERGOAL;
    }

    if(superGoal)
    {
        this->rewardCount = rewards.size();
        this->qTableList.resize(pow(2,this->rewardCount), qTable); // Allocate space for qTableList for Markov implementation
        std::cout << "qTableList size: " << this->qTableList.size() << '\n';
    }
}

void QLearning::setStartPosition(cv::Point start)
{ 
    agent = {start.x, start.y, true}; 
    this->start = start;
}

void QLearning::setVariables(float gamma, float epsilon, float alpha, int totalSteps, float wallPoint, float stepPoint)
{
    this->gamma = gamma;
    this->epsilon = epsilon;
    this->alpha = alpha;
    this->totalSteps = totalSteps;
    this->wallPoint = wallPoint;
    this->stepPoint = stepPoint;
}

void QLearning::training(int trainingAttempts)
{
    QLearning::resetVariables();
    int count = 0;
    steps = this->totalSteps;

    std::ofstream test_data("ai_test.csv");
    while(trainingAttempts >= 0)
    {
        count++;
        steps--;
        action a = nextAction(agent);
        state next = getNextState(a, agent);
        
        float reward = getReward(a, next);

        action aNext = nextAction(next);
        qTableList[qTableIndex][agent.x][agent.y][a] += alpha * (reward + gamma * qTableList[qTableIndex][next.x][next.y][aNext] - qTableList[qTableIndex][agent.x][agent.y][a]);
        this->qTableIndex |= 1 << rewardIndex; //Markov property to see which goal we have taken.

        if(this->enviromentTemp.at<uchar>(cv::Point(next.x, next.y)) != 0)
            agent = next;

        if(trainingAttempts < 1)
        {
            QLearning::plotDebug(a);
            cv::waitKey(0);	
        }


        agent = terminal(agent);

        if(!agent.validPos)
        {
            if(epsilon > 0.05f) 
                epsilon -= 0.0005;
            if(alpha > 0.01f) 
                alpha -= 0.0005;
            trainingAttempts--;
    
            QLearning::plotDebug(a);

            QLearning::resetVariables();
            steps = this->totalSteps;
            std::cout << count << '\n';
            test_data << count << '\n';
            count = 0;
        }
    }
}

void QLearning::plotDebug(int act)
{
    imgDisplay = img.clone();
    imgDisplay.at<cv::Vec3b>({agent.x, agent.y}) = cv::Vec3b{COLOR_AGENT};                  //draw agent

    //draw next state if there is an action
    if(act != -1) 
    {
        QLearning::state nextState = QLearning::getNextState(static_cast<QLearning::action>(act), agent);
        imgDisplay.at<cv::Vec3b>({nextState.x, nextState.y}) = cv::Vec3b{COLOR_NEXTSTATE};
    }
    

    //draw negative and positive "rewards"
    for (QLearning::reward_t reward : this->rewardList)
    {
        if (reward.score > 0)
            if (reward.superReward)                                                         //positive and supergoal
            {
                if(this->enviromentTemp.at<uchar>(reward.position) == EMPTY)
                    imgDisplay.at<cv::Vec3b>(reward.position) = cv::Vec3b{COLOR_SUPERGOAL_X};
                else
                    imgDisplay.at<cv::Vec3b>(reward.position) = cv::Vec3b{COLOR_SUPERGOAL};

            }
            else                                                                            //positive and everything else
                if(this->enviromentTemp.at<uchar>(reward.position) == EMPTY)
                    imgDisplay.at<cv::Vec3b>(reward.position) = cv::Vec3b{COLOR_GOAL_X};
                else
                    imgDisplay.at<cv::Vec3b>(reward.position) = cv::Vec3b{COLOR_GOAL};
        else if (reward.score < 0)                                                          //if negative
            imgDisplay.at<cv::Vec3b>(reward.position) = cv::Vec3b{COLOR_PUNISH};
    }
    cv::imshow(windowName, imgDisplay);

}

void QLearning::printTable()
{
    for(size_t row = 0; row < height; row++)
    {
        for (size_t col = 0; col < width; col++)
        {
            std::cout << "{ ";
            for(size_t act = 0; act < 4; act++)
            {
                std::cout << std::setprecision(2) << qTable[col][row][act] << " ";
            }
            std::cout << "}\t\t\t";
        }
        std::cout << '\n';
    }
    std::cout << "--------------------------------------------\n";

}