#include "inc/RoadMap.h"

RoadMap::RoadMap()
{
    lineWidth = 1;
}


RoadMap::~RoadMap()
{

}


bool RoadMap::loadMap(std::string map, std::string name, int scalingFactor)
{
    this->name = name;
    img = cv::imread(map);
    if(img.empty())
    {
        std::cout << "Could not read the image/map: " << map << std::endl;
        return false;
    }
    cv::resize(img, img, cv::Size(), scalingFactor, scalingFactor, cv::INTER_NEAREST_EXACT);
    cv::cvtColor(img, greyImg, cv::COLOR_BGR2GRAY);

    cv::Mat kernel = cv::Mat::ones(10,10,CV_8UC1);
    cv::erode(greyImg, greyImg, kernel); // Erosion to avoid getting too close to walls.

    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::resizeWindow(name, 600, 600);

    width = img.cols;
    height = img.rows;
    this->scalingFactor = scalingFactor;

    cv::imshow(name, img);
    cv::waitKey(0);
    return true;
}


void RoadMap::quasiSampling(int n)
{
    float p, u, v;
    int k, kk, pos;
    cv::Point pt;
    for(k=0, pos=0; k<n; k++)
    {
        u = 0;
        for(p=0.5, kk=k;kk;p*=0.5, kk>>=1)
        {
            if(kk & 1)
                u += p;
        }
        v = (k+0.5)/n;
        pt = {u*width,v*height};
        
        if( greyImg.at<unsigned char>(pt) == 255 )
        {
            randomPoints.push_back(pt);
            if(k != n)
            {
                img.at<cv::Vec3b>(pt).val[2] = 255;
                img.at<cv::Vec3b>(pt).val[1] = 0;
                img.at<cv::Vec3b>(pt).val[0] = 0;
            }
        }

    }
}


void RoadMap::generateNodes(int nPoints, cv::Point start, std::vector<cv::Point> &goals)
{
    RoadMap::quasiSampling(nPoints);
    std::cout << "done quasi\n";
    
    this->start = start*scalingFactor;
    cv::circle(img, this->start, 8, cv::Scalar(0,255,0), -1, 8);

    this->goals = goals;

    for(int subGoal = 0; subGoal < this->goals.size(); subGoal++)
    {
        this->goals[subGoal] = this->goals[subGoal]*scalingFactor;
        cv::circle(img, this->goals[subGoal], 8, cv::Scalar(0,0,255), -1, 8);
    }

    cv::imshow(name, img);
    cv::waitKey(0);
}


void RoadMap::findShortestDist()
{
    path defaultPath ={
        cv::Point(0,0),
        cv::Point(0,0),
        9999,
        false
    };

    for(int i = 0; i < randomPoints.size(); i++)
    {
        for (int j = i+1; j < randomPoints.size() - 1; j++)
        {
            if(RoadMap::valid(randomPoints[i], randomPoints[j], greyImg))
            {
                std::sort(shortestDist.begin(), shortestDist.end(), [](path a, path b)
                {
                    return a.distance > b.distance;
                });

                float dist = sqrt(pow(randomPoints[j].x - randomPoints[i].x,2) + pow(randomPoints[j].y - randomPoints[i].y,2) );

                if(shortestDist[0].distance > dist && dist != 0 && dist < 300)
                {
                    shortestDist[0].start = randomPoints[i];
                    shortestDist[0].end = randomPoints[j];
                    shortestDist[0].distance = dist;
                }
            }
        }

        for(int k = 0; k < shortestDist.size() ; k++)
            if(shortestDist[k].distance != 9999 && shortestDist[k].distance > 0.001)
                pathHolder.push_back(shortestDist[k]);

        shortestDist.fill(defaultPath);
    }
}


void RoadMap::generateRM()
{
    RoadMap::findShortestDist();
    for(int i = 0; i < pathHolder.size();i++) //Draw yellow lines to form roadmap
        cv::line(img, pathHolder[i].start, pathHolder[i].end, cv::Scalar(92,204,255), lineWidth);


    float distStartTemp = 999, distGoalTemp = 999;
    goalConnections.resize(goals.size());
    for(int subGoal = 0; subGoal<goals.size(); subGoal++)
    {
        for( int i = 0; i < randomPoints.size(); i++) //Connect start and goal to nearest node
        {
            if(subGoal == 0)
            {
                float distStart = sqrt(pow(randomPoints[i].x - start.x,2) + pow(randomPoints[i].y - start.y,2) );
                if(distStartTemp > distStart)
                {
                    startConnection = randomPoints[i];
                    distStartTemp = distStart;

                }
            }
            float distGoal = sqrt(pow(randomPoints[i].x - goals[subGoal].x,2) + pow(randomPoints[i].y - goals[subGoal].y,2) );
            if(distGoalTemp > distGoal)
            {
                goalConnections[subGoal] = randomPoints[i];
                distGoalTemp =  distGoal;
            }
        }
        distGoalTemp = 999;
        cv::line(img, goals[subGoal], goalConnections[subGoal] , cv::Scalar(0,0,255), lineWidth);
        cv::circle(img, goals[subGoal], 8, cv::Scalar(0,0,255), -1, 8);
    }
    
    cv::line(img, start, startConnection , cv::Scalar(0,255,0), lineWidth);
    cv::circle(img, start, 8, cv::Scalar(0,255,0), -1,8);

    std::cout << "Roadmap done\n";
    cv::imshow(name, img);
    cv::waitKey(0);
}


std::vector<RoadMap::path> RoadMap::weightedPath(cv::Point start)
{
    path temp;
    temp.distance = 0;
    std::vector<RoadMap::path> weightedPaths;
    for( int i= 0; i < pathHolder.size(); i++)  // Kan optimeres ved ikke at kører hele listen igennem.
    {
        if(start == pathHolder[i].start && pathHolder[i].visited == false) //1st iteration start == startCOnnection
        {
            pathHolder[i].visited = true;
            weightedPaths.push_back(pathHolder[i]);
            //cv::line(img, pathHolder[i].start, pathHolder[i].end, cv::Scalar(0,255,0), lineWidth);
        }
        else if(start == pathHolder[i].end && pathHolder[i].visited == false)
        {
            temp = {
                pathHolder[i].end,
                pathHolder[i].start,
                pathHolder[i].distance,
                true
            };
            pathHolder[i].visited = true;
            weightedPaths.push_back(temp);
            //cv::line(img, temp.start, temp.end, cv::Scalar(0,255,0), lineWidth);
        }
    }

    for(int i = 0; i < weightedPaths.size(); i++)
    {
        for( int pathIndex = 0; pathIndex < pathHolder.size(); pathIndex++)
        {
            if(pathHolder[pathIndex].start == weightedPaths[i].end && pathHolder[pathIndex].visited == false) //Outwards from the node
            {
                temp = {
                    pathHolder[pathIndex].start, 
                    pathHolder[pathIndex].end, 
                    pathHolder[pathIndex].distance + weightedPaths[i].distance,
                    true
                };
                weightedPaths.push_back(temp);
                pathHolder[pathIndex].visited = true;
                //cv::line(img, temp.start, temp.end, cv::Scalar(0,255,0), lineWidth);
            }
            if(pathHolder[pathIndex].end == weightedPaths[i].start && pathHolder[pathIndex].visited == false) // Towards the node
            {
                temp = {
                    pathHolder[pathIndex].end, 
                    pathHolder[pathIndex].start, 
                    pathHolder[pathIndex].distance + weightedPaths[i].distance,
                    true
                };
                weightedPaths.push_back(temp);

                pathHolder[pathIndex].start = temp.start;
                pathHolder[pathIndex].end = temp.end; 
                pathHolder[pathIndex].visited = true;

                //cv::line(img, temp.start, temp.end, cv::Scalar(0,255,0), lineWidth);
            }
        }
    }

    return weightedPaths;
}


// //EXPLORE ALGORITHM DJIKSTRA ------------------------------------------------------------------------------- 
std::vector<std::vector<cv::Point>> RoadMap::explore()
{
    std::vector<RoadMap::path> weightedPaths; 
    std::vector<std::vector<cv::Point>> subGoalVec;
    std::vector<cv::Point> subGoal;
    cv::Point node;
    path pathToStart;
    std::vector<path> pathHolderTemp;

    int goalIndex = 0;
    int distance = 9999;

    for(int j = 0; j<goalConnections.size(); j++)
    {
        pathHolderTemp = this->pathHolder;
        weightedPaths = RoadMap::weightedPath(startConnection);
        this->pathHolder = pathHolderTemp;

        for(int i = 0; i < weightedPaths.size(); i++) //Find goalIndex in weightedPaths that has the shortest distance.
        {
            if(weightedPaths[i].end == goalConnections[j] && weightedPaths[i].distance < distance)
            {
                distance = weightedPaths[i].distance;
                goalIndex = i;
            }
        }
        distance = 9999;

        node = weightedPaths[goalIndex].end;
        pathToStart = weightedPaths[goalIndex];
        subGoal.push_back(goals[j]);

        while(node != startConnection)
        {
            for(auto path : weightedPaths) //Find edge connected to node
                if(path.end == node)
                    if(pathToStart.distance > path.distance) //If distance on path is shorter.
                        pathToStart = path;
            
            cv::line(img, pathToStart.start, pathToStart.end, cv::Scalar(0,0,255), lineWidth);

            node = pathToStart.start;

            subGoal.push_back(node);
        }
  
        subGoal.push_back(startConnection);
        subGoal.push_back(start);


        std::reverse(subGoal.begin(), subGoal.end()); // WE go from finish to start to find shortest path
        subGoalVec.push_back(subGoal);
        subGoal.clear();
        cv::line(img, start, node, cv::Scalar(0,0,255), lineWidth);
        
        start = goals[j];   //Update current start to reached goal
        startConnection = goalConnections[j]; //Update current start connecttion to reached goal connection
    }

    std::cout << "explore done, click to proceed to post processing.\n";
    cv::imshow(name, img);
    cv::waitKey();

    for(int i = 0; i < subGoalVec.size(); i++)
    {
        subGoalVec[i] = RoadMap::postProcessing(subGoalVec[i]);
        cv::circle(img, goals[i], 8, cv::Scalar(0,255,0), -1, 8); //To update visited goals to green

        cv::imshow(name, img);
        cv::waitKey();
    }

    std::cout << "post processing done, click to exit.\n";
    cv::waitKey();


    return subGoalVec;
}


//POST PROCESSING TO SKIP REDUNDANT POINTS--------------------------------------------------------------------
std::vector<cv::Point> RoadMap::postProcessing(std::vector<cv::Point> &subGoal)
{
    for(int i = 0; i < subGoal.size()-2; i++)
    {
        if(RoadMap::valid(subGoal[i], subGoal[i+2], greyImg))
        {
            subGoal.erase(
                std::remove_if(subGoal.begin(), subGoal.end(), [&](cv::Point const & point){
                    return point == subGoal[i+1];
                }), 
                subGoal.end());
            i = 0;
        }
        
    }

    for(int i = 0; i < subGoal.size()-1; i++)
        cv::line(img, subGoal[i], subGoal[i+1], cv::Scalar(255,0,0), 2);   

    return subGoal;
}


//CHECKS IF LINE BETWEEN TWO POINTS INTERSECTS A WALL-------------------------------
bool RoadMap::valid(cv::Point A, cv::Point B, cv::Mat img)
{
    bool valid = true;
    cv::LineIterator it(img, A, B);

    for(int k = 1; k < it.count-1; k++, ++it)
        valid = (*(const int*)* it == false) ? false : valid;

    return valid;
}


//DEBUGGING PURPOSES ------------------------------------------------------------
void RoadMap::print(RoadMap::path p) 
{
    std::cout << "start: " << p.start << " end: " << p.end << " dist: " << p.distance << " visited: " << p.visited << '\n';
}