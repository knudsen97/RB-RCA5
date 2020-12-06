#include <iostream> 
#include <chrono> 
#include <random>
 

#include "inc/location.h"

#define PI 3.141592653589793

void quasiRandom(int p, int k, std::vector<int> output)
{
  int _p = p;
  int _k = k;
  int a = 0;
  int theta = 0;
  while( _k > 0 )
  {
    int a = _k % p;
    int Theta = Theta;
    output.push_back(theta);
    _k = _k/p;
    _p = _p*p;
  }
}


location::location()
{
  x = 0;
  y = 0;
  theta = 0;
}

location::~location()
{
  ;
}

location::location(pose initialPose)
{
  x = initialPose.x;
  y = initialPose.x;
  theta = initialPose.theta;
  objectPose = initialPose;
}

location::pose location::update(float speed, float dir, float sec)
{
  time = sec - pretime;
  pretime = sec;
  this->theta += dir*time;           // theta
  this->theta = (this->theta < -PI) ? (this->theta+2*PI) : (this->theta);
  this->theta = (this->theta > PI) ? (this->theta-2*PI) : (this->theta);
  float cordlength = 2*(speed/dir)*sin(this->theta/2);
  if(dir > 0.001 && speed > 0.001)
  {
    this->x += sin((this->theta)/2)*cordlength*time; // x
    this->y += cos(this->theta/2)*cordlength*time; // y
  }
  else
  { 
    this->x += cos(this->theta)*speed*time; // x
    this->y += sin(this->theta)*speed*time; // y
  }

  objectPose = pose{x, y, theta};
  return pose{x, y, theta};
}

void location::initialParticleDistributer(cv::Mat& map, int particleNumber, std::vector<location::pose>& output)
{
  std::vector<location::pose> particles;
  location::pose poseTemp;
  std::default_random_engine generator;
  std::uniform_int_distribution<int> x_distribution(20, map.cols-20);
  std::uniform_int_distribution<int> y_distribution(20, map.rows-20);
  std::uniform_real_distribution<double> angle_distribution(-PI, PI);
  for(int particleIndex = 0; particleIndex < particleNumber; particleIndex++)
  {
    poseTemp.x = x_distribution(generator);
    poseTemp.y = y_distribution(generator);
    poseTemp.theta = angle_distribution(generator);
    output.push_back(poseTemp);
  }
}

void location::lidarSimulator(cv::Mat& inputMap, cv::Mat& outputMap, location::pose particle, double lidarMaxRange)
{
  cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(particle.x, particle.y), particle.theta*(180/PI), 1);
  cv::Mat rotatedMap;
  
  //rotate
  cv::warpAffine(inputMap, rotatedMap, rotationMatrix, inputMap.size(), 1, 0, 255);

  int xOffset = particle.x-lidarMaxRange;
  int yOffset = particle.y-lidarMaxRange;
  int width = lidarMaxRange*2;
  int height = lidarMaxRange*2;

  //set boundary
  xOffset = (xOffset >= 0) ? (xOffset) : (0);
  yOffset = (yOffset >= 0) ? (yOffset) : (0);
  width = (xOffset+width > rotatedMap.cols) ? (rotatedMap.cols-xOffset) : (width);
  height = (yOffset+height > rotatedMap.rows) ? (rotatedMap.rows-yOffset) : (height);


  
  //crop
  cv::Rect crop( xOffset, yOffset, width, height );
  outputMap = rotatedMap(crop);
}

void location::particleFilter(cv::Mat& _map, std::vector<location::pose>& input, std::vector<location::pose>& output, ConstLaserScanStampedPtr &msg)
{
  //clone map to not manipulate map directly
  cv::Mat map;
  if(_map.channels() != 1)
    cv::cvtColor(_map, map, cv::COLOR_BGR2GRAY);
  else
    map = _map.clone();
  
  //internal errode walls
  cv::Mat core = cv::Mat::ones(3,3, CV_8UC1);
  cv::Mat dilatedMap;
  cv::dilate(map, dilatedMap, core);
  map = ~(map ^ dilatedMap);

  //extract lidar data
  double maxrange = msg->scan().range_max();
  std::vector<double> lidarData;
  std::vector<double> lidarAngle;
  int samples = msg->scan().ranges_size();
  double angleStep = msg->scan().angle_step();
  double angleStart = msg->scan().angle_min();

  for(int laserIndex = 0; laserIndex < samples; laserIndex++)
  {
    lidarData.push_back(msg->scan().ranges(laserIndex));
    lidarAngle.push_back(angleStart + laserIndex * angleStep);
  }

  /***analyse lidar data***/
  int width = 400;  int height = 400;
  float pixlePerUnit = 200 / maxrange;
  cv::Mat particleVision;
  cv::Mat robotVision(height, width, CV_8UC1);
  robotVision.setTo(255);

  //draw points for robot
  for(int dataIndex = 0; dataIndex < lidarData.size(); dataIndex++)
  {
    robotVision.at<uchar>
    (
      200.5f + lidarData[dataIndex] * pixlePerUnit * std::cos(lidarAngle[dataIndex]),
      200.5f - lidarData[dataIndex] * pixlePerUnit * std::sin(lidarAngle[dataIndex])
    ) = 0;
  }
  cv::Mat rotationMatrix = cv::getRotationMatrix2D(cv::Point2f(robotVision.cols/2, robotVision.rows/2), 90, 1);
  cv::warpAffine(robotVision, robotVision, rotationMatrix, robotVision.size());

  cv::imshow("robot", robotVision);
  //draw points for particle
  std::string windowTitle;
  int i = 0;
  for(location::pose particle : input)
  {
    location::lidarSimulator(map, particleVision, particle, pixlePerUnit*maxrange);
    windowTitle = "particle: " + std::to_string(i);
    cv::namedWindow(windowTitle, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowTitle, 400, 400);
    cv::imshow(windowTitle, particleVision);
    i++;
    // filter or weight particle probability.
  }
  cv::imshow("Map", map);
}

void location::plot(cv::Mat& map, float x, float y, std::string name)
{
  cv::Mat frame = map.clone();
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 500, 500);
  float xx, yy;
  xx = this->x;
  yy = this->y;

  cv::circle(frame, cv::Point(x, y) + cv::Point(xx, yy), 1, cv::Scalar(0, 255, 255));
  cv::imshow(name, frame);
}

void location::plot(cv::Mat& map, location::pose input, std::string name)
{
  cv::Mat frame = map.clone();
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 500, 500);

  cv::circle(frame, cv::Point(x, y) + cv::Point(input.x, input.y), 1, cv::Scalar(0, 255, 255));
  cv::imshow(name, frame);
}

void location::plot(cv::Mat& map, std::vector<location::pose> inputs, std::string name)
{
  cv::Mat frame = map.clone();
  cv::namedWindow(name, cv::WINDOW_NORMAL);
  cv::resizeWindow(name, 500, 500);

  for(location::pose input : inputs)
  {
    cv::circle(frame, cv::Point(x, y) + cv::Point(input.x, input.y), 1, cv::Scalar(0, 255, 255));
  }
  cv::imshow(name, frame);
}

float location::get(char pose)
{
  switch (pose)
  {
  case 'x':
    return this->x;

  case 'y':
    return this->y;

  case 't':
    return this->theta;

  default:
    throw "not an option";
  }
}

float location::get(state index)
{
  switch (index)
  {
  case 0 :
    return this->x;

  case 1 :
    return this->y;

  case 2:
    return this->theta;

  default:
    throw "not an option";
  }
}



void location::testingFunction(char input)
{
  switch (input)
  {
  case 'w':
    this->theta = PI/2;
    break;
  case 'a':
    this->theta = PI;
    break;
  case 's':
    this->theta = 3*PI/2;
    break;
  case 'd':
    this->theta = 0;
    break;
  
  default:
    break;
  }
}