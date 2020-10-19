#include "inc/location.h"
#define PI 3.141592653589793



location::location()
{
  x = 0;
  y = 0;
  theta = 0;
}

location::~location()
{

}

void location::update(float speed, float dir, int sec)
{
  this->theta += (dir/685.0F)*2*PI;           // theta
  this->x += (cos(this->theta) * speed)/48.5; // x
  this->y += (sin(this->theta) * speed)/48.5; // y

  //temporary debug input
  if(sec == 'r')
  {
    this->x = 0;
    this->y = 0;
  }

  std::cout << "theta: "  << this->theta  << std::endl;
  std::cout << "x: "      << this->x      << std::endl;
  std::cout << "y: "      << this->y      << std::endl;
}

void location::plot(cv::Mat& map, float x, float y)
{
  cv::Mat frame = map.clone();
  cv::namedWindow("Location", cv::WINDOW_NORMAL);
  cv::resizeWindow("Location", 500, 500);
  float xx, yy;
  xx = this->x;
  yy = this->y;

  frame.at<cv::Vec3b>(cv::Point(x, y) + cv::Point(xx, yy))[0] = 0;
  frame.at<cv::Vec3b>(cv::Point(x, y) + cv::Point(xx, yy))[1] = 255;
  frame.at<cv::Vec3b>(cv::Point(x, y) + cv::Point(xx, yy))[2] = 255;
  cv::imshow("Location", frame);
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
