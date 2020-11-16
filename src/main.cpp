#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>
//#include <opencv2/core.hpp>

#include <opencv2/opencv.hpp>

#include "fl/Headers.h"
#include "inc/lidar.h"
#include "inc/fuzzycontrol.h"
#include "inc/location.h"
#include "inc/pose.h"

static boost::mutex mutex;
bool k = true;

lidar lidarData;
pose robotPose;

void circleDetection(cv::Mat &img)
{
    cv::Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    //medianBlur(gray, gray, 5);
    GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 2,
                 gray.rows/2,  // change this value to detect circles with different distances to each other
                 10,           //canny edge threshold
                 100,
                 0, 200);      // change the last two parameters
                               // (min_radius & max_radius) to detect larger circles
    cv::Vec3i c;
    cv::Point center;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        c = circles[i];
        center = cv::Point(c[0], c[1]);
        // circle center
        circle( img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle( img, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
//        if(center.x)
//        {
//            std::cout << "circle detected, radius: " << radius << std::endl;
//        }
    }
}


void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}


void poseCallback(ConstPosesStampedPtr &_msg) { robotPose.poseCallback(_msg); }

void cameraCallback(ConstImageStampedPtr &msg) {

  std::size_t width = msg->image().width();
  std::size_t height = msg->image().height();
  const char *data = msg->image().data().c_str();
  cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

  im = im.clone();
  cv::cvtColor(im, im, cv::COLOR_RGB2BGR);

  mutex.lock();
  circleDetection(im);
  cv::imshow("camera", im);
  mutex.unlock();
}


void lidarCallback(ConstLaserScanStampedPtr &msg) { lidarData.lidarCallback(msg); }



int main(int _argc, char **_argv) {

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr statSubscriber =
      node->Subscribe("~/world_stats", statCallback);

  gazebo::transport::SubscriberPtr poseSubscriber =
      node->Subscribe("~/pose/info", poseCallback);

  gazebo::transport::SubscriberPtr cameraSubscriber =
      node->Subscribe("~/pioneer2dx/camera/link/camera/image", cameraCallback);

  gazebo::transport::SubscriberPtr lidarSubscriber =
      node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", lidarCallback);

  // Publish to the robot vel_cmd topic
  gazebo::transport::PublisherPtr movementPublisher =
      node->Advertise<gazebo::msgs::Pose>("~/pioneer2dx/vel_cmd");

  // Publish a reset of the world
  gazebo::transport::PublisherPtr worldPublisher =
      node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  gazebo::msgs::WorldControl controlMessage;
  controlMessage.mutable_reset()->set_all(true);
  worldPublisher->WaitForConnection();
  worldPublisher->Publish(controlMessage);

  const int key_esc = 27;

  fuzzyControl robotControl;
  //fuzzyControl::fuzzyData controlData;
  //Insert goal point for fuzzy:
  cv::Point goal(5, -3);
  float speed = 0;
  float steer = 0;

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();


    fuzzyControl::fuzzyData controlData = robotControl.setControl(robotPose, lidarData, goal);
    speed = controlData.speed;
    steer = controlData.steer;

    if (key == key_esc)
      break;

    // Generate a pose
    ignition::math::Pose3d pose(double(speed), 0, 0, 0, 0, double(steer));

    // Convert to a pose message
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
    movementPublisher->Publish(msg);

  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
