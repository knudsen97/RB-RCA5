#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <iostream>
//#include <opencv2/core.hpp>

#include <opencv2/opencv.hpp>

#include "fl/Headers.h"
#include "inc/lidarscanner.h"
#include "inc/fuzzycontrol.h"
#include "inc/location.h"


static boost::mutex mutex;
bool k = true;
float speed = 0;
float steer = 0;


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


void poseCallback(ConstPosesStampedPtr &_msg) {
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();

//    for (int i = 0; i < _msg->pose_size(); i++) {
//    if (_msg->pose(i).name() == "pioneer2dx") {
//      std::cout << std::setprecision(2) << std::fixed << std::setw(6)
//                << _msg->pose(i).position().x() << std::setw(6)
//                << _msg->pose(i).position().y() << std::setw(6)
//                << _msg->pose(i).position().z() << std::setw(6)
//                << _msg->pose(i).orientation().w() << std::setw(6)
//                << _msg->pose(i).orientation().x() << std::setw(6)
//                << _msg->pose(i).orientation().y() << std::setw(6)
//                << _msg->pose(i).orientation().z() << std::endl;
//    }
//  }

    fuzzyControl B;
    steer = B.setControl(_msg).steer;
    speed = B.setControl(_msg).speed;
}


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


void lidarCallback(ConstLaserScanStampedPtr &msg) {
//    lidarScanner A;
//    std::cout << "range: " << A.lidarMin(msg).range <<std::endl;
//    std::cout << "angle: " << A.lidarMin(msg).angle <<std::endl;
    
    
    // fuzzyControl B;
    // steer = B.setControl(msg).steer;
    // speed = B.setControl(msg).speed;

}



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

  // Loop
  while (true) {
    gazebo::common::Time::MSleep(10);

    mutex.lock();
    int key = cv::waitKey(1);
    mutex.unlock();

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
