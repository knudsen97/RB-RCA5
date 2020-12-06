#include "inc/driver.h"

static boost::mutex mutex;
bool k = true;

lidar lidarData;
pose robotPose;
camera cam;
vision vis;

void statCallback(ConstWorldStatisticsPtr &_msg) {
  (void)_msg;
  // Dump the message contents to stdout.
  //  std::cout << _msg->DebugString();
  //  std::cout << std::flush;
}

void poseCallback(ConstPosesStampedPtr &_msg) { robotPose.poseCallback(_msg); }

void cameraCallback(ConstImageStampedPtr &msg) { cam.cameraCallback(msg); }

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


  //Uncomment according to what to test:
  //pathFinder(); //Pathfinding
  //qLearning(); //QLearning


  const int key_esc = 27;
  fuzzyControl robotControl;
  
  //Insert goal point for fuzzy:
  cv::Point goal(5, 3);
  float speed = 0;
  float steer = 0;

  // Fuzzy loop
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
