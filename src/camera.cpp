#include "inc/camera.h"



camera::camera()
{

}

void camera::cameraCallback(ConstImageStampedPtr &msg)//Update image stored in class´
{
    std::size_t width = msg->image().width(); //Get image width from msg information
    std::size_t height = msg->image().height(); //Get image height from msg information
    const char *data = msg->image().data().c_str(); //Get image from msg information

    cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data)); //Create image from heigth, width and image

    im = im.clone();
    cv::cvtColor(im, im, cv::COLOR_RGB2BGR); //Convert from RGB to BGR


    image = im; //Update image stored in class

}

cv::Mat camera::getImage()//Returns image stored in class
{
    return image;
}

