#include "./inc/vision.h"

vision::vision()
{ }


float vision::distanceCalculation(float objH)
{
    float realDiameter = 0.5 * 2 *1000 ; //Real radius is 0.5 m, the real diameter is 1m = 1000 mm
    float focalLength = 0.5544;// mm
    float sensorH =  0.51703344; //mm
    float imageHeight = 240; //mm

    //The formula below is:
    /*  distanceToObject [mm] = ( focalLength[mm] * realHeightOfObject[mm] * imageHeight[pixels] ) /  ( objectHeight[pixels] * sensorHeight[mm] )  */

    float dist = (focalLength*realDiameter*imageHeight)/(2*objH * sensorH);

    return dist;
}

void vision::circleDetection(cv::Mat img)
{
    cv::Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    //medianBlur(gray, gray, 5);
    GaussianBlur( gray, gray, cv::Size(9, 9), 2, 2 );
    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 20,           //canny edge threshold
                 100,
                 5, 200);      // change the last two parameters
                               // (min_radius & max_radius) to detect larger circles
    cv::Vec3i c;
    cv::Point center;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        c = circles[i];

         // circle center
        center = cv::Point(c[0], c[1]);
        circle( img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);

        // circle outline
        int radius = c[2];
        circle( img, center, radius, cv::Scalar(255,0,255), 1, cv::LINE_AA);

        if(center.x  < 150)
        {
            std::cout << "Robot should turn left" << std::endl;
        }
        else if(center.x > 160)
        {
            std::cout << "Robot should turn right" << std::endl;
        }
        else
            std::cout << "Robot should go ahead" << std::endl;

    std::cout << "The radius of the marble is: " << radius <<" The distance to the marble is: " << distanceCalculation(radius)/1000 << "[m]" <<std::endl;
    }
}
