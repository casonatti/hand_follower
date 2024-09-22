#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#define LINE_THICKNESS 2

int main(int argc, char** argv) {
  ros::init(argc, argv, "face_recognition");

  ros::NodeHandle nh("~");

  ros::Publisher 

  cv::VideoCapture camera(0);

  if(!camera.isOpened()) {
    ROS_ERROR("Nao abriu a camera");
    return -1;
  }

  cv::Mat frame;
  cv::CascadeClassifier face_detect;

  face_detect.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");

  while(true) {
    camera.read(frame);

    if(frame.empty()) {
      std::cout << "Deu pau no frame" << std::endl;
      continue;
    } else {
      std::vector<cv::Rect> faces;
      face_detect.detectMultiScale(frame, faces, 1.3, 5);

      for(int i = 0; i < faces.size(); i++) {
        cv::rectangle(frame, faces[i].tl(), faces[i].br(), cv::Scalar(0, 255, 0), LINE_THICKNESS);
      }

      //Estudar como publicar um std::vector
      ROS_WARN("x: %d, y: %d", faces[0].x, faces[0].y);

      cv::imshow("Face Detection", frame);
      cv::waitKey(1);
    }
  }

  return 0;
}