#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <face_recognition/my_msg.h>

#define LINE_THICKNESS 2

int main(int argc, char** argv) {
  ros::init(argc, argv, "face_recognition");

  ros::NodeHandle nh("~");

  face_recognition::my_msg msg_to_pub;

  ros::Publisher pub = nh.advertise<face_recognition::my_msg>("face_center", 10);

  cv::VideoCapture camera(0);

  if(!camera.isOpened()) {
    ROS_ERROR("Nao abriu a camera");
    return -1;
  }

  cv::Mat frame;
  cv::CascadeClassifier face_detect;

  face_detect.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");

  while(ros::ok()) {
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

      ROS_WARN("x: %d, y: %d", faces[0].x, faces[0].y);
      
      //Publicando o centro do retangulo
      int32_t x_pos = faces[0].x + faces[0].width/2;
      int32_t y_pos = faces[0].y + faces[0].height/2;

      msg_to_pub.pos_x = x_pos;
      msg_to_pub.pos_y = y_pos;

      pub.publish(msg_to_pub);

      cv::imshow("Face Detection", frame);
      cv::waitKey(1);
    }
  }

  return 0;
}