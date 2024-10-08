#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <custom_msgs/my_msg.h>
#include <custom_msgs/point_vector.h>

#define LINE_THICKNESS 2

int main(int argc, char** argv) {
  ros::init(argc, argv, "face_recognition");

  ros::NodeHandle nh("~");

  custom_msgs::point_vector vector_to_pub;

  ros::Publisher pub_vector = nh.advertise<custom_msgs::point_vector>("vector_face_center", 10);

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

      vector_to_pub.size = faces.size();

      for(int i = 0; i < faces.size(); i++) {
        cv::rectangle(frame, faces[i].tl(), faces[i].br(), cv::Scalar(0, 255, 0), LINE_THICKNESS);

        geometry_msgs::Point point;

        point.x = faces[i].x + faces[i].width/2;
        point.y = faces[i].y + faces[i].height/2;

        vector_to_pub.points.push_back(point);
      }
      
      //Publicando o centro do(s) retangulo(s)
      pub_vector.publish(vector_to_pub);

      cv::imshow("Face Detection", frame);
      cv::waitKey(1);

      vector_to_pub.points.clear();
    }
  }

  return 0;
}