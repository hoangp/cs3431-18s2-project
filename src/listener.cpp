#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <openpose/headers.hpp>
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

#include <openpose/flags.hpp>
#include <openpose/core/common.hpp>
#include <openpose/core/enumClasses.hpp>
#include <openpose/pose/enumClasses.hpp>
#include <openpose/pose/poseParameters.hpp>
#include <openpose/pose/poseParametersRender.hpp>

ros::Subscriber sub;
ros::Publisher pub;

void handle_openpose(const cv_bridge::CvImagePtr &cv_ptr);

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const sensor_msgs::Image &img)
{
  cv_bridge::CvImagePtr cv_ptr;

  try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e){
      return;
  }

  handle_openpose(cv_ptr);
}

void handle_openpose(const cv_bridge::CvImagePtr &cv_ptr) 
{
  const auto data = cv_ptr->image;
  
  std::cout << "converted image" << std::endl;
  op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};

  op::WrapperStructPose wrapper{};
  wrapper.modelFolder = "/home/sassbot/openpose/models/";

  // when given the turtlebot camera, openpose netInputSize defaults to [656,368]
  // the turtlebot camera appears to output at a resolution of [640,480]
  // wrapper.netInputSize = op::Point<int>(640,480);
  
  // not really neccesary to disable it but the option is there
  // wrapper.renderMode = op::RenderMode::None;
  
  // set it to scale results between 0 and 1
  //wrapper.keypointScale = op::ScaleMode::ZeroToOne;

  opWrapper.configure(wrapper);
    
  opWrapper.start(); 
  
  auto datumProcessed = opWrapper.emplaceAndPop(data);

  // std_msgs::String msg;
  // std::stringstream ss;
  // ss << "hello world ";
  // msg.data = ss.str();

  if (datumProcessed != nullptr){
      std::cout << "processed image" << std::endl;
     
      // next 3 lines are for printouts
      //op::log("Body keypoints: " + datumProcessed->at(0).poseKeypoints.toString());
      cv::imshow("User worker GUI", datumProcessed->at(0).cvOutputData);
      cv::waitKey(15);

      std_msgs::String contents;
      contents.data = datumProcessed->at(0).poseKeypoints.toString();
    
      pub.publish(contents);

  } else {
      std::cout << "Dead" << std::endl;
  }

}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  sub = n.subscribe("camera/rgb/image_raw", 1, chatterCallback);
  //sub = n.subscribe("pr/person_image", 1, chatterCallback);
  
  pub = n.advertise<std_msgs::String>("pr/op_25kps", 10);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}