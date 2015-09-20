#include <ros/ros.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <boost/scoped_array.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>


void imageCallback(const sensor_msgs::ImageConstPtr& msg);

int AprilSocket = -1;

#define MAGIC 0x17923349ab10ea9aL

//function copied directly from prosilica_april.cpp
int64_t utime_now()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

//function copied directly from prosilica_april.cpp
void write_i32(uint8_t* buf, int32_t v)
{
  buf[0] = (v >> 24) & 0xFF;
  buf[1] = (v >> 16) & 0xFF;
  buf[2] = (v >>  8) & 0xFF;
  buf[3] = (v) & 0xFF;
}

//function copied directly from prosilica_april.cpp
void write_i64(uint8_t* buf, int64_t v)
{
  uint32_t h = (uint32_t)(v >> 32);
  uint32_t l = (uint32_t)(v);

  write_i32(buf + 0, h);
  write_i32(buf + 4, l);
}

/**
 * Image callback, handles ros msgs converts cv::Mat 
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
 	{
    	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
    }

  cv::Mat gray = cv_ptr->image.clone();
  cv::cvtColor(cv_ptr->image, gray, CV_RGB2GRAY);

  static const int HeaderBytes = 37;
  static const std::string format = "GRAY8";
  size_t BufferLength = HeaderBytes + gray.total();
  static boost::scoped_array<unsigned char> buffer(new unsigned char[BufferLength]);
  unsigned char* ptr = buffer.get();
  write_i64(ptr, MAGIC);
  ptr += 8;
  write_i64(ptr, utime_now());
  ptr += 8;
  write_i32(ptr, cv_ptr->image.cols);
  ptr += 4;
  write_i32(ptr, cv_ptr->image.rows);
  ptr += 4;
  write_i32(ptr, format.size());
  ptr += 4;
  memcpy(ptr, format.c_str(), format.size());
  ptr += format.size();
  write_i32(ptr, gray.total());
  ptr += 4;
  memcpy(ptr, gray.data, gray.total());

  int sent_bytes = send(AprilSocket, buffer.get(), HeaderBytes + gray.total(), 0);

  if(sent_bytes != BufferLength)
  {
    std::cout << "Only " << sent_bytes << " were sent for this image." << std::endl;
  }
};


/**
 * Configures april socket.
 */

bool configure_tcp(const std::string& host, int port, int& sock)
{
  sock = socket(AF_INET, SOCK_STREAM, 0);

  if(sock < 0)
  {
    printf("Error opening socket\n");
    perror("AprilSocket: ");
    return false;
  }

  struct sockaddr_in serv_addr;

  memset((char*)&serv_addr, 0, sizeof(serv_addr));

  serv_addr.sin_family = AF_INET;

  serv_addr.sin_port   = htons(port);

  serv_addr.sin_addr.s_addr = inet_addr(host.c_str());

  if(connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
  {
    printf("Error connecting to socket\n");
    perror("connect: ");
    return false;
  }

  return true;

}

/**
 * Clean up socket
 */
void cleanup_socket(int sock)
{
  if(sock > 0)
  {
    close(sock);
  }
}

/**
 * Main function, creates ros node and publishes, subscribes
 * to image_transport
 */
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "april_node");
	ros::NodeHandle nh;

	std::cout << "Printing args: " << std::endl;
	std::cout << "Host: " << argv[1] << ", " << "Port: " << argv[2] << std::endl;

 	if(argc != 3)
  	{
    	printf("Usage: prosilica_april <host> <port>\n");
    	exit(1);
  	}

  	std::string host = std::string(argv[1]);

  	int port = atoi(argv[2]);
	
  	if(!configure_tcp(host, port, AprilSocket))
  	{
    	cleanup_socket(AprilSocket);
    	return 1;
  	}
	
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);
    
    ros::spin();

	return 0;
}

