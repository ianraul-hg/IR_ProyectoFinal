#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void imgCallback(const sensor_msgs::ImageConstPtr& msg);

void controlador_X(int cx, int columnas, ros::NodeHandle node_handle);
void controlador_Y(int cy, int filas, ros::NodeHandle node_handle);

bool init_gazebo_engine(void);

float T_muestra=0.02;

//Variables control eje X
float paraX_K0=0, paraX_K1=0, paraX_K2=0;
float error_X=0, error_X1=0, error_X2=0, control_X1=0, control_X=0;

//Variables control eje Y
float paraY_K0=0, paraY_K1=0, paraY_K2=0;
float error_Y=0, error_Y1=0, error_Y2=0, control_Y1=0, control_Y=0;

int columnas = 0;
int filas = 0;

int cx = 0;
int cy = 0;


int main(int argc, char** argv)
{ 
    cv::Mat imagen_entrada;
    cv::Mat imagen_salida;
    cv::Mat img_hsv;

    ros::init(argc,argv, "proyecto");
    ros::NodeHandle node_handle;


    image_transport::ImageTransport imt(node_handle);

    image_transport::Subscriber img_sub = imt.subscribe("/robotis_op3/camera/image_raw", 1, imgCallback);

    init_gazebo_engine();

    while(ros::ok())
    {
    ros::spin();

    //Segmentacion
    cvtColor(imagen_entrada, img_hsv, cv::COLOR_BGR2HSV);
    inRange(img_hsv, cv::Scalar(0, 173, 85), cv::Scalar(159, 255, 255), imagen_salida);

    erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    erode(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
    dilate(imagen_salida, imagen_salida, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

    //Centroide Frame
    int columnas = imagen_entrada.cols;
    int filas = imagen_entrada.rows;

    //Centroide Objeto
    cv::Moments moment = cv::moments(imagen_salida, true);

    cx = int(moment.m10/moment.m00);
    cy = int(moment.m01/moment.m00);

    controlador_X(cx, columnas, node_handle);
    controlador_Y(cy, filas, node_handle);

    }

    return 0;
}

void imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat imagen_entrada = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("image wrong");
    }


}

void controlador_X(int cx, int columnas, ros::NodeHandle node_handle)
{
    ros::Publisher signal_motorX = node_handle.advertise<std_msgs::Float64>("/robotis_op3/head_pan_position/command", 1);

    std_msgs::Float64 signal_X;

    float K_pro = 0.0035;
    float T_der = 0.005;    
    float  T_int = 0.1;
    float valor_pos= 160;

    paraX_K0 = K_pro + (K_pro * T_der/T_muestra) + (K_pro * T_muestra/T_int);
    paraX_K1 = K_pro - (2*K_pro*T_der/T_muestra);
    paraX_K2 = K_pro * T_der/T_muestra;

    error_X = cx - (columnas/2);
    control_X = control_X1 + paraX_K0*error_X + paraX_K1*error_X1 + paraX_K2*error_X2;

    signal_X.data = valor_pos - control_X;  //Importante signo
    std::cout << control_X << "\n";
    signal_motorX.publish(signal_X);

    control_X1 = control_X;
    error_X2 = error_X1;
    error_X1 = error_X;


}

void controlador_Y(int cy, int filas, ros::NodeHandle node_handle)
{   
    ros::Publisher signal_motorY  = node_handle.advertise<std_msgs::Float64>("/robotis_op3/head_tilt_position/command", 1);

    std_msgs::Float64 signal_Y;

    float K_pro = 0.0035;
    float T_der = 0.005;    
    float  T_int = 0.1;
    float valor_pos= 100;

    paraY_K0 = K_pro + (K_pro * T_der/T_muestra) + (K_pro * T_muestra/T_int);
    paraY_K1 = K_pro - (2*K_pro*T_der/T_muestra);
    paraY_K2 = K_pro * T_der/T_muestra;

    error_Y = cy - (filas/2);
    control_Y = control_Y1 + paraY_K0*error_Y + paraY_K1*error_Y1 + paraY_K2*error_Y2;

    signal_Y.data = valor_pos - control_Y;      //Importante signo
    std::cout << control_Y << "\n";
    signal_motorY.publish(signal_Y);

    control_Y1 = control_Y;
    error_Y2 = error_Y1;
    error_Y1 = error_Y;

}

bool init_gazebo_engine(void)
{
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;
  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused)
  {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }
  if (!unpaused)
  {
    ROS_FATAL("Could not wake up Gazebo.");
    return false;
  }
  ROS_INFO("Unpaused the Gazebo simulation.");
  // Wait for Gazebo GUI show up.
  ros::Duration(10).sleep();
  return true;
}