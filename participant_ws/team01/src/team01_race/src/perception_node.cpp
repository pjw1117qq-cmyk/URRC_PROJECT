#include <cmath>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
class PerceptionNode:public rclcpp::Node{
public:PerceptionNode():Node("team01_perception"){
  const auto input=declare_parameter<std::string>("input_topic","/team01/scan");
  const auto output=declare_parameter<std::string>("output_topic","/team01/internal/scan");
  max_=declare_parameter<double>("max_distance_m",5.0);
  min_deg_=declare_parameter<double>("roi_min_deg",-90.0);max_deg_=declare_parameter<double>("roi_max_deg",90.0);
  if(max_<=0||min_deg_<-90||max_deg_>90||min_deg_>=max_deg_)throw std::invalid_argument("invalid ROI");
  auto qos=rclcpp::SensorDataQoS().keep_last(1);pub_=create_publisher<sensor_msgs::msg::LaserScan>(output,qos);
  sub_=create_subscription<sensor_msgs::msg::LaserScan>(input,qos,[this](sensor_msgs::msg::LaserScan::ConstSharedPtr in){
    auto out=*in;const double rad=3.14159265358979323846/180.0;
    for(std::size_t i=0;i<out.ranges.size();++i){const double a=out.angle_min+i*double(out.angle_increment);
      if(a<min_deg_*rad||a>max_deg_*rad)out.ranges[i]=NAN;
      else if(std::isfinite(out.ranges[i])&&out.ranges[i]>max_)out.ranges[i]=INFINITY;}
    pub_->publish(out);
  });}
private:double max_,min_deg_,max_deg_;rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<PerceptionNode>());rclcpp::shutdown();}
