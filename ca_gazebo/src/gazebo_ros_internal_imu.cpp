#include <functional>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
double orientation_;
namespace gazebo
{
//angle_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("angle", 30);


class ModelPush : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"angle_node",ros::init_options::AnonymousName);
    }
    
    angle_pub_= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("angle", 30);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));
  }

  // Called by the world update start event
public:
  void OnUpdate()
  {
    // Apply a small linear velocity to the model.
    //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
    //double x, y, z;
    //ignition::math::Pose pose;
    ignition::math::Pose3d pose = this->model->gazebo::physics::Entity::RelativePose();	
    //pose = link->GetWorldPose();
    //std::cout << pose;
    //ignition::math::Pose3d v(0, 0, 0);
    //float yaw = pose.Rot().Yaw()*57.2958;
    float yaw = pose.Rot().Yaw();
    orientation_=yaw;
    //v = pose.rot;
    //x = v.x; // x coordinate
    //y = v.y; // y coordinate
    //z = v.z; // z coordinate
    //std::cout << this->model->ignition::math::Quaternion::Euler(ignition::math::Vector3d(pose.Rot().Roll(),pose.Rot().Pitch(), pose.Rot().Yaw()))	 << std::endl;
    //std::cout <<  yaw << std::endl;
    //std::cout <<  ignition::math::getAngle(pose.Rot());
    //std::cout << y;
    //std::cout << z;
    publishAngle();
    //node = new ros::NodeHandle(this->robot_namespace);
    //imu_data_publisher = node->advertise<sensor_msgs::Imu>(topic_name,1);
  }

  // Pointer to the model
private:
  physics::ModelPtr model;
  void publishAngle();
  ros::Publisher angle_pub_  ;
private:
  event::ConnectionPtr updateConnection;
protected:
      ros::NodeHandle nh;
  
};
void ModelPush::publishAngle()
{
    geometry_msgs::PoseWithCovarianceStamped angle_msg_;
    const float orientation_stddev = 5.0*M_PI/180.0; // 5 degrees
    /*
     * According to Roomba Open Interface specs, the angle has to be
     * divided by 0.324056 to get the angle in degrees.
     * So, to get it in radians, the number has to be divided by 180 too.
     */
    //orientation_ += (robot_->getAngle() * M_PI / 58.33008);
    /*float angle = std::fmod(orientation_  + M_PI,2*M_PI);
    if (angle < 0)
        angle += 2*M_PI;
    const float yaw = angle - M_PI;*/
    const float yaw = orientation_ + (((double) std::rand()*1) / (RAND_MAX));
    const float yaw_2 = yaw / 2.;
    const std::string str_base_footprint("base_link");
    angle_msg_.header.frame_id = str_base_footprint;
    angle_msg_.pose.pose.orientation.w = cos(yaw_2);
    angle_msg_.pose.pose.orientation.z = sin(yaw_2);
    angle_msg_.header.seq += 1;
    angle_msg_.header.stamp = ros::Time::now();
    angle_msg_.pose.covariance[35] = orientation_stddev * orientation_stddev;
    //ros::init(argc,argv,"angle_node",ros::init_options::AnonymousName);
   // ros::NodeHandle nh;
    //ros::Publisher angle_pub_= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("angle", 30);
    angle_pub_.publish(angle_msg_);
    return;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo


