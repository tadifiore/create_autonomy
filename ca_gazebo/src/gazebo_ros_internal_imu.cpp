#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class ModelPush : public ModelPlugin
{
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
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
    float yaw = pose.Rot().Yaw();
    //v = pose.rot;
    //x = v.x; // x coordinate
    //y = v.y; // y coordinate
    //z = v.z; // z coordinate
    //std::cout << this->model->gazebo::math::Quaternion::GetAsEuler(yaw)	 << std::endl;
    std::cout <<  yaw;
    //std::cout <<  ignition::math::getAngle(yaw);
    //std::cout << y;
    //std::cout << z;

    //node = new ros::NodeHandle(this->robot_namespace);
    //imu_data_publisher = node->advertise<sensor_msgs::Imu>(topic_name,1);
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

    //ros::NodeHandle* node;
    /// \brief Ros Publisher for imu data.
    //ros::Publisher imu_data_publisher;
    /// \brief Ros IMU message.
    //sensor_msgs::Imu imu_msg;
  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo