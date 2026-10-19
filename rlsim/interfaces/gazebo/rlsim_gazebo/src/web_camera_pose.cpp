// Move only the observation camera on Gazebo's update thread. The generic ROS
// SetModelState API pauses/resumes the whole world for each pose update.
#include <gazebo/common/Events.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

namespace rlsim {
class WebCameraPose : public gazebo::ModelPlugin {
 public:
  ~WebCameraPose() override {
    update_.reset();
    subscriber_.shutdown();
    if (node_) node_->shutdown();
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) override {
    if (!ros::isInitialized() || model->GetName() != "rlsim_web_camera") {
      gzerr << "WebCameraPose requires ROS and the observation camera model\n";
      return;
    }
    model_ = model;
    node_.reset(new ros::NodeHandle("/rlsim_viewer"));
    subscriber_ = node_->subscribe("camera_pose", 1, &WebCameraPose::Pose, this,
                                  ros::TransportHints().tcpNoDelay());
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&WebCameraPose::Update, this));
  }

 private:
  void Pose(const geometry_msgs::Pose::ConstPtr& message) {
    const auto& p = message->position;
    const auto& q = message->orientation;
    for (double v : {p.x, p.y, p.z, q.w, q.x, q.y, q.z})
      if (!std::isfinite(v)) return;
    const double norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    if (norm < 1e-12) return;
    ignition::math::Quaterniond rotation(q.w, q.x, q.y, q.z);
    rotation.Normalize();
    std::lock_guard<std::mutex> guard(mutex_);
    pending_ = ignition::math::Pose3d({p.x, p.y, p.z}, rotation);
    changed_ = true;
  }

  void Update() {
    if (!changed_.exchange(false)) return;
    ignition::math::Pose3d pose;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      pose = pending_;
    }
    model_->SetWorldPose(pose);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Subscriber subscriber_;
  std::mutex mutex_;
  std::atomic<bool> changed_{false};
  ignition::math::Pose3d pending_;
};
GZ_REGISTER_MODEL_PLUGIN(WebCameraPose)
}  // namespace rlsim
