#pragma once
// -----------------------------------------------------------------------------
// base_link_estimator.h
//
// Estimates the static transform from a tracked rigid body frame to the robot's
// base_link (the midpoint of the two fixed wheels) for a unicycle / tricycle
// style robot, using only the streamed mocap pose plus two markers that straddle 
// the wheel axle, which are used to center the base link on the axel line.
// -----------------------------------------------------------------------------

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>
#include <vector>
#include <deque>
#include <mutex>

class BaseLinkEstimator
{
public:
    void Init(ros::NodeHandle &n);

    bool Enabled() const { return enabled_; }

    void SetBodyMarker(const std::string &body, int marker_idx,
                       double mx, double my, double mz);

    void AddSample(const std::string &body, double t,
                   double px, double py, double pz,
                   double qx, double qy, double qz, double qw);

    void PublishIfReady(const std::string &body,
                        double px, double py, double pz,
                        double qx, double qy, double qz, double qw,
                        const std::string &global_frame);

private:
    struct Sample { double t, x, y, th; };

    bool computeSrv(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);
    bool compute(std::string &report);
    static double yawFromQuat(double qx, double qy, double qz, double qw);

    // --- config ---
    bool enabled_ = false;
    std::string target_body_;
    int marker_a_ = 0, marker_b_ = 1;
    double omega_min_ = 0.20;     // rad/s, used to filter out near-straight motion
    double min_speed_ = 0.05;     // m/s, used to filter out standing still
    int    diff_stride_ = 5;      // frames between samples used for ICR
    int    min_samples_ = 300;    // ICR points needed before compute is allowed
    double max_icr_dist_ = 5.0;   // m, reject ICRs further than this from body
    bool   publish_cloud_ = true; // publish ICR cloud (in body frame) for rviz

    // --- live state ---
    std::deque<Sample> hist_;            // recent samples for finite differencing
    std::vector<float> icr_x_, icr_y_;   // ICR cloud in body frame
    std::vector<float> icr_w_;           // per-point weight 
    double fwd_sum_x_ = 0.0, fwd_sum_y_ = 0.0; // mean body-frame velocity (heading sign)
    bool   have_marker_a_ = false, have_marker_b_ = false;
    double ma_x_ = 0, ma_y_ = 0, ma_z_ = 0, mb_x_ = 0, mb_y_ = 0, mb_z_ = 0;

    // --- result (body-frame offset body -> base_link) ---
    bool   calibrated_ = false;
    double off_x_ = 0, off_y_ = 0, off_z_ = 0, off_yaw_ = 0;

    // --- ros handles ---
    ros::ServiceServer srv_;
    ros::Publisher pose_pub_;
    ros::Publisher cloud_pub_;
    std::mutex mtx_; 
};

extern BaseLinkEstimator base_link_estimator;
