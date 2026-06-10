// -----------------------------------------------------------------------------
// base_link_estimator.cpp   
// 
// Written by Nico Zucca with Claude 4.8 for NavFlex
// -----------------------------------------------------------------------------
#include "base_link_estimator.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <cmath>

// TODO Add saving and loading of the base-link offset per vehicle
namespace
{
inline double wrap(double a)            // wrap to (-pi, pi]
{
    return std::atan2(std::sin(a), std::cos(a));
}
}

void BaseLinkEstimator::Init(ros::NodeHandle &n)
{
    n.param<bool>("estimate_base_link", enabled_, false);
    if (!enabled_)
        return;

    n.param<std::string>("base_link/target_body", target_body_, "");
    n.param<int>("base_link/marker_a", marker_a_, 0);
    n.param<int>("base_link/marker_b", marker_b_, 1);
    n.param<double>("base_link/omega_min", omega_min_, 0.20);
    n.param<double>("base_link/min_speed", min_speed_, 0.05);
    n.param<int>("base_link/diff_stride", diff_stride_, 5);
    n.param<int>("base_link/min_samples", min_samples_, 300);
    n.param<double>("base_link/max_icr_dist", max_icr_dist_, 5.0);
    n.param<bool>("base_link/publish_cloud", publish_cloud_, true);

    if (target_body_.empty())
        ROS_WARN("[base_link] estimate_base_link is set but base_link/target_body is empty.");

    srv_       = n.advertiseService("estimate_base_link", &BaseLinkEstimator::computeSrv, this);
    pose_pub_  = n.advertise<geometry_msgs::PoseStamped>(target_body_ + "/base_link/pose", 50);
    if (publish_cloud_)
        cloud_pub_ = n.advertise<sensor_msgs::PointCloud>(target_body_ + "/base_link/icr_cloud", 5);

    ROS_INFO("[base_link] enabled for body '%s' (markers %d,%d). "
             "Drive with turns, then: rosservice call %s/estimate_base_link",
             target_body_.c_str(), marker_a_, marker_b_, n.getNamespace().c_str());
}

double BaseLinkEstimator::yawFromQuat(double qx, double qy, double qz, double qw)
{
    tf2::Quaternion q(qx, qy, qz, qw);
    double r, p, y;
    tf2::Matrix3x3(q).getRPY(r, p, y);
    return y; // yaw about Z (heading for a level, Z-up body)
}

void BaseLinkEstimator::SetBodyMarker(const std::string &body, int marker_idx,
                                      double mx, double my, double /*mz*/)
{
    if (!enabled_ || body != target_body_)
        return;
    if (marker_idx == marker_a_) { ma_x_ = mx; ma_y_ = my; have_marker_a_ = true; }
    if (marker_idx == marker_b_) { mb_x_ = mx; mb_y_ = my; have_marker_b_ = true; }
}

void BaseLinkEstimator::AddSample(const std::string &body, double t,
                                  double px, double py, double /*pz*/,
                                  double qx, double qy, double qz, double qw)
{
    if (!enabled_ || body != target_body_)
        return;

    std::lock_guard<std::mutex> lk(mtx_);
    if (calibrated_)
        return;

    Sample s{t, px, py, yawFromQuat(qx, qy, qz, qw)};
    hist_.push_back(s);
    while ((int)hist_.size() > diff_stride_ + 1)
        hist_.pop_front();
    if ((int)hist_.size() < diff_stride_ + 1)
        return;

    const Sample &a = hist_.front();   // older
    const Sample &b = hist_.back();    // newer
    double dt = b.t - a.t;

    if (dt <= 1e-4 ||  // Too close (time)
        dt > 1.0)      // Too far (time)
        return;

    double vx = (b.x - a.x) / dt;
    double vy = (b.y - a.y) / dt;
    double w  = wrap(b.th - a.th) / dt;
    double speed = std::hypot(vx, vy);

    if (std::fabs(w) < omega_min_ ||        // Too straight
        speed < min_speed_)                 // Too slow
        return;   

    // Instantaneous center of rotation
    double ox = -vy / w;
    double oy =  vx / w;
    if (std::hypot(ox, oy) > max_icr_dist_)
        return;                        // Too straight 

    // TODO
    // Rotate the offset into the body frame using the MIDPOINT heading: the
    // finite-difference velocity approximates the instantaneous velocity at the
    // middle of the interval, so the heading must be taken there too (using the
    // end heading injects a stride-dependent bias).
    double th = std::atan2(0.5 * (std::sin(a.th) + std::sin(b.th)),
                           0.5 * (std::cos(a.th) + std::cos(b.th)));
    double c = std::cos(th), sn = std::sin(th);
    double bx =  c * ox + sn * oy;
    double by = -sn * ox + c * oy;
    icr_x_.push_back((float)bx);
    icr_y_.push_back((float)by);
    icr_w_.push_back((float)(w * w));  // tight turns are far less noisy -> trust them more

    // Accumulate mean body-frame velocity to fix the forward (heading) sign.
    fwd_sum_x_ +=  c * vx + sn * vy;
    fwd_sum_y_ += -sn * vx + c * vy;

    // TODO this is messy, don't use icr_x_, don't use a magic number
    if (publish_cloud_ && cloud_pub_.getNumSubscribers() > 0 && (icr_x_.size() % 10 == 0))
    {
        sensor_msgs::PointCloud pc;
        pc.header.stamp = ros::Time::now();
        pc.header.frame_id = target_body_;     // body tf frame from PubRigidbodyPose
        pc.points.reserve(icr_x_.size());
        for (size_t i = 0; i < icr_x_.size(); ++i)
        {
            geometry_msgs::Point32 p;
            p.x = icr_x_[i]; p.y = icr_y_[i]; p.z = 0.0f;
            pc.points.push_back(p);
        }
        cloud_pub_.publish(pc);
    }
}

bool BaseLinkEstimator::compute(std::string &report)
{
    std::lock_guard<std::mutex> lk(mtx_);
    const int n = (int)icr_x_.size();
    if (n < min_samples_)
    {
        report = "Not enough ICR samples (" + std::to_string(n) + "/" +
                 std::to_string(min_samples_) + "). Drive with more / tighter turns.";
        return false;
    }
    if (!have_marker_a_ || !have_marker_b_)
    {
        report = "Marker positions not found. Check base_link/marker_a,marker_b "
                 "and that the body has those marker indices.";
        return false;
    }

    // --- weighted PCA line fit of the ICR cloud (axle line) ---
    double W = 0, mx = 0, my = 0;
    for (int i = 0; i < n; ++i) { W += icr_w_[i]; mx += icr_w_[i] * icr_x_[i]; my += icr_w_[i] * icr_y_[i]; }
    mx /= W; my /= W;

    double Cxx = 0, Cyy = 0, Cxy = 0;
    for (int i = 0; i < n; ++i)
    {
        double dx = icr_x_[i] - mx, dy = icr_y_[i] - my;
        Cxx += icr_w_[i] * dx * dx; Cyy += icr_w_[i] * dy * dy; Cxy += icr_w_[i] * dx * dy;
    }
    Cxx /= W; Cyy /= W; Cxy /= W;

    double ang = 0.5 * std::atan2(2.0 * Cxy, Cxx - Cyy); // axle (major-axis) direction
    double ax = std::cos(ang), ay = std::sin(ang);       // axle unit vector
    double hx = -ay, hy = ax;                            // heading = perpendicular

    // line-fit quality: RMS perpendicular distance = sqrt(smaller eigenvalue)
    double diff = Cxx - Cyy;
    double disc = std::sqrt(diff * diff + 4.0 * Cxy * Cxy);
    double lambda_min = 0.5 * ((Cxx + Cyy) - disc);
    double rms_perp = std::sqrt(std::max(0.0, lambda_min));

    // --- marker midpoint -> along-axle position ---
    double Mx = 0.5 * (ma_x_ + mb_x_);
    double My = 0.5 * (ma_y_ + mb_y_);
    double tproj = (Mx - mx) * ax + (My - my) * ay;      // project midpoint onto axle
    off_x_ = mx + tproj * ax;
    off_y_ = my + tproj * ay;

    // Set orientation assuming marker A is on the left
    double lx = ma_x_ - mb_x_;
    double ly = ma_y_ - mb_y_;
    double fwd_mx =  ly;
    double fwd_my = -lx;
    if (hx * fwd_mx + hy * fwd_my < 0.0) { hx = -hx; hy = -hy; }
    off_yaw_ = std::atan2(hy, hx);

    calibrated_ = true;

    char buf[700];
    std::snprintf(buf, sizeof(buf),
        "base_link calibration for '%s'\n"
        "  samples used     : %d\n"
        "  axle-fit RMS      : %.2f mm (perp. spread of ICR cloud)\n"
        "  body->base_link   : x=%.4f m  y=%.4f m  yaw=%.3f deg\n"
        "  marker midpoint   : (%.4f, %.4f) m  -> along-axle proj %.4f m\n"
        "  static_transform_publisher:\n"
        "    %.4f %.4f 0 0 0 %.5f /%s /%s/base_link",
        target_body_.c_str(), n, rms_perp * 1000.0,
        off_x_, off_y_, off_yaw_ * 180.0 / M_PI,
        Mx, My, tproj,
        off_x_, off_y_, off_yaw_, target_body_.c_str(), target_body_.c_str());
    report = buf;
    ROS_INFO("\n%s", report.c_str());
    return true;
}

bool BaseLinkEstimator::computeSrv(std_srvs::Trigger::Request &,
                                   std_srvs::Trigger::Response &res)
{
    res.success = compute(res.message);
    return true;
}

void BaseLinkEstimator::PublishIfReady(const std::string &body,
                                       double px, double py, double pz,
                                       double qx, double qy, double qz, double qw,
                                       const std::string &global_frame)
{
    if (!enabled_ || body != target_body_)
        return;

    std::lock_guard<std::mutex> lk(mtx_);
    if (!calibrated_)
        return;

    // body -> base_link (constant offset, planar)
    tf2::Quaternion q_body(qx, qy, qz, qw);
    tf2::Quaternion q_off; q_off.setRPY(0, 0, off_yaw_);
    tf2::Transform T_body(q_body, tf2::Vector3(px, py, pz));
    tf2::Transform T_off(q_off, tf2::Vector3(off_x_, off_y_, 0.0));
    tf2::Transform T_base = T_body * T_off;

    // tf: <body>/base_link as child of the body frame
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = body;
    tf.child_frame_id = body + "/base_link";
    tf.transform.translation.x = off_x_;
    tf.transform.translation.y = off_y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q_off.x();
    tf.transform.rotation.y = q_off.y();
    tf.transform.rotation.z = q_off.z();
    tf.transform.rotation.w = q_off.w();
    br.sendTransform(tf);

    // PoseStamped of base_link in the global frame
    tf2::Quaternion q_base = T_base.getRotation();
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = global_frame;
    msg.pose.position.x = T_base.getOrigin().x();
    msg.pose.position.y = T_base.getOrigin().y();
    msg.pose.position.z = T_base.getOrigin().z();
    msg.pose.orientation.x = q_base.x();
    msg.pose.orientation.y = q_base.y();
    msg.pose.orientation.z = q_base.z();
    msg.pose.orientation.w = q_base.w();
    pose_pub_.publish(msg);
}
