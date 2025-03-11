#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_msgs/TFMessage.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float64MultiArray.h>

#include <thread>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>



class MotionRepub {
private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher motion_repub;
    ros::Subscriber mocap_sub;

    std::string body_frame_str;
    std::vector<std::string> foot_frames_str;

    std::unordered_map<std::string, int> foot_frame_num;

    std::vector<double> body_pos = {0, 0, 0};       // x, y, z
    std::vector<double> body_quat = {1, 0, 0, 0};   // w, x, y, z
    std::vector<std::vector<double>> foot_pos;      // [foot_num][x, y, z]

public:
    MotionRepub() : tfListener(tfBuffer) {
        motion_repub = nh.advertise<std_msgs::Float64MultiArray>("/reference_motion", 1);
        mocap_sub = nh.subscribe("/tf", 1, &MotionRepub::mocapCallback, this);

        body_frame_str = "RigidBody3";
        foot_frames_str = {"RigidBody4"};
        for (int i = 0; i < foot_frames_str.size(); i++) {
            foot_frame_num[foot_frames_str[i]] = i;
            foot_pos.push_back(std::vector<double>(3, 0));
        }
    }

    void mocapCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
        for (const auto& transform : msg->transforms) {
            ROS_INFO_STREAM("child_frame_id: " << transform.child_frame_id);
            if (transform.child_frame_id == body_frame_str) {
                body_pos = {transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z};
                body_quat = {transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w};
            }
            else if (std::find(foot_frames_str.begin(), foot_frames_str.end(), transform.child_frame_id) != foot_frames_str.end()) {
                int foot_index = foot_frame_num[transform.child_frame_id];
                
                foot_pos[foot_index] = {
                    transform.transform.translation.x - body_pos[0],
                    transform.transform.translation.y - body_pos[1],
                    transform.transform.translation.z - body_pos[2]
                };
            }
        }
    }

    std::vector<double> quatToProjGrav(std::vector<double> q) {
        std::vector<double> grav = {0.0, 0.0, -1.0};
        // rotate grav by inverse of quaternion q
        Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
        Eigen::Vector3d grav_vec(grav[0], grav[1], grav[2]);
        Eigen::Vector3d rotated_grav = quat.inverse() * grav_vec;
        return {rotated_grav[0], rotated_grav[1], rotated_grav[2]};
    }

    void publishMotionData() {
        ros::Rate rate(50);
        while (ros::ok()) {
            std_msgs::Float64MultiArray motion_msg;

            std::vector<double> proj_grav = quatToProjGrav(body_quat);

            for (int i = 0; i < foot_pos.size(); i++) {
                motion_msg.data.push_back(foot_pos[i][0]);
                motion_msg.data.push_back(foot_pos[i][1]);
                motion_msg.data.push_back(foot_pos[i][2]);
            }
            motion_msg.data.push_back(proj_grav[0]);
            motion_msg.data.push_back(proj_grav[1]);
            motion_msg.data.push_back(proj_grav[2]);
            motion_msg.data.push_back(body_pos[2]);

            motion_repub.publish(motion_msg);
            rate.sleep();
        }
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_repub");
    MotionRepub motion_repub;

    std::thread motion_thread(&MotionRepub::publishMotionData, &motion_repub);
    ros::spin();
    motion_thread.join();
    return 0;
}