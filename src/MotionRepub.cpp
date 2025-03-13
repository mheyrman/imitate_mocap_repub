#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_msgs/TFMessage.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <thread>
#include <vector>
#include <unordered_map>
#include <string>
#include <Eigen/Dense>



class MotionRepub {
private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher motion_repub;
    ros::Subscriber mocap_sub;

    // std::string body_frame_str;
    std::vector<int> foot_frames_ids;
    std::vector<int> shoulder_frames_ids;

    std::unordered_map<int, int> foot_frame_num;
    std::unordered_map<int, int> shoulder_frame_num;

    std::vector<double> body_pos = {0, 0, 0};       // x, y, z
    std::vector<double> body_quat = {1, 0, 0, 0};   // w, x, y, z
    std::vector<std::vector<double>> shoulder_pos;      // [shoulder_num][x, y, z]
    std::vector<std::vector<double>> foot_pos;      // [foot_num][x, y, z]

    std::vector<double> quatToProjGrav(std::vector<double> q) {
        std::vector<double> grav = {0.0, 0.0, -1.0};
        // rotate grav by inverse of quaternion q
        Eigen::Quaterniond quat(q[0], q[1], q[2], q[3]);
        Eigen::Vector3d grav_vec(grav[0], grav[1], grav[2]);
        Eigen::Vector3d rotated_grav = quat.inverse() * grav_vec;
        return {rotated_grav[0], rotated_grav[1], rotated_grav[2]};
    }
    
    std::vector<double> transformW2B(std::vector<double> base_p_w, std::vector<double> base_q_w, std::vector<double> p_w) {
        Eigen::Quaterniond base_q(base_q_w[0], base_q_w[1], base_q_w[2], base_q_w[3]);
        Eigen::Vector3d base_p(base_p_w[0], base_p_w[1], base_p_w[2]);
        Eigen::Vector3d p(p_w[0], p_w[1], p_w[2]);

        Eigen::Vector3d p_b = base_q.inverse() * (p - base_p);
        return {p_b[0], p_b[1], p_b[2]};
    }

    void calculateBodyPosQuat() {
        // calculate body position as average of shoulder positions
        // calculate body quaternion based on shoulder positions given order FL, BL, FR, BR
        body_pos[0] = (shoulder_pos[0][0] + shoulder_pos[1][0] + shoulder_pos[2][0] + shoulder_pos[3][0]) / 4;
        body_pos[1] = (shoulder_pos[0][1] + shoulder_pos[1][1] + shoulder_pos[2][1] + shoulder_pos[3][1]) / 4;
        body_pos[2] = (shoulder_pos[0][2] + shoulder_pos[1][2] + shoulder_pos[2][2] + shoulder_pos[3][2]) / 4;

        // calculate body quaternion
        Eigen::Vector3d fl = {shoulder_pos[0][0], shoulder_pos[0][1], shoulder_pos[0][2]};
        Eigen::Vector3d rl = {shoulder_pos[1][0], shoulder_pos[1][1], shoulder_pos[1][2]};
        Eigen::Vector3d fr = {shoulder_pos[2][0], shoulder_pos[2][1], shoulder_pos[2][2]};
        Eigen::Vector3d rr = {shoulder_pos[3][0], shoulder_pos[3][1], shoulder_pos[3][2]};

        Eigen::Vector3d x_axis = (fl - fr).normalized();
        Eigen::Vector3d y_axis = (fl - rl).normalized();
        Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();
        y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix3d rot_mat;
        rot_mat << x_axis[0], y_axis[0], z_axis[0],
                   x_axis[1], y_axis[1], z_axis[1],
                   x_axis[2], y_axis[2], z_axis[2];
        
        Eigen::Quaterniond quat(rot_mat);
        body_quat = {quat.w(), quat.x(), quat.y(), quat.z()};
    }
    
public:
    MotionRepub() : tfListener(tfBuffer) {
        motion_repub = nh.advertise<std_msgs::Float64MultiArray>("/reference_motion", 1);
        mocap_sub = nh.subscribe("/qualisys/no_labels_marker_array", 1, &MotionRepub::mocapCallback, this);

        // body_frame_str = "2616";

        // FL, RL, FR, RR
        foot_frames_ids = {5820, 2857, 5822, 5817};
        shoulder_frames_ids = {5784, 5799, 5795, 5798};
        for (int i = 0; i < foot_frames_ids.size(); i++) {
            foot_frame_num[foot_frames_ids[i]] = i;
            foot_pos.push_back(std::vector<double>(3, 0));
        }
        for (int i = 0; i < shoulder_frames_ids.size(); i++) {
            shoulder_frame_num[shoulder_frames_ids[i]] = i;
            shoulder_pos.push_back(std::vector<double>(3, 0));
        }
    }

    // void mocapCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    //     for (const auto& transform : msg->transforms) {
    //         // ROS_INFO_STREAM("child_frame_id: " << transform.child_frame_id);
    //         if (transform.child_frame_id == body_frame_str) {
    //             body_pos = {transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z};
    //             body_quat = {transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w};
    //         }
    //         else if (std::find(foot_frames_str.begin(), foot_frames_str.end(), transform.child_frame_id) != foot_frames_str.end()) {
    //             int foot_index = foot_frame_num[transform.child_frame_id];

    //             std::vector<double> foot_pos_w = {transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z};
    //             std::vector<double> foot_pos_b = transformW2B(body_pos, body_quat, foot_pos_w);
    //             foot_pos[foot_index] = foot_pos_b;
    //         }
    //     }
    // }

    void mocapCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        for (const auto& marker : msg->markers) {
            // ROS_INFO_STREAM("child_frame_id: " << marker.header.frame_id);
            if (std::find(shoulder_frames_ids.begin(), shoulder_frames_ids.end(), marker.id) != shoulder_frames_ids.end()) {
                // body_pos = {marker.pose.position.x, marker.pose.position.y, marker.pose.position.z};
                // body_quat = {marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z};
                int shoulder_index = shoulder_frame_num[marker.id];

                std::vector<double> shoulder_pos_w = {marker.pose.position.x, marker.pose.position.y, marker.pose.position.z};
                shoulder_pos[shoulder_index] = shoulder_pos_w;

                calculateBodyPosQuat();
            }
            else if (std::find(foot_frames_ids.begin(), foot_frames_ids.end(), marker.id) != foot_frames_ids.end()) {
                int foot_index = foot_frame_num[marker.id];

                std::vector<double> foot_pos_w = {marker.pose.position.x, marker.pose.position.y, marker.pose.position.z};
                std::vector<double> foot_pos_b = transformW2B(body_pos, body_quat, foot_pos_w);
                foot_pos[foot_index] = foot_pos_b;
            }
        }
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