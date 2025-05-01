#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_msgs/TFMessage.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <thread>
#include <vector>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>
#include <yaml_tools/YamlNode.hpp>



class MotionRepub {
private:
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ros::Publisher motion_repub;
    ros::Subscriber mocap_sub;

    std::vector<std::string> foot_frames_ids;
    std::vector<std::string> shoulder_frames_ids;

    std::unordered_map<std::string, int> foot_frame_num;
    std::unordered_map<std::string, int> shoulder_frame_num;

    Eigen::Vector3d body_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond body_quat = {1.0, 0.0, 0.0, 0.0};
    Eigen::Vector3d proj_grav = {0.0, 0.0, -1.0};
    std::vector<Eigen::Vector3d> shoulder_pos;      // [shoulder_num][x, y, z]
    std::vector<Eigen::Vector3d> shoulder_offset;
    std::vector<Eigen::Vector3d> foot_pos;          // [foot_num][x, y, z]
    std::vector<Eigen::Vector3d> foot_offset;
    std::vector<Eigen::Quaterniond> foot_quat;      // [foot_num][w, x, y, z]

    double t_received = 0.0;

    yaml_tools::YamlNode yamlNode = yaml_tools::YamlNode::fromFile(
        ros::package::getPath("imitate_mocap_repub") + "/config" + "/frame_cfg.yaml"
    );

    Eigen::Vector3d quatToProjGrav(Eigen::Quaterniond q) {
        Eigen::Vector3d grav_vec = {0.0, 0.0, -1.0};
        Eigen::Vector3d rotated_grav = q.inverse() * grav_vec;
        return rotated_grav;
    }

    Eigen::Vector3d offsetWorldCoordinates(Eigen::Vector3d p, Eigen::Vector3d offset) {
        return p + body_quat * offset;
    }
    
    Eigen::Vector3d transformW2B(Eigen::Vector3d base_p, Eigen::Quaterniond base_q, Eigen::Vector3d p) {
        Eigen::Matrix3d rotation_matrix = base_q.toRotationMatrix();
        Eigen::Vector3d p_t_b = p - base_p;
        Eigen::Vector3d p_b = rotation_matrix.transpose() * p_t_b;
        return p_b;
    }
    
    /* 
    How Dongho does it:
    {
        // compute frame by fitting plane
        crl::P3D rightShoulder = sk->getMarkerByName("RightShoulder")->state.getWorldCoordinates(crl::P3D());
        crl::P3D leftShoulder = sk->getMarkerByName("LeftShoulder")->state.getWorldCoordinates(crl::P3D());
        crl::P3D rightUpLeg = sk->getMarkerByName("RightUpLeg")->state.getWorldCoordinates(crl::P3D());
        crl::P3D leftUpLeg = sk->getMarkerByName("LeftUpLeg")->state.getWorldCoordinates(crl::P3D());
        
        vvvvvvv HOW HE CALCULATES BASE POS, BASE QUAT vvvvvvv

        crl::P3D shoulderMid = (rightShoulder + leftShoulder) * 0.5;  // front
        crl::P3D upLegMid = (rightUpLeg + leftUpLeg) * 0.5;           // rear
        crl::P3D bodyFrameOrigin = (shoulderMid + upLegMid) * 0.5;
        crl::V3D xAxis(shoulderMid, upLegMid);  // front to rear
        crl::V3D z1(rightShoulder, leftShoulder);
        crl::V3D z2(rightUpLeg, leftUpLeg);
        crl::V3D yAxis = (z1 + z2).cross(xAxis);

        xAxis.normalize();
        yAxis.normalize();
        crl::V3D zAxis = xAxis.cross(yAxis);
        zAxis.normalize();

        crl::Matrix3x3 R;
        R << xAxis.x(), yAxis.x(), zAxis.x(), xAxis.y(), yAxis.y(), zAxis.y(), xAxis.z(), yAxis.z(), zAxis.z();
        if (considerNominalPose)
            bodyFrames.push_back({bodyFrameOrigin, rStar.inverse() * crl::Quaternion(R)});
        else
            bodyFrames.push_back({bodyFrameOrigin, crl::Quaternion(R)});
    }

    Offset is calculated in world frame from a set offset in the .BVH files
    Mocap points are on the joints (shoulder, wrist) and offset defined in BVH and applied w/ getWorldCoordinates

        Note: Dongho does not offset the shoulders, Y up coordinate system
    
    */

    void calculateBodyPosQuat() {
        // calculate body position as average of shoulder positions
        Eigen::Vector3d fl = shoulder_pos[0];
        Eigen::Vector3d rl = shoulder_pos[1];
        Eigen::Vector3d fr = shoulder_pos[2];
        Eigen::Vector3d rr = shoulder_pos[3];

        body_pos = (fl + fr + rl + rr) / 4;
        // body_pos[2] -= 0.5;    // box dog height offset

        // calculate body quaternion
        Eigen::Vector3d front_mid = (fl + fr) / 2;
        Eigen::Vector3d rear_mid = (rl + rr) / 2;
        Eigen::Vector3d left_mid = (fl + rl) / 2;
        Eigen::Vector3d right_mid = (fr + rr) / 2;

        Eigen::Vector3d x_axis = (front_mid - rear_mid).normalized();
        Eigen::Vector3d y_axis = (left_mid - right_mid).normalized();
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
        // mocap_sub = nh.subscribe("/qualisys/no_labels_marker_array", 1, &MotionRepub::mocapCallback, this);
        mocap_sub = nh.subscribe("/tf", 1, &MotionRepub::mocapCallback, this);

        foot_frames_ids = {
            yamlNode["frame_info"]["foot_frame_ids"]["FL"].as<std::string>(),
            yamlNode["frame_info"]["foot_frame_ids"]["RL"].as<std::string>(),
            yamlNode["frame_info"]["foot_frame_ids"]["FR"].as<std::string>(),
            yamlNode["frame_info"]["foot_frame_ids"]["RR"].as<std::string>()
        };
        shoulder_frames_ids = {
            yamlNode["frame_info"]["shoulder_frame_ids"]["FL"].as<std::string>(),
            yamlNode["frame_info"]["shoulder_frame_ids"]["RL"].as<std::string>(),
            yamlNode["frame_info"]["shoulder_frame_ids"]["FR"].as<std::string>(),
            yamlNode["frame_info"]["shoulder_frame_ids"]["RR"].as<std::string>()
        };

        foot_offset = {
            Eigen::Vector3d(
            yamlNode["frame_info"]["foot_offset"]["FL"][0].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["FL"][1].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["FL"][2].as<double>()
            ),
            Eigen::Vector3d(
            yamlNode["frame_info"]["foot_offset"]["RL"][0].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["RL"][1].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["RL"][2].as<double>()
            ),
            Eigen::Vector3d(
            yamlNode["frame_info"]["foot_offset"]["FR"][0].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["FR"][1].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["FR"][2].as<double>()
            ),
            Eigen::Vector3d(
            yamlNode["frame_info"]["foot_offset"]["RR"][0].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["RR"][1].as<double>(),
            yamlNode["frame_info"]["foot_offset"]["RR"][2].as<double>()
            )
        };

        shoulder_offset = {
            Eigen::Vector3d(
            yamlNode["frame_info"]["shoulder_offset"]["FL"][0].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["FL"][1].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["FL"][2].as<double>()
            ),
            Eigen::Vector3d(
            yamlNode["frame_info"]["shoulder_offset"]["RL"][0].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["RL"][1].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["RL"][2].as<double>()
            ),
            Eigen::Vector3d(
            yamlNode["frame_info"]["shoulder_offset"]["FR"][0].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["FR"][1].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["FR"][2].as<double>()
            ),
            Eigen::Vector3d(
            yamlNode["frame_info"]["shoulder_offset"]["RR"][0].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["RR"][1].as<double>(),
            yamlNode["frame_info"]["shoulder_offset"]["RR"][2].as<double>()
            )
        };

        for (int i = 0; i < foot_frames_ids.size(); i++) {
            foot_frame_num[foot_frames_ids[i]] = i;
            foot_pos.push_back(Eigen::Vector3d::Zero());
        }
        for (int i = 0; i < shoulder_frames_ids.size(); i++) {
            shoulder_frame_num[shoulder_frames_ids[i]] = i;
            shoulder_pos.push_back(Eigen::Vector3d::Zero());
        }
    }

    void mocapCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
        for (const auto& transform : msg->transforms) {
            const std::string& child_frame_id = transform.child_frame_id;
            
            if (child_frame_id[0] == 'S') {
                t_received = 0.0;
                int shoulder_index = shoulder_frame_num[child_frame_id.substr(0, 3)];
                Eigen::Vector3d shoulder_pos_w = Eigen::Vector3d(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                );

                shoulder_pos_w = offsetWorldCoordinates(shoulder_pos_w, shoulder_offset[shoulder_index]);
                shoulder_pos[shoulder_index] = shoulder_pos_w;
                calculateBodyPosQuat();
            } else if (child_frame_id[0] == 'F') {
                t_received = 0.0;
                int foot_index = foot_frame_num[child_frame_id.substr(0, 3)];
                Eigen::Vector3d foot_pos_w = Eigen::Vector3d(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                );

                foot_pos_w = offsetWorldCoordinates(foot_pos_w, foot_offset[foot_index]);
                foot_pos[foot_index] = transformW2B(body_pos, body_quat, foot_pos_w);
            }
        }
    }


    void publishMotionData() {
        ros::Rate rate(50);
        while (ros::ok()) {
            // if not receiving publish empty message
            ROS_INFO_STREAM("t_received: " << t_received);
            if (t_received <= 0.06) {
                std_msgs::Float64MultiArray motion_msg;
    
                proj_grav = quatToProjGrav(body_quat);
    
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
    
                t_received += 0.02;
    
                rate.sleep();
            } else if (t_received <= 0.2) {
                std_msgs::Float64MultiArray motion_msg;
                for (int i = 0; i < 16; i++) {
                    motion_msg.data.push_back(0.0);
                }
                motion_repub.publish(motion_msg);

                t_received += 0.02;

                rate.sleep();
            } else {
                rate.sleep();
            }
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