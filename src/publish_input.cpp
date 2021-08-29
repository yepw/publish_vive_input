#include <netinet/in.h>
#include <poll.h>
#include <thread>
#include <math.h>
#include <sstream>
#include <unistd.h>
#include <termios.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <ros_server/EEPoseGoals.h>
#include <publish_vive_input/ButtonInfo.h>
#include <publish_vive_input/ControllerInfo.h>
#include <publish_vive_input/ControllersInput.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "nlohmann/json.hpp"
#include "publish_vive_input/utilities.hpp"
#include "publish_vive_input/publish_input.hpp"

using json = nlohmann::json;
using EEPoseGoals = ros_server::EEPoseGoals;
using ButtonInfo = publish_vive_input::ButtonInfo;
using ControllerInfo = publish_vive_input::ControllerInfo;
using ControllersInput = publish_vive_input::ControllersInput;
using App = vive_input::App;

#define LOOP_RATE 60

namespace vive_input {

    ContrCommands translateInputToCommand(std::string button)
    {
        if (button == "pose") {
            return ContrCommands::POSE;
        }
        else if (button == "trigger") {
            return ContrCommands::GRAB;
        }
        else if (button == "gripper") {
            return ContrCommands::RESET;
        }
        else if (button == "menu") {
            return ContrCommands::CLUTCH;
        }
        else if (button == "trackpad") {
            return ContrCommands::OFFSET;
        }

        return ContrCommands::NONE;
    }

    bool initializeSocket(Socket &sock, bool incoming)
    {
        if ((sock.socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
            printText("Could not initialize socket.");
            return false;
        }

        memset(&sock.address, 0, sizeof(sock.address));
        sock.address.sin_family = AF_INET; 
        sock.address.sin_addr.s_addr = INADDR_ANY;
        sock.address.sin_port = htons(sock.port);

        if (incoming) {
            if (bind(sock.socket, (const sockaddr *)&sock.address, sizeof(sock.address)) < 0) { 
                printText("Socket binding failed.");
                sock.socket = 0;
                return false;
            }
        }
        else {
            if (connect(sock.socket, (struct sockaddr *)&sock.address, sizeof(sock.address)) < 0) { 
                std::string err_str = strerror(errno);
                printText("Socket connection failed.");
                printText("Error: " + err_str);
                sock.socket = 0;
                return false; 
            }
        }

        return true;
    }

    bool App::init()
    {
        // Init ROS
        ros::NodeHandle n;
        ee_pub = n.advertise<ros_server::EEPoseGoals>("ee_pose_goals", 10);
        reset_pub = n.advertise<std_msgs::Bool>("/relaxed_ik/reset", 10);
        grasper_pub = n.advertise<std_msgs::Bool>("/robot_state/grasping", 10);
        clutching_pub = n.advertise<std_msgs::Bool>("/robot_state/clutching", 10);

        toggle_pub = n.advertise<std_msgs::Bool>("/relaxed_ik/toggle", 10);
        controller_raw_pub = n.advertise<ControllersInput>("/vive_input/raw_data", 10);
        controller_raw_string_pub = n.advertise<std_msgs::String>("/vive_input/raw_string", 10);
        // TODO: Add raw input publishing
        // Add sub/pub for raw character input

        manual_reset = n.subscribe("/vive_input/manual_reset", 10, &App::triggerManualReset, this);

        // Init sockets

        // Make sure that this matches the Vive params file and that it's not
        // the same as the out port
        in_socket.port = 8081;
        out_socket.port = 8080;

        if (!initializeSocket(in_socket)) {
            return false;
        }

        if (!initializeSocket(out_socket, false)) {
            return false;
        }

        return true;
    }

    std::string getSocketData(Socket &sock)
    {
        int len_data;
        len_data = recv(sock.socket, sock.buffer, sock.DATA_SIZE, 0); 
        while (len_data == -1 && ros::ok())
        {
            len_data = recv(sock.socket, sock.buffer, sock.DATA_SIZE, 0);   
        }
        sock.buffer[len_data] = '\0';
        std::string data = sock.buffer;

        return data;
    }

    inline glm::quat quaternionDisplacement(const glm::quat &q1, const glm::quat &q2)
    {
        return glm::inverse(q1) * q2;
    }

    glm::quat rotateQuaternionByMatrix(glm::quat q, glm::mat3 R)
    {
        float angle(glm::angle(q));
        glm::vec3 axis(glm::axis(q));

        glm::vec3 out_imag((float)glm::sin(angle / 2.0) * (R * axis));
        float out_real(glm::cos(angle / 2.0));

        return glm::quat(out_real, out_imag.x, out_imag.y, out_imag.z);
    }

    glm::quat quaternionMultiplication(glm::quat q1, glm::quat q2)
    {
        glm::quat result;
        result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
        result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
        result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
        result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;

        return result;      
    }

    inline glm::vec3 updatePosition(const glm::vec3 &prev_p, const glm::vec3 &input_vel, 
            const glm::mat3 &r_cam)
    {
        return prev_p + r_cam*input_vel;
    }

    glm::vec3 rotatePositionByQuaternion(glm::vec3 pos, glm::quat q)
    {
        glm::quat q_inverse(glm::inverse(q));
        glm::quat p(0.0, pos.x, pos.y, pos.z);
        glm::quat p_prime((q * p) * q_inverse);

        return glm::vec3(p_prime.x, p_prime.y, p_prime.z);
    }


    glm::vec3 positionToUR5Frame(glm::vec3 v)
    {
        return glm::vec3(-v.z, -v.x, v.y);
        // return glm::vec3(v.x, v.y, v.z);
        //return glm::vec3(-v.z, v.x, -v.y);
    }

    glm::quat orientationToUR5Frame(glm::quat q)
    {
        // glm::vec3 new_euler(glm::yaw(q), glm::pitch(q), -glm::roll(q));
        //glm::vec3 new_euler(glm::pitch(q), glm::yaw(q), glm::roll(q));
        glm::quat new_quat(q.w, -q.z, -q.x, q.y);

        return new_quat;
        // return q;
    }

    void App::triggerManualReset(std_msgs::Bool msg)
    {
        input.manual_reset = msg.data;
    }

    void App::resetPose(glm::vec3 new_pos, glm::quat new_orient)
    {
        input.prev_raw_pos = new_pos;
        input.prev_ee_pos = glm::vec3(0.0, 0.0, 0.0);
        input.prev_raw_orient = new_orient;
        input.prev_ee_orient = glm::quat(1.0, 0.0, 0.0, 0.0);

        input.init_raw_orient = new_orient;

        input.cur_outer_cone = input.kStartingOuterCone;
        input.cur_distance = input.kStartingDistance;

        input.reset_cam = true;
    }

    void App::handleControllerInput(std::string data)
    {
        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        json j(json::parse(data));
        json out_msg;

        glm::vec3 cur_raw_pos, cam_raw_pos;
        glm::vec3 cur_cam_offset(0.0);
        glm::quat cur_raw_orient;
        bool right_contr_active(false), left_contr_active(false);

        ControllersInput contr_input;
        ControllerInfo right_contr; right_contr.role = "right";
        ControllerInfo left_contr; left_contr.role = "left";

        for (json::iterator contr_it(j.begin()); contr_it != j.end(); ++contr_it) {
            std::string controller(contr_it.key());
            if (j[controller]["_role"] == "right") {
                // --- Right controller - main control and commands ---
                right_contr_active = true;
                for (json::iterator button_it(contr_it->begin()); button_it != contr_it->end(); ++button_it) {
                    if (button_it->is_object()) {
                        std::string button(button_it.key());
                        ContrCommands command(translateInputToCommand(button));

                        switch (command)
                        {
                            case ContrCommands::POSE:
                            {
                                auto pos(j[controller][button]["position"]);
                                cur_raw_pos = glm::vec3(pos["x"], pos["y"], pos["z"]);
                                auto quat(j[controller][button]["orientation"]);
                                cur_raw_orient = glm::normalize(glm::quat(quat["w"], quat["x"], quat["y"], quat["z"]));

                                if (!input.initialized) {
                                    resetPose(cur_raw_pos, cur_raw_orient);
                                    input.initialized = true;
                                }

                                right_contr.pose.position.x = cur_raw_pos.x;
                                right_contr.pose.position.y = cur_raw_pos.y;
                                right_contr.pose.position.z = cur_raw_pos.z;

                                right_contr.pose.orientation.x = cur_raw_orient.x;
                                right_contr.pose.orientation.y = cur_raw_orient.y;
                                right_contr.pose.orientation.z = cur_raw_orient.z;
                                right_contr.pose.orientation.w = cur_raw_orient.w;

                            }   break;

                            case ContrCommands::GRAB:
                            {
                                bool raw_input(j[controller][button]["boolean"] and j[controller][button]["pressure"] == 1.0 );
                                // input.grabbing = !raw_input;
                                input.grabbing = raw_input;

                                right_contr.button1.name = "grab";
                                right_contr.button1.has_boolean = true;
                                right_contr.button1.boolean = raw_input;

                            }   break;

                            case ContrCommands::RESET:
                            {
                                bool raw_input(j[controller][button]["boolean"]);
                                if (input.manual_reset) {
                                    input.reset = true;
                                    input.manual_reset = false; // Reset manual_reset variable
                                }
                                else {
                                    input.reset = raw_input;
                                }
                                input.reset_cam = input.reset.is_on();

                                right_contr.button2.name = "reset";
                                right_contr.button2.has_boolean = true;
                                right_contr.button2.boolean = raw_input;

                            }   break;

                            case ContrCommands::CLUTCH:
                            {
                                bool raw_input(j[controller][button]["boolean"]);
                                input.clutching = raw_input;

                                right_contr.button3.name = "clutch";
                                right_contr.button3.has_boolean = true;
                                right_contr.button3.boolean = raw_input;

                            }   break;

                            case ContrCommands::OFFSET:
                            {
                                bool raw_bool(j[controller][button]["boolean"]);
                                input.toggle = raw_bool;
                                // input.manual_offset.x = j[controller][button]["2d"]["x"];
                                // input.manual_offset.y = j[controller][button]["2d"]["y"];
                                // input.manual_offset.z = 0.0;

                                if (input.toggle.confirm_flip_on()) {
                                    out_msg["toggle"] = true;

                                    std_msgs::Bool toggle;
                                    toggle.data = out_msg["toggle"];
                                    toggle_pub.publish(toggle);
                                }

                                // Available commands when not in clutching mode
                                // if (!input.clutching.is_on() && input.manual_adj.confirm_flip_on()) {
                                //     if (input.manual_offset.x >= 0.5) {
                                //         out_msg["primary_next"] = true;
                                //     }
                                //     else if (input.manual_offset.x <= -0.5) {
                                //         out_msg["primary_prev"] = true;
                                //     }
                                //     else if (input.manual_offset.y >= 0.5) {
                                //         out_msg["pip_prev"] = true;
                                //     }
                                //     else if (input.manual_offset.y <= -0.5) {
                                //         out_msg["pip_next"] = true;
                                //     }
                                //     else {
                                //         out_msg["pip_toggle"] = true;
                                //     }
                                // }

                                right_contr.button4.name = "trackpad";
                                right_contr.button4.has_boolean = true;
                                right_contr.button4.boolean = raw_bool;

                            }   break;
                        
                            default:
                            {
                                out_msg[button] = button_it.value();
                            }   break;
                        }
                    }

                }
                // --- Right controller ---
            }
            else if (j[contr_it.key()]["_role"] == "left") {
                // --- Left controller - camera adjustments ---
                left_contr_active = true;
                for (json::iterator button_it(contr_it->begin()); button_it != contr_it->end(); ++button_it) {
                    if (button_it->is_object()) {
                        std::string button(button_it.key());
                        ContrCommands command(translateInputToCommand(button));

                        switch (command)
                        {
                            case ContrCommands::POSE:
                            {
                                auto pos(j[controller][button]["position"]);
                                cam_raw_pos = glm::vec3(pos["x"], pos["y"], pos["z"]);

                                if (!input.cam_offset_init) {
                                    input.cam_init_raw_pos = cam_raw_pos;
                                    input.cam_offset_init = true;
                                }

                                cur_cam_offset = cam_raw_pos - input.cam_init_raw_pos;

                                left_contr.pose.position.x = cam_raw_pos.x;
                                left_contr.pose.position.y = cam_raw_pos.y;
                                left_contr.pose.position.z = cam_raw_pos.z;

                                auto quat(j[controller][button]["orientation"]);
                                glm::quat raw_orient = glm::normalize(glm::quat(quat["w"], quat["x"], quat["y"], quat["z"]));

                                left_contr.pose.orientation.x = raw_orient.x;
                                left_contr.pose.orientation.y = raw_orient.y;
                                left_contr.pose.orientation.z = raw_orient.z;
                                left_contr.pose.orientation.w = raw_orient.w;

                            }   break;

                            case ContrCommands::RESET:
                            {
                                bool raw_input(j[controller][button]["boolean"]);
                                // Ignore L controller if already getting reset signal from R controller
                                if (!input.reset.is_on()) {
                                    input.reset_cam = raw_input;
                                }

                                left_contr.button1.name = "reset";
                                left_contr.button1.has_boolean = true;
                                left_contr.button1.boolean = raw_input;

                            }   break;
                        }
                    }
                }

                if (input.reset_cam.confirm_flip_off()) {
                    input.cam_init_raw_pos = cam_raw_pos;
                }
                // --- Left controller ---
            }
        }

        // Since right controller is main, we need its pose data for useful input
        if (!right_contr_active) {
            return;
        }

        if (input.clutching.confirm_flip_off()) {
            input.reset_cam = true;
        }

        if (input.reset.confirm_flip_off()) {
            resetPose(cur_raw_pos, cur_raw_orient);
        }

        // Publish the pose as normal
        if (!input.clutching.is_on() && !input.reset.is_on()) {
            // Get the camera pose matrix
            // glm::mat3 cam_rot_mat(1.0);
            glm::mat3 cam_rot_mat(input.cam_rot_mat);

            // Calculate camera offset
            input.camera_offset = cam_rot_mat * cur_cam_offset;

            if (glm::length(input.camera_offset) > 0.1) {
                input.camera_control = true;
            }
            else {
                input.camera_control = false;
            }

            // Calculate new position
            glm::vec3 input_vel(cur_raw_pos - input.prev_raw_pos);
            input.cur_ee_pos = updatePosition(input.prev_ee_pos, input_vel, cam_rot_mat);
            input.out_pos = positionToUR5Frame(input.cur_ee_pos);
            // input.out_pos = input.cur_ee_pos;

            // Calculate new orientation
            glm::quat q_v1(glm::normalize(rotateQuaternionByMatrix(input.prev_raw_orient, glm::mat3_cast(cur_raw_orient))));
            glm::quat q_v(rotateQuaternionByMatrix(quaternionDisplacement(q_v1, cur_raw_orient), cam_rot_mat));
            // q_v is nan when there is no change
            if (!std::isnan(q_v.w)) { 
                input.cur_ee_orient = quaternionMultiplication(q_v, input.prev_ee_orient);
                input.out_orient = orientationToUR5Frame(input.cur_ee_orient);
                // input.out_orient = input.cur_ee_orient;
                // input.out_orient = glm::quat(1.0, 0.0, 0.0, 0.0);
            }
        }

        input.prev_raw_pos = cur_raw_pos;
        input.prev_ee_pos = input.cur_ee_pos;
        input.prev_raw_orient = cur_raw_orient;
        input.prev_ee_orient = input.cur_ee_orient;

        // std::cout << "Rot_mat: " << glm::to_string(input.cam_rot_mat) << std::endl;
        // printText(input.to_str());

        publishRobotData();

        if (!out_msg.is_null()) {
            std::string output(out_msg.dump(3));
            send(out_socket.socket, output.c_str(), output.size(), 0);
        }

        // Publish raw input as string and as ControllersInput message
        std_msgs::String raw_controller_input;
        raw_controller_input.data = j.dump(3);
        controller_raw_string_pub.publish(raw_controller_input);

        right_contr.active = true; // Already validated above
        if (left_contr_active) {
            left_contr.active = true;
        }

        contr_input.header.stamp = ros::Time::now();
        contr_input.controllers.push_back(right_contr);
        contr_input.controllers.push_back(left_contr);
        controller_raw_pub.publish(contr_input);
    }

    void App::publishRobotData()
    {
        EEPoseGoals goal;
        // End-effector pose goal
        geometry_msgs::Pose pose;
        pose.position.x = input.out_pos.x;
        pose.position.y = input.out_pos.y;
        pose.position.z = input.out_pos.z;

        // pose.orientation.x = input.out_orient.x;
        // pose.orientation.y = input.out_orient.y;
        // pose.orientation.z = input.out_orient.z;
        // pose.orientation.w = input.out_orient.w;

        pose.orientation.x = 0.;
        pose.orientation.y = 0.;
        pose.orientation.z = 0.;
        pose.orientation.w = 1.;

        goal.header.stamp = ros::Time::now();
        goal.ee_poses.push_back(pose);

        std_msgs::Bool reset;
        reset.data = input.reset.is_on();

        ee_pub.publish(goal);
        reset_pub.publish(reset);


        std_msgs::Bool grabbing, clutching;
        grabbing.data = input.grabbing.is_on();
        clutching.data = input.clutching.is_on();

        grasper_pub.publish(grabbing);
        clutching_pub.publish(clutching);        
    }


    int App::run()
    {
        if (!init()) {
            return 1;
        }
        
        pollfd poll_fds;
        poll_fds.fd = in_socket.socket;
        poll_fds.events = POLLIN; // Wait until there's data to read

        spinner.start();

        while (ros::ok())
        {
            if (poll(&poll_fds, 1, int(1.0 / LOOP_RATE)) > 0) {
                std::string input_data(getSocketData(in_socket));
                handleControllerInput(input_data);
            }
        }

        spinner.stop();

        shutdown(in_socket.socket, SHUT_RDWR);
        shutdown(out_socket.socket, SHUT_RDWR);
        
        return 0;
    }

} // namespace vive_input


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publish_vive_input");

    App app;
    return app.run();
}