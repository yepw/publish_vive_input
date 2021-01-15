#include <netinet/in.h>
#include <poll.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <relaxed_ik/EEPoseGoals.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "nlohmann/json.hpp"
#include "publish_vive_input/utilities.hpp"
#include "publish_vive_input/publish_input.hpp"

using json = nlohmann::json;
using Bool = std_msgs::Bool;
using Pose = geometry_msgs::Pose;
using EEPoseGoals = relaxed_ik::EEPoseGoals;
using App = vive_input::App;

#define LOOP_RATE 60

namespace vive_input {

    ContrCommands translateInputToCommand(std::string button)
    {
        if (button == "pose")
        {
            return ContrCommands::POSE;
        }
        else if (button == "trigger")
        {
            return ContrCommands::GRAB;
        }
        else if (button == "gripper")
        {
            return ContrCommands::RESET;
        }
        else if (button == "menu")
        {
            return ContrCommands::CLUTCH;
        }
        else if (button == "trackpad")
        {
            return ContrCommands::OFFSET;
        }

        return ContrCommands::NONE;
    }

    bool initializeSocket(Socket &sock, bool incoming)
    {
        if ((sock.socket = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
        {
            printText("Could not initialize socket.");
            return false;
        }

        memset(&sock.address, 0, sizeof(sock.address));
        sock.address.sin_family = AF_INET; 
        sock.address.sin_addr.s_addr = INADDR_ANY;
        sock.address.sin_port = htons(sock.port);

        if (incoming)
        {
            if (bind(sock.socket, (const sockaddr *)&sock.address, sizeof(sock.address)) < 0)
            { 
                printText("Socket binding failed."); 
                return false;
            }
        }
        else
        {
            if (connect(sock.socket, (struct sockaddr *)&sock.address, sizeof(sock.address)) < 0) 
            { 
                std::string err_str = strerror(errno);
                printText("Socket connection failed.");
                printText("Error: " + err_str);
                return false; 
            }
        }

        return true;
    }

    bool App::init()
    {
        // Init ROS
        ros::NodeHandle n;
        ee_pub = n.advertise<relaxed_ik::EEPoseGoals>("/relaxed_ik/ee_pose_goals", 1000);
        grasper_pub = n.advertise<std_msgs::Bool>("/relaxed_ik/grasper_state", 1000);

        // Init sockets

        // Make sure that this matches the Vive params file and that it's not
        // the same as the out port
        in_socket.port = 8081;

        if (!initializeSocket(in_socket)) 
        {
            return false;
        }

        if (!initializeSocket(out_socket, false)) 
        {
            return false;
        }

        return true;
    }

    std::string getSocketData(Socket &sock)
    {
        int len_data;
        len_data = recvfrom(sock.socket, sock.buffer, sock.DATA_SIZE, MSG_WAITALL, (sockaddr *) &(sock.address), &(sock.len)); 
        while (len_data == -1 && ros::ok())
        {
            len_data = recvfrom(sock.socket, sock.buffer, sock.DATA_SIZE, MSG_WAITALL, (sockaddr *) &(sock.address), &(sock.len));   
        }
        sock.buffer[len_data] = '\0';
        std::string data = sock.buffer;

        return data;
    }

    glm::vec3 positionToUR5Frame(glm::vec3 v)
    {
        return glm::vec3(-v.z, -v.x, v.y);
    }

    glm::quat orientationToUR5Frame(glm::quat quat_in)
    {
        glm::vec3 new_euler = glm::vec3(glm::pitch(quat_in), -glm::roll(quat_in), -glm::yaw(quat_in));
        glm::quat new_quat = glm::quat(new_euler);

        return new_quat;
    }

    glm::vec3 positionToCameraFrame(glm::vec3 prev_p, glm::vec3 input_vel, glm::mat3 r_cam)
    {
        glm::vec3 new_pos;
        new_pos = prev_p + r_cam*input_vel;
        return new_pos;
    }

    glm::quat orientationToCameraFrame(glm::quat q)
    {

    }

    glm::mat4 translation_matrix(glm::vec3 coords)
    {
        glm::mat4 mat = glm::mat4();
        mat[0][3] = coords.x;
        mat[1][3] = coords.y;
        mat[2][3] = coords.z;

        return mat;
    }

    glm::vec3 translation_from_matrix(glm::mat4 mat)
    {
        return glm::vec3(mat[0][3], mat[1][3], mat[2][3]);
    }

    void App::handleControllerInput(std::string data)
    {
        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        json j = json::parse(data);
        json out_msg;

        glm::vec3 pos_vec;
        glm::quat quat_vec;

        auto contr = j.begin(); // We only handle the first controller
        for (json::iterator button = contr->begin(); button != contr->end(); button++)
        {
            if (button->is_object())
            {
                ContrCommands command(translateInputToCommand(button.key()));

                switch (command)
                {
                    case ContrCommands::POSE:
                    {
                        auto pos = (*button)["position"];
                        pos_vec = glm::vec3(pos["x"], pos["y"], pos["z"]);

                        auto quat = (*button)["orientation"];
                        quat_vec = glm::quat(quat["w"], quat["x"], quat["y"], quat["z"]);

                        if (!input.initialized)
                        {
                            input.init_pos = pos_vec;
                            input.prev_input_pos = pos_vec;
                            input.init_orient = quat_vec;

                            input.inv_init_quat = glm::inverse(quat_vec);

                            input.initialized = true;
                        }
                    }   break;

                    case ContrCommands::GRAB:
                    {
                        input.grabbing = (*button)["boolean"];
                    }   break;

                    case ContrCommands::RESET:
                    {
                        input.reset = (*button)["boolean"];
                    }   break;

                    case ContrCommands::CLUTCH:
                    {
                        input.clutching = (*button)["boolean"];

                        if (input.clutching.confirm_flip())
                        {
                            out_msg["clutching"] = input.clutching.is_on();
                        }
                    }   break;

                    case ContrCommands::OFFSET:
                    {
                        input.manual_adj = (*button)["boolean"];
                        input.manual_offset.x = (*button)["2d"]["x"];
                        input.manual_offset.y = (*button)["2d"]["y"];

                        if (!input.clutching.is_on() && input.manual_adj.confirm_flip_on())
                        {
                            if (input.manual_offset.x >= 0.5) {
                                out_msg["primary_next"] = true;
                            }
                            else if (input.manual_offset.x <= -0.5) {
                                out_msg["primary_prev"] = true;
                            }
                            else if (input.manual_offset.y >= 0.5) {
                                out_msg["pip_prev"] = true;
                            }
                            else if (input.manual_offset.y <= -0.5) {
                                out_msg["pip_next"] = true;
                            }
                            else {
                                out_msg["pip_toggle"] = true;
                            }
                        }
                    }   break;
                
                    default:
                    {
                        out_msg[button.key()] = button.value();
                    }   break;
                }
            }
        }


        if (input.clutching.is_flipping()) {
            if (input.clutching.is_on()) { // When just turned on
                input.clutch_offset = pos_vec - input.init_pos;
                // TODO: Add orientation handling
            }
            else {
                input.init_pos = pos_vec - input.clutch_offset;
            }
        }

        if (!input.clutching.is_on() && input.reset.confirm_flip_on())
        {
            input.init_pos = pos_vec;
            input.init_orient = quat_vec;

            input.inv_init_quat = glm::inverse(quat_vec);
        }

        if (!input.clutching.is_on() && !input.reset.is_on()) {
            glm::vec3 prev_pos = input.prev_input_pos - input.init_pos;
            glm::vec3 input_vel = pos_vec - input.prev_input_pos;
            // tf::TransformListener listener;
            // tf::StampedTransform base_to_cam;

            bool transform_found(false);
            geometry_msgs::TransformStamped base_to_cam;
            while (!transform_found)
            {
                try{
                    base_to_cam = tf_buffer.lookupTransform("base", "right_hand", ros::Time(0));
                    transform_found = true;
                }
                catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what());
                }
            }

            geometry_msgs::Quaternion cam_quat(base_to_cam.transform.rotation);
            glm::quat glm_quat(cam_quat.w, cam_quat.x, cam_quat.y, cam_quat.z);
            glm::mat3 rot_mat(glm::mat3_cast(glm_quat));
            std::cout << glm::to_string(rot_mat) << std::endl;

            input.position = positionToCameraFrame(prev_pos, input_vel, rot_mat);
            input.position = positionToUR5Frame(input.position);

            input.orientation = glm::quat(1.0, 0.0, 0.0, 0.0);


            // // input.manual_offset = positionToRobotFrame(input.manual_offset);
            // input.orientation = orientationToRobotFrame(input.orientation);
        }

        input.prev_input_pos = pos_vec;

        // printText(input.to_str());

        publishRobotData();

        if (!out_msg.is_null())
        {
            std::string output = out_msg.dump(3);
            send(out_socket.socket, output.c_str(), output.size(), 0);
        }
    }

    void App::publishRobotData()
    {
        EEPoseGoals goal;
        Pose pose;
        pose.position.x = input.position.x;
        pose.position.y = input.position.y;
        pose.position.z = input.position.z;

        pose.orientation.x = input.orientation.x;
        pose.orientation.y = input.orientation.y;
        pose.orientation.z = input.orientation.z;
        pose.orientation.w = input.orientation.w;

        Pose pose_cam;
        // pose_cam.position.x = input.position.x;
        // pose_cam.position.y = input.position.y;
        // pose_cam.position.z = input.position.z;

        pose_cam.orientation.x = 0.0;
        pose_cam.orientation.y = 0.0;
        pose_cam.orientation.z = 0.0;
        pose_cam.orientation.w = 1.0;


        Pose pose_head;

        pose_head.orientation.x = 0.0;
        pose_head.orientation.y = 0.0;
        pose_head.orientation.z = 0.0;
        pose_head.orientation.w = 1.0;


        goal.header.stamp = ros::Time::now();
        goal.ee_poses.push_back(pose);
        goal.ee_poses.push_back(pose_cam);
        goal.ee_poses.push_back(pose_head);

        ee_pub.publish(goal);


        Bool grabbing;
        grabbing.data = input.grabbing.is_on();

        grasper_pub.publish(grabbing);        
    }


    int App::run()
    {
        if (!init())
        {
            return 1;
        }
        
        pollfd poll_fds;
        poll_fds.fd = in_socket.socket;
        poll_fds.events = POLLIN; // Wait until there's data to read

        while (ros::ok())
        {
            if (poll(&poll_fds, 1, LOOP_RATE) > 0)
            {
                std::string input_data = getSocketData(in_socket);
                handleControllerInput(input_data);
            }
        }

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