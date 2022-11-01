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
#include <ur5_optimization/EEPoseGoals.h>
#include <publish_vive_input/ButtonInfo.h>
#include <publish_vive_input/ControllerInfo.h>
#include <publish_vive_input/ControllersInput.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include "nlohmann/json.hpp"
#include "publish_vive_input/utilities.hpp"
#include "publish_vive_input/publish_input.hpp"

using json = nlohmann::json;
using EEPoseGoals = ur5_optimization::EEPoseGoals;
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
        else if (button == "menu") {
            return ContrCommands::GRAB;
        }
        else if (button == "gripper") {
            return ContrCommands::RESET;
        }
        else if (button == "trigger") {
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
        ros::NodeHandle private_n("~");

        grasper_pub = n.advertise<std_msgs::Bool>("/vive_input/grasping", 1);
        clutching_pub = n.advertise<std_msgs::Bool>("/vive_input/clutching", 1);

        controller_pose_pub = n.advertise<geometry_msgs::Pose>("/vive_input/controller_vel", 1);
        controller_raw_pub = n.advertise<ControllersInput>("/vive_input/raw_data", 1);
        controller_raw_string_pub = n.advertise<std_msgs::String>("/vive_input/raw_string", 1);
        user_start_pub = n.advertise<std_msgs::Bool>("user_start", 1);

        task_state = "pre";
        study_state_sub = n.subscribe("study_state", 1000, &App::studyStateCb, this);

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


    void App::studyStateCb(const std_msgs::String::ConstPtr& msg)
    {
        std::string s = msg->data.c_str();
        if (s.find_last_of('_') != std::string::npos) {
            task_state = s.substr(s.find_last_of('_')+1);
        }
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

    void App::triggerManualReset(std_msgs::Bool msg)
    {
        input.manual_reset = msg.data;
    }

    void App::resetPose(glm::vec3 new_pos, glm::quat new_orient)
    {
        input.prev_raw_pos = new_pos;
        input.prev_raw_orient = new_orient;
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

    void App::handleControllerInput(std::string data)
    {
        static tf2_ros::Buffer tf_buffer;
        static tf2_ros::TransformListener tf_listener(tf_buffer);

        json j(json::parse(data));
        json out_msg;

        glm::vec3 cur_raw_pos;
        glm::quat cur_raw_orient;
        bool right_contr_active(false);

        ControllersInput contr_input;
        ControllerInfo right_contr; right_contr.role = "right";

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
                                // bool raw_input(j[controller][button]["boolean"] and j[controller][button]["pressure"] < 1.0 );
                                input.clutching = raw_input;
                                if (raw_input == true && strcmp(task_state.c_str(), "ready") == 0) {
                                    std_msgs::Bool msg;
                                    msg.data = true;
                                    user_start_pub.publish( msg );
                                }

                                right_contr.button3.name = "clutch";
                                right_contr.button3.has_boolean = true;
                                right_contr.button3.boolean = raw_input;
                                right_contr.button3.has_pressure = true;
                                right_contr.button3.pressure = j[controller][button]["pressure"];

                            }   break;

                            default:
                            {
                                out_msg[button] = button_it.value();
                            }   break;
                        }
                    }

                }
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
        if (input.clutching.is_on() && !input.reset.is_on()) {

            input.input_posi_vel = cur_raw_pos - input.prev_raw_pos;

            glm::quat q_v1(glm::normalize(rotateQuaternionByMatrix(input.prev_raw_orient, glm::mat3_cast(cur_raw_orient))));
            input.input_rot_vel = glm::normalize(quaternionDisplacement(q_v1, cur_raw_orient));
            
            geometry_msgs::Pose pose;
            pose.position.x = input.input_posi_vel.x;
            pose.position.y = input.input_posi_vel.y;
            pose.position.z = input.input_posi_vel.z;
            pose.orientation.x = input.input_rot_vel.x;
            pose.orientation.y = input.input_rot_vel.y;
            pose.orientation.z = input.input_rot_vel.z;
            pose.orientation.w = input.input_rot_vel.w;
            controller_pose_pub.publish(pose);
        }

        input.prev_raw_pos = cur_raw_pos;
        input.prev_raw_orient = cur_raw_orient;

        std_msgs::Bool reset;
        reset.data = input.reset.is_on();

        std_msgs::Bool grabbing, clutching;
        grabbing.data = input.grabbing.is_on();
        clutching.data = input.clutching.is_on();

        grasper_pub.publish(grabbing);
        clutching_pub.publish(clutching);        


        if (!out_msg.is_null()) {
            std::string output(out_msg.dump(3));
            send(out_socket.socket, output.c_str(), output.size(), 0);
        }

        // Publish raw input as string and as ControllersInput message
        std_msgs::String raw_controller_input;
        raw_controller_input.data = j.dump(3);
        controller_raw_string_pub.publish(raw_controller_input);

        right_contr.active = true; // Already validated above

        contr_input.header.stamp = ros::Time::now();
        contr_input.controllers.push_back(right_contr);
        controller_raw_pub.publish(contr_input);
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