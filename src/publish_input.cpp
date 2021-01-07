#include <netinet/in.h>
#include <poll.h>

// ROS
#include <ros/ros.h>
#include <relaxed_ik/EEPoseGoals.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "nlohmann/json.hpp"
#include "publish_vive_input/utilities.hpp"
#include "publish_vive_input/publish_input.hpp"

using json = nlohmann::json;
using App = vive_input::App;

#define LOOP_RATE 60

namespace vive_input {

    ContrCommands translateButtonToCommand(std::string button)
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

    bool initializeSocket(Socket &sock)
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

        if (bind(sock.socket, (const sockaddr *)&sock.address, sizeof(sock.address)) < 0)
        { 
            printText("Socket binding failed."); 
            return false;
        }

        return true;
    }

    bool App::init()
    {
        // Init ROS
        ros::NodeHandle n;
        ee_pub = n.advertise<relaxed_ik::EEPoseGoals>("/relaxed_ik/ee_pose_goals", 1000);


        // Init sockets

        // Make sure that this matches the Vive params file and that it's not
        // the same as the out port
        in_socket.port = 8081;

        if (!initializeSocket(in_socket)) 
        {
            return false;
        }
        if (!initializeSocket(out_socket)) 
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

    void App::handleControllerInput(std::string data)
    {
        json j = json::parse(data);
        json out_msg;

        glm::vec3 pos_vec;
        glm::quat quat_vec;

        auto contr = j.begin(); // We only handle the first controller
        for (json::iterator button = contr->begin(); button != contr->end(); button++)
        {
            if (button->is_object())
            {
                ContrCommands command(translateButtonToCommand(button.key()));

                switch (command)
                {
                    case ContrCommands::POSE:
                    {
                        auto pos = (*button)["position"];
                        pos_vec = glm::vec3(pos["x"], pos["y"], pos["z"]);

                        auto quat = (*button)["orientation"];
                        quat_vec = glm::quat(quat["w"], quat["x"], quat["y"], quat["z"]);
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
                    }   break;

                    case ContrCommands::OFFSET:
                    {
                        input.manual_adj = (*button)["boolean"];
                        input.manual_offset.x = (*button)["2d"]["x"];
                        input.manual_offset.y = (*button)["2d"]["y"];
                    }   break;
                
                    default:
                    {
                        
                    }   break;
                }

                // else if(button.key() == "trigger")
                // {
                //     out_msg[button.key()] = button.value();
                    
                // }

            }
        }

        if (!input.initialized)
        {
            input.init_pos = pos_vec;
            input.prev_pos = pos_vec;
            input.init_orient = quat_vec;

            input.inv_init_quat = glm::inverse(quat_vec);

            input.initialized = true;
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

        // if (input.manual_adj.confirm_flip_on()) {
        //     if (input.manual_offset.x >= 0.5) {
        //         nextCamera(active_camera, pip_camera, cam_info.size());
        //     }
        //     else if (input.manual_offset.x <= -0.5) {
        //         previousCamera(active_camera, pip_camera, cam_info.size());
        //     }
        //     else if (input.manual_offset.z >= 0.5) {
        //         nextCamera(pip_camera, active_camera, cam_info.size(), false);
        //     }
        //     else if (input.manual_offset.z <= -0.5) {
        //         previousCamera(pip_camera, active_camera, cam_info.size(), false);
        //     }
        //     else {
        //         pip_enabled = !pip_enabled;
        //     }
        // }


        // // TODO: Consider how this should interact with clutching
        // if (!input.reset.is_on() && input.reset.is_flipping()) {
        //     input.init_pos = pos_vec;
        //     input.prev_pos = pos_vec;
        //     input.init_orient = quat_vec;

        //      input.inv_init_quat = glm::inverse(quat_vec);
        // }
        // // TODO: Allow reset to work while clutching

        // // TODO: **Add mode for using camera frame**
        // if (!input.clutching.is_on() && !input.reset.is_on()) {
        //     input.position = pos_vec - input.init_pos;
        //     input.position = glm::rotate(input.init_orient, input.position);
        //     glm::mat4 trans_mat = glm::toMat4(input.cam_orient) * translation_matrix(input.position);
        //     input.position = translation_from_matrix(trans_mat);
        //     input.orientation = input.inv_init_quat * quat_vec; // Displacement b/w quats

        //     input.position = positionToRobotFrame(input.position);
        //     // input.manual_offset = positionToRobotFrame(input.manual_offset);
        //     input.orientation = orientationToRobotFrame(input.orientation);
        // }
        // else {
        //     // Clutching mode handling
        //     // NOTE: The behavior of buttons changes while in this mode
        // }
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