#ifndef __PUBLISH_INPUT_HPP__
#define __PUBLISH_INPUT_HPP__

#include <string>
#include <ros/ros.h>
#include <netinet/in.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

#include "publish_vive_input/switch.hpp"


namespace vive_input
{

    struct Socket
    {
        uint static const DATA_SIZE = 2048;

        int socket;
        uint port = 8080;
        sockaddr_in address;
        socklen_t len;
        char buffer[DATA_SIZE];
    };

    struct Input
    {
        glm::vec3 init_pos, position;
        glm::vec3 clutch_offset, manual_offset;
        glm::quat init_orient, orientation, inv_init_quat;
        Switch grabbing, reset, clutching, manual_adj;
        bool initialized;

        Input()
        {
            grabbing = Switch(false, Switch::Type::HOLD);
            reset = Switch(false, Switch::Type::HOLD);
            clutching = Switch(false, Switch::Type::SINGLE);
            manual_adj = Switch(false, Switch::Type::HOLD);
        }

        std::string to_str()
        {
            std::string content;
            content  = "Position: " + glm::to_string(position) + "\n";
            content += "Orientation: " + glm::to_string(orientation) + "\n";
            content += "Manual Adj: " + manual_adj.to_str();
            content +=  "\t" + glm::to_string(manual_offset) + "\n";
            content += "Grab: " + grabbing.to_str() + "\n";
            content += "Reset: " + reset.to_str() + "\n";
            content += "Clutch: " + clutching.to_str() + "\n";

            return content;
        }
    };

    enum ContrCommands
    {
        NONE,
        POSE,
        GRAB,
        RESET,
        CLUTCH,
        OFFSET
    };


    class App
    {
    public:
        App() {}

        int run();

    private:
        Input input;
        Socket in_socket; // Raw Vive input data
        Socket out_socket; // Commands for interface

        // ROS
        ros::Publisher ee_pub;
        ros::Publisher gripper_pub;


        bool init();
        void handleControllerInput(std::string data);
        void publishRobotData();
    };

    bool initializeSocket(Socket &sock, bool incoming=true);
    std::string getSocketData(Socket &sock);
    glm::vec3 positionToRobotFrame(glm::vec3 v);
    glm::quat orientationToRobotFrame(glm::quat quat_in);
    glm::mat4 translation_matrix(glm::vec3 coords);
    glm::vec3 translation_from_matrix(glm::mat4 mat);

} // namespace vive_input

#endif // __PUBLISH_INPUT_HPP__