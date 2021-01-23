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
        glm::vec3 prev_raw_pos, prev_ee_pos, cur_ee_pos, out_pos;
        glm::quat prev_raw_orient, prev_ee_orient, cur_ee_orient, out_orient;
        Switch grabbing, reset, clutching, manual_adj;
        bool initialized;

        Input() : out_orient(1.0, 0.0, 0.0, 0.0)
        {
            grabbing = Switch(true, Switch::Type::SINGLE);
            reset = Switch(false, Switch::Type::HOLD);
            clutching = Switch(true, Switch::Type::SINGLE);
            manual_adj = Switch(false, Switch::Type::HOLD);
        }

        std::string to_str()
        {
            std::string content;
            content  = "Position: " + glm::to_string(out_pos) + "\n";
            content += "Orientation: " + glm::to_string(out_orient) + "\n";
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
        App() : spinner(ros::AsyncSpinner(0)) {}

        int run();

    private:
        Input input;
        Socket in_socket; // Raw Vive input data
        Socket out_socket; // Commands for interface

        // ROS
        ros::Publisher ee_pub;
        ros::Publisher grasper_pub;
        ros::Subscriber cam_sub;
        ros::AsyncSpinner spinner;

        static void evaluateVisibility(const sensor_msgs::ImageConstPtr image);

        bool init();
        void resetPose(glm::vec3 pos, glm::quat quat);
        void handleControllerInput(std::string data);
        void publishRobotData();
    };

    bool initializeSocket(Socket &sock, bool incoming=true);
    std::string getSocketData(Socket &sock);
    inline glm::quat quaternionDisplacement(const glm::quat &q1, const glm::quat &quat2);
    glm::quat rotateQuaternion(glm::quat q, glm::mat3 R);
    glm::vec3 rotatePositionByQuaternion(glm::vec3 pos, glm::quat q, glm::quat q_inverse);
    inline glm::vec3 positionToCameraFrame(const glm::vec3 &prev_p, const glm::vec3 &input_vel, const glm::mat3 &r_cam);
    glm::vec3 positionToUR5Frame(glm::vec3 v);
    glm::quat orientationToUR5Frame(glm::quat quat_in);
    glm::mat4 translation_matrix(glm::vec3 coords);
    glm::vec3 translation_from_matrix(glm::mat4 mat);

} // namespace vive_input

#endif // __PUBLISH_INPUT_HPP__