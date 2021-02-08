#ifndef __PUBLISH_INPUT_HPP__
#define __PUBLISH_INPUT_HPP__

#include <string>
#include <ros/ros.h>
#include <math.h>
#include <netinet/in.h>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/mat3x3.hpp>
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
        glm::quat init_raw_orient;
        glm::vec3 manual_offset;
        glm::mat3 cam_rot_mat;
        Switch grabbing, reset, clutching, manual_adj;
        float cur_outer_cone, cur_distance;
        bool initialized;

        const float kStartingOuterCone = M_PI_2;
        const float kStartingDistance = 1.0;

        Input() : initialized(false), out_orient(1.0, 0.0, 0.0, 0.0), cur_ee_pos(0.0, 0.0, 0.0),
                cur_ee_orient(1.0, 0.0, 0.0, 0.0), init_raw_orient(1.0, 0.0, 0.0, 0.0),
                out_pos(0.0, 0.0, 0.0), cam_rot_mat(1.0), cur_outer_cone(kStartingOuterCone), 
                cur_distance(kStartingDistance)
        {
            grabbing = Switch(true, Switch::Type::HOLD);
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
        App() : spinner(ros::AsyncSpinner(0)), shutting_down(false) {}

        int run();

    private:
        Input input;
        Socket in_socket; // Raw Vive input data
        Socket out_socket; // Commands for interface
        bool shutting_down;

        // ROS
        ros::Publisher ee_pub;
        ros::Publisher grasper_pub;
        ros::Publisher clutching_pub;
        ros::Publisher outer_cone_pub;
        ros::Publisher inner_cone_pub;
        ros::Publisher distance_pub;
        ros::Subscriber rot_mat_sub;
        ros::Subscriber cam_sub;
        ros::AsyncSpinner spinner;

        void camRotationMatrixCallback(std_msgs::Float64MultiArrayConstPtr msg);
        void evaluateVisibility(const sensor_msgs::ImageConstPtr image);

        bool init();
        void resetPose(glm::vec3 pos, glm::quat quat);
        void handleControllerInput(std::string data);
        void publishRobotData();
        void getKeyboardInput();
        void handleKeyboardInput(int command);
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