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
#include <tf/transform_listener.h>

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
        glm::vec3  prev_raw_pos, input_posi_vel;
        glm::quat  prev_raw_orient, input_rot_vel;
        glm::vec3 manual_adjust;
        // glm::mat3 cam_rot_mat;
        glm::mat3 control_mappings;
        glm::vec3 cam_init_raw_pos, camera_offset;
        Switch grabbing, reset, clutching, toggle, reset_cam;
        float cur_outer_cone, cur_distance;
        bool cam_offset_init, camera_control;
        bool initialized;
        bool manual_reset;

        const float kStartingOuterCone = M_PI_4;
        const float kStartingDistance = 0.75;

        Input() :  initialized(false), manual_reset(false),
                input_rot_vel(1.0, 0.0, 0.0, 0.0), prev_raw_pos(0.0), prev_raw_orient(1.0, 0.0, 0.0, 0.0), 
                 input_posi_vel(0.0), manual_adjust(0.0), control_mappings(1.0), 
                camera_offset(0.0), cur_outer_cone(kStartingOuterCone), cur_distance(kStartingDistance),
                grabbing(Switch(true, Switch::Type::HOLD)), reset(Switch(false, Switch::Type::HOLD)),
                clutching(Switch(false, Switch::Type::HOLD)), toggle(Switch(false, Switch::Type::HOLD)),
                reset_cam(Switch(false, Switch::Type::HOLD)) {}

        std::string to_str()
        {
            std::string content;
            content  = "Position: " + glm::to_string(input_posi_vel) + "\n";
            content += "Orientation: " + glm::to_string(input_rot_vel) + "\n";
            content += "Manual Adj: " + toggle.to_str();
            content +=  "\t" + glm::to_string(manual_adjust) + "\n";
            content += "Grab: " + grabbing.to_str() + "\n";
            content += "Reset: " + reset.to_str() + "\n";
            content += "Clutch: " + clutching.to_str() + "\n";
            content += "Camera Offset: " + glm::to_string(camera_offset) + "\n";

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
        tf::StampedTransform wrist_3_transform;

        // ROS
        ros::Publisher controller_pose_pub;
        ros::Publisher grasper_pub;
        ros::Publisher clutching_pub;
        ros::Publisher controller_raw_pub;
        ros::Publisher controller_raw_string_pub;

        ros::AsyncSpinner spinner;

        void triggerManualReset(std_msgs::Bool msg);

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
    inline glm::vec3 updatePosition(const glm::vec3 &prev_p, const glm::vec3 &input_vel, const glm::mat3 &r_cam);
    glm::vec3 rotatePositionByQuaternion(glm::vec3 pos, glm::quat q, glm::quat q_inverse);
    inline glm::vec3 positionToCameraFrame(const glm::vec3 &prev_p, const glm::vec3 &input_vel, const glm::mat3 &r_cam);
    glm::vec3 positionToUR5Frame(glm::vec3 v);
    glm::quat orientationToUR5Frame(glm::quat quat_in);
    glm::mat4 translation_matrix(glm::vec3 coords);
    glm::vec3 translation_from_matrix(glm::mat4 mat);

} // namespace vive_input

#endif // __PUBLISH_INPUT_HPP__