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
        glm::vec3 manual_adjust;
        glm::mat3 cam_rot_mat;
        glm::vec3 cam_init_raw_pos, camera_offset;
        Switch grabbing, reset, clutching, toggle, reset_cam;
        float cur_outer_cone, cur_distance;
        bool initialized;
        bool cam_offset_init, camera_control;

        const float kStartingOuterCone = M_PI_4;
        const float kStartingDistance = 0.75;

        Input() : initialized(false), cam_offset_init(false), camera_control(false), 
                out_orient(1.0, 0.0, 0.0, 0.0), cur_ee_pos(0.0), cur_ee_orient(1.0, 0.0, 0.0, 0.0), 
                init_raw_orient(1.0, 0.0, 0.0, 0.0), out_pos(0.0), manual_adjust(0.0), cam_rot_mat(1.0), 
                camera_offset(0.0), cur_outer_cone(kStartingOuterCone), cur_distance(kStartingDistance),
                grabbing(Switch(true, Switch::Type::HOLD)), reset(Switch(false, Switch::Type::HOLD)),
                clutching(Switch(true, Switch::Type::SINGLE)), toggle(Switch(false, Switch::Type::HOLD)),
                reset_cam(Switch(false, Switch::Type::HOLD)) {}

        std::string to_str()
        {
            std::string content;
            content  = "Position: " + glm::to_string(out_pos) + "\n";
            content += "Orientation: " + glm::to_string(out_orient) + "\n";
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
        Socket controller_socket; // Controller commands such as vibration
        bool shutting_down;

        // ROS
        ros::Publisher ee_pub;
        ros::Publisher grasper_pub;
        ros::Publisher clutching_pub;
        ros::Publisher outer_cone_pub;
        ros::Publisher inner_cone_pub;
        ros::Publisher distance_pub;
        ros::Publisher toggle_pub;
        ros::Publisher controller_raw_pub;
        ros::Publisher controller_raw_string_pub;
        ros::Publisher keyboard_raw_pub;

        ros::Subscriber rot_mat_sub;
        ros::Subscriber keyboard_input_sub;
        ros::Subscriber cam_sub;
        ros::Subscriber collision_sub;

        ros::AsyncSpinner spinner;

        void camRotationMatrixCallback(std_msgs::Float32MultiArrayConstPtr msg);
        void controlFrameMatrixCallback(std_msgs::Float32MultiArrayConstPtr msg);
        void keyboardInputCallback(geometry_msgs::TwistStampedConstPtr msg);
        void evaluateVisibility(const sensor_msgs::ImageConstPtr image);
        void collisionsCallback(const std_msgs::String msg);

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
    glm::quat rotateQuaternionByMatrix(glm::quat q, glm::mat3 R);
    inline glm::vec3 updatePosition(const glm::vec3 &prev_p, const glm::vec3 &input_vel, const glm::mat3 &r_cam);
    glm::vec3 rotatePositionByQuaternion(glm::vec3 pos, glm::quat q, glm::quat q_inverse);
    inline glm::vec3 positionToCameraFrame(const glm::vec3 &prev_p, const glm::vec3 &input_vel, const glm::mat3 &r_cam);
    glm::vec3 positionToUR5Frame(glm::vec3 v);
    glm::quat orientationToUR5Frame(glm::quat quat_in);
    glm::mat4 translation_matrix(glm::vec3 coords);
    glm::vec3 translation_from_matrix(glm::mat4 mat);

} // namespace vive_input

#endif // __PUBLISH_INPUT_HPP__