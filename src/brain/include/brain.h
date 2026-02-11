#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vision_interface/msg/detections.hpp>
#include <vision_interface/msg/line_segments.hpp>
#include <vision_interface/msg/cal_param.hpp>
#include <vision_interface/msg/segmentation_result.hpp>
#include <game_controller_interface/msg/game_control_data.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "booster_interface/msg/odometer.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/raw_bytes_msg.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"

#include "RoboCupGameControlData.h"
#include "team_communication_msg.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdexcept>
#include "brain/msg/kick.hpp"

#include "brain_config.h"
#include "brain_data.h"
#include "brain_log.h"
#include "brain_tree.h"
#include "brain_communication.h"
#include "locator.h"
#include "robot_client.h"

using namespace std;

/**
 * Brain 核心类，因为要传递智能指针给 BrainTree，所以需要继承自 enable_shared_from_this
 * 数据封闭到各个子对象中，不要直接存在在 Brain 类
 * 如果是静态的配置值，放到 config
 * 运行时的动态数据，放到 data
 * TODO:
 * BehaviorTree 的 blackboard 里也存了一些数据，看看是不是有些重复存储了的，可以考虑去掉,
 * 目前的设计里，brain 指针传到了 BehaviorTree 的结点里了，在那里直接访问 brain->config 和 brain->data
 */
class Brain : public rclcpp::Node
{
public:
    // BrainConfig 对象，主要包含运行时需要的配置值（静态）
    std::shared_ptr<BrainConfig> config;
    // BrainLog 对象，封装 rerun log 相关的操作
    std::shared_ptr<BrainLog> log;
    // BrainData 对象，Brain 所有运行时的值都放在这里
    std::shared_ptr<BrainData> data;
    // RobotClient 对象，包含所有对机器人的操作
    std::shared_ptr<RobotClient> client;
    // locator 对象
    std::shared_ptr<Locator> locator;
    // BrainTree 对象，里面包含 BehaviorTree 相关的操作
    std::shared_ptr<BrainTree> tree;
    // Communication 对象，里面包含通信相关的操作，主要是双机通信和裁判机通信
    std::shared_ptr<BrainCommunication> communication;

    // 构造函数，接受 nodeName 创建 ros2 结点
    Brain();

    ~Brain();

    // 初始化操作，只需要在 main 函数中调用一次，初始化过程如不符合预期，可以抛异常直接中止程序
    void init();

    // 在 ros2 循环中调用
    void tick();

    // 处理特殊状态, 如发球状态, 任意球发球状态等
    void handleSpecialStates();

    void registerLocatorNodes(BT::BehaviorTreeFactory &factory)
    {
        RegisterLocatorNodes(factory, this);
    }

    /**
     * @brief 计算当前球到对方两个球门柱的向量角度, 球场坐标系
     *
     * @param  margin double, 计算角度时, 返回值比实际的球门的位置向内移动这个距离, 因为这个角度球会被门柱挡住. 此值越大, 射门准确性越高, 但调整角度花的时间也越长.
     *
     * @return vector<double> 返回值中 vec[0] 是左门柱, vec[1] 是右门柱. 注意左右都是以对方的方向为前方.
     */
    vector<double> getGoalPostAngles(const double margin = 0.3);

    double calcKickDir(double goalPostMargin = 0.3);

    // type: kick: 趟球, shoot: rl 踢球
    bool isAngleGood(double goalPostMargin = 0.3, string type = "kick");

   
    bool isBallOut(double locCompareDist = 2.0, double lineCompareDist = 0.3);

    
    void updateBallOut();

    
    double distToBorder();

    bool isDefensing();

    bool isBoundingBoxInCenter(BoundingBox boundingBox, double xRatio = 0.8, double yRatio = 0.8);

    void calibrateOdom(double x, double y, double theta);

    double msecsSince(rclcpp::Time time);

    bool isFreekickStartPlacing();

    rclcpp::Time timePointFromHeader(std_msgs::msg::Header header);

    void playSound(string soundName, double blockMsecs = 1000, bool allowRepeat = false);

    void speak(string text, bool allowRepeat = false);

    bool isPrimaryStriker();

    void updateCostToKick();



    // ------------------------------------------------------ SUB CALLBACKS ------------------------------------------------------

    void joystickCallback(const booster_interface::msg::RemoteControllerState &msg);

    void gameControlCallback(const game_controller_interface::msg::GameControlData &msg);

    void detectionsCallback(const vision_interface::msg::Detections &msg);

    void fieldLineCallback(const vision_interface::msg::LineSegments &msg);

    void imageCallback(const sensor_msgs::msg::Image &msg);

    void depthImageCallback(const sensor_msgs::msg::Image &msg);

    void odometerCallback(const booster_interface::msg::Odometer &msg);

    void lowStateCallback(const booster_interface::msg::LowState &msg);

    // void updateHeadPosBuffer(double pitch, double yaw);

    // bool isHeadStable(double msecSpan = 200);

    void headPoseCallback(const geometry_msgs::msg::Pose &msg);

    void recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg);

    void updateRelativePos(GameObject &obj);

    void updateFieldPos(GameObject &obj);

    /**
     * @brief 计算碰撞距离
     * 
     * @param angle double, 目标角度
     * 
     * @return double, 碰撞距离
     */
    double distToObstacle(double angle);

    vector<double> findSafeDirections(double startAngle, double safeDist, double step=deg2rad(10));

    double calcAvoidDir(double startAngle, double safeDist);

    Point2D findDribleDir(vector<Point2D> &detectedOpponents);


private:
    void loadConfig();

    void updateBallMemory();

    void updateRobotMemory();

    void updateObstacleMemory();

    void updateKickoffMemory();

    void updateMemory();

    void handleCooperation();

    // ------------------------------------------------------ 视觉处理 ------------------------------------------------------

    int markCntOnFieldLine(const string MarkType, const FieldLine line, const double margin = 0.2);

    int goalpostCntOnFieldLine(const FieldLine line, const double margin = 0.2);

    bool isBallOnFieldLine(const FieldLine line, const double margin = 0.3); 

    void identifyFieldLine(FieldLine &line);

    void identifyMarking(GameObject& marking);

    void identifyGoalpost(GameObject& goalpost);

    void updateLinePosToField(FieldLine &line);

    vector<FieldLine> processFieldLines(vector<FieldLine> &fieldLines);

    vector<GameObject> getGameObjects(const vision_interface::msg::Detections &msg);
    void detectProcessBalls(const vector<GameObject> &ballObjs);

    void detectProcessMarkings(const vector<GameObject> &markingObjs);

    void detectProcessRobots(const vector<GameObject> &robotObjs);

    void detectProcessGoalposts(const vector<GameObject> &goalpostObjs);

    void detectProcessVisionBox(const vision_interface::msg::Detections &msg);

    void logVisionBox(const rclcpp::Time &timePoint);

    void logDetection(const vector<GameObject> &gameObjects, bool logBoundingBox = true);

    void logMemRobots();

    void logObstacles();
    
    void logDepth(int grid_x_count, int grid_y_count, vector<vector<int>> &grid, vector<rerun::Vec3D> &points);

    void logDebugInfo();

    void updateLogFile();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscription;
    rclcpp::Subscription<game_controller_interface::msg::GameControlData>::SharedPtr gameControlSubscription;
    rclcpp::Subscription<vision_interface::msg::Detections>::SharedPtr detectionsSubscription;
    rclcpp::Subscription<vision_interface::msg::LineSegments>::SharedPtr subFieldLine;
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr odometerSubscription;
    rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr lowStateSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr headPoseSubscription;
    rclcpp::Subscription<booster_interface::msg::RawBytesMsg>::SharedPtr recoveryStateSubscription;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubSoundPlay;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubSpeak;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------ 调试 log 相关 ------------------------------------------------------
    void logObstacleDistance();
    void logLags();
    void statusReport();
    void logStatusToConsole();
    string getComLogString(); 
    void playSoundForFun();
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
