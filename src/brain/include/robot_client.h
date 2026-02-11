#pragma once

#include <iostream>
#include <string>
#include <rerun.hpp>

#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "booster_msgs/msg/rpc_req_msg.hpp"
#include "booster_internal/robot/b1/b1_loco_internal_api.hpp"

using namespace std;

class Brain; // 类相互依赖，向前声明


/**
 * RobotClient 类，调用 RobotSDK 操控机器人的操作都放在这里
 * 因为目前的代码里依赖 brain 里相关的一些东西，现在设计成跟 brain 相互依赖
 */
class RobotClient
{
public:
    RobotClient(Brain* argBrain) : brain(argBrain) {}

    void init();

    /**
     * @brief 
     *
     * @param pitch
     * @param yaw
     *
     * @return int , 0 表示执行成功
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief 
     * 
     * @param x double, 
     * @param y double, 
     * @param theta double, 
     * @param applyMinX, applyMinY, applyMinTheta bool 
     * 
     * @return int , 0 表示执行成功
     * 
    */
    int setVelocity(double x, double y, double theta, bool applyMinX=true, bool applyMinY=true, bool applyMinTheta=true);

    int crabWalk(double angle, double speed);

    /**
     * @brief 
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double,
     * @param avoidObstacle bool, 
     * 
     * @return int 运控命令返回值, 0 代表成功
     */
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief   
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double,
     * @param avoidObstacle bool, 
     * 
     * @return int 运控命令返回值, 0 代表成功
     */

    int moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);
    /**
     * @brief 
     * 
     * @param tx, ty, ttheta double, 
     * @param longRangeThreshold double, 
     * @param turnThreshold double, 
     * @param vxLimit, vyLimit, vthetaLimit double, 
     * @param xTolerance, yTolerance, thetaTolerance double, 
     * @param avoidObstacle bool, 
     * 
     * @return int 运控命令返回值, 0 代表成功
     */
    int moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief 挥手
     */
    int waveHand(bool doWaveHand);

    /**
     * @brief 起身
     */
    int standUp();

    /**
     * @brief 恢复行走模式
     */
    int walkMode();

    /**
     * @brief 切换到robocup步态
     */
    int robocupWalk();

    /**
     * @brief 进阻尼
     */
    int enterDamping();

    double msecsToCollide(double vx, double vy, double vtheta, double maxTime=10000);

    bool isStandingStill(double timeBuffer = 1000);

    int highKick();

    int squatAction(booster_internal::robot::b1::SquatDirection direction);

    int goalieSquatDown(booster_internal::robot::b1::SquatDirection direction,
                        booster_internal::robot::b1::SquatSide side = booster_internal::robot::b1::SquatSide::kLeft);

    int enableVisualKickMode();

    int rlKickBall(float kick_speed, float kick_dir);

    int fancyKick(float kick_speed, float kick_dir);

    int32_t moveToTargetWithKick(float x, float y, float yaw);

private:
    int call(booster_interface::msg::BoosterApiReqMsg msg);
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
    double _vx, _vy, _vtheta;
    rclcpp::Time _lastCmdTime;
    rclcpp::Time _lastNonZeroCmdTime;
};
