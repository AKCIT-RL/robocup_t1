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

    /**
     * @brief Navega o robô até um ponto (x,y) no campo com desvio de obstáculos.
     *        Interface simplificada que reutiliza moveToPoseOnField3 internamente.
     *        O theta final é calculado automaticamente como a direção para a bola.
     * 
     * @param tx coordenada x do alvo no sistema do campo
     * @param ty coordenada y do alvo no sistema do campo
     * @param vxLimit limite de velocidade em x (default: 0.6)
     * @param vyLimit limite de velocidade em y (default: 0.5)
     * @param distTolerance tolerância de distância para considerar chegada (default: 0.3)
     * @param avoidObstacle se deve desviar de obstáculos (default: true)
     * 
     * @return true se o robô já está no ponto alvo (dentro da tolerância)
     */
    bool navigateToPoint(double tx, double ty, 
                         double vxLimit = 0.6, double vyLimit = 0.5,
                         double distTolerance = 0.3, bool avoidObstacle = true);

private:
    int call(booster_interface::msg::BoosterApiReqMsg msg);
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
    double _vx, _vy, _vtheta;
    rclcpp::Time _lastCmdTime;
    rclcpp::Time _lastNonZeroCmdTime;
};
