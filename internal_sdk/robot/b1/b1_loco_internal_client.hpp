#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_INTERNAL_CLIENT_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_INTERNAL_CLIENT_HPP__

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>

#include "b1_loco_internal_api.hpp"

using namespace booster::robot;

namespace booster_internal {
namespace robot {
namespace b1 {

class B1LocoInternalClient {
public:
    B1LocoInternalClient() = default;
    ~B1LocoInternalClient() = default;

    void Init();

    void Init(const std::string &robot_name);
    /**
     * @brief Send API request to B1 robot
     * 
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(LocoInternalApiId api_id, const std::string &param);

    /**
     * @brief Move robot to target position
     * 
     * @param x target position in x direction, unit: m
     * @param y target position in y direction, unit: m
     * @param yaw target yaw angle, unit: rad
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t MoveToTarget(float x, float y, float yaw) {
        MoveToTargetParameter move(x, y, yaw);
        std::string param = move.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kMoveToTargetInternal, param);
    }

    /**
     * @brief Move robot to target position and kick
     * 
     * @param x target position in x direction, unit: m
     * @param y target position in y direction, unit: m
     * @param yaw target yaw angle, unit: rad
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t MoveToTargetWithKick(float x, float y, float yaw) {
        MoveToTargetParameter move(x, y, yaw, true);
        std::string param = move.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kMoveToTargetInternal, param);
    }

    /**
     * @brief Move robot to target position with centripetal
     * 
     * @param x target position in x direction, unit: m
     * @param y target position in y direction, unit: m
     * @param yaw target yaw angle, unit: rad
     * @param centripetal_radius centripetal radius, unit: m
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t MoveToTargetWithCentripetal(float x, float y, float yaw, float centripetal_radius) {
        MoveToTargetParameter move(x, y, yaw, false, true, centripetal_radius);
        std::string param = move.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kMoveToTargetInternal, param);
    }

    /**
     * @brief High kick
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t HighKick() {
        std::string param = "{}";
        return SendApiRequest(LocoInternalApiId::KHighKick, param);
    }

    /**
     * @brief Hand action
     * 
     * @param hand_action_index hand action index
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t HandAction(int hand_action_index) {
        HandActionParameter hand_action(hand_action_index);
        std::string param = hand_action.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kHandAction, param);
    }

    /**
     * @brief set hand action params by action index
     * 
     * @param hand_action_index hand action index
     * @param reset_default reset hand action with specified index to default params
     * @param hand_action_params hand action params
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t SetHandActionParams(int hand_action_index, bool reset_default, std::vector<double> & hand_action_params) {
        SetHandActionParamsParameter set_hand_action_params(hand_action_index, reset_default, hand_action_params);
        std::string param = set_hand_action_params.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kSetHandActionParams, param);
    }

    /**
     * @brief Stance action
     * 
     * @param Stance_action_index stance action index, -1 to stance idle state, which means can change to
     * PREPARE, STANCE and etc.
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t StanceAction(int stance_action_index) {
        StanceActionParameter stance_action(stance_action_index);
        std::string param = stance_action.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kStanceAction, param);
    }

    /**
     * @brief squat action
     * 
     * @param direction squat direction up or down
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t SquatAction(SquatDirection direction) {
        SquatParameter squat_action(direction);
        std::string param = squat_action.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kSquatAction, param);
    }

    /**
     * @brief Enable robocup walk mode
     * 
     * @return 0 if success, otherwise return error code
     * 
    */
    int32_t EnableRobocupWalkMode() {
        std::string param = "{}";
        return SendApiRequest(LocoInternalApiId::kEnableRobocupWalkMode, param);
    }

    /**
     * @brief Change control mode
     * 
     * @param mode control mode
     * 
     * @return 0 if success, otherwise return error code
     * 
    */
    int32_t ChangeControlMode(ControlMode mode) {
        ControlModeParameter control_mode(mode);
        std::string param = control_mode.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kChangeControlMode, param);
    }

    /**
     * @brief Switch to custom_traj and doing the stand_up and then switch to RL
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t StandUpAndSwitch2RL() {
        std::string param = "{}";
        return SendApiRequest(LocoInternalApiId::kStandUp, param);
    }

    /**
     * @brief Goalie squat down action
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t GoalieSquatDown(SquatDirection direction, SquatSide side = SquatSide::kLeft) {
        GoalieSquatDownParameter goalie_squat_down_param(direction, side);
        std::string param = goalie_squat_down_param.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kGoalieSquatDown, param);
    }

    /**
     * @brief Enable visual kick mode
     *
     * @return 0 if success, otherwise return error code
     *
     */
    int32_t EnableVisualKickMode() {
        std::string param = "{}";
        return SendApiRequest(LocoInternalApiId::kEnableVisualKickMode, param);
    }

    /**
     * @brief Enable RL VMP side kick mode
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t RLKickBall(float kick_speed, float kick_dir) {
        RLKickBallParameter kick_paramenter(kick_speed, kick_dir);
        std::string param = kick_paramenter.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kRLKickBall, param);
    }

    /**
     * @brief RL fancy kick ball
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t RLFancyKickBall(float kick_speed, float kick_dir) {
        RLFancyKickBallParameter kick_parameter(kick_speed, kick_dir);
        std::string param = kick_parameter.ToJson().dump();
        return SendApiRequest(LocoInternalApiId::kRLFancyKickBall, param);
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__