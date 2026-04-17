#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_INTERNAL_API_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_INTERNAL_API_HPP__

#include <string>
#include <iostream>
#include <booster/third_party/nlohmann_json/json.hpp>
#include <booster_internal/robot/b1/b1_internal_api_const.hpp>

namespace booster_internal {
namespace robot {
namespace b1 {

/* service name */
const std::string LOCO_SERVICE_NAME = "loco_internal";

/*API version*/
const std::string LOCO_API_VERSION = "1.0.0.1";

/*API ID */
enum class LocoInternalApiId {
    kMoveToTargetInternal = 100000, 
    KHighKick = 100001,
    kHandAction = 100002,
    kSquatAction = 100003,
    kStanceAction = 100004,
    kChangeControlMode = 100005,
    kSetHandActionParams = 100006,
    kStandUp = 100007, //ficar em pé
    kEnableRobocupWalkMode = 100008,
    kGoalieSquatDown = 100009, //abaixa e expande uma perna pro lado para defender
    kEnableVisualKickMode = 100010, //nao ocorre nada 
    kRLKickBall = 100011, //chute pé de lado e mais fraco
    kRLFancyKickBall = 100012, //chute que fica com o pe mais alto e é mais forte
};

class MoveToTargetParameter {
public:
    MoveToTargetParameter() = default;
    MoveToTargetParameter(float x, float y, float yaw) :
        x_(x),
        y_(y),
        yaw_(yaw) {
    }

    MoveToTargetParameter(float x, float y, float yaw, bool is_kick_with_target) :
        x_(x),
        y_(y),
        yaw_(yaw),
        is_kick_with_target_(is_kick_with_target) {
    }
    MoveToTargetParameter(float x, float y, float yaw, bool is_kick_with_target, bool is_centripetal, float centripetal_radius) :
        x_(x),
        y_(y),
        yaw_(yaw),
        is_kick_with_target_(is_kick_with_target),
        is_centripetal_(is_centripetal),
        centripetal_radius_(centripetal_radius) {
    }

public:
    void FromJson(nlohmann::json &json) {
        x_ = json["x"];
        y_ = json["y"];
        yaw_ = json["yaw"];
        if (json.find("is_kick_with_target") != json.end()) {
            is_kick_with_target_ = json["is_kick_with_target"];
        }
        if (json.find("is_centripetal") != json.end()) {
            is_centripetal_ = json["is_centripetal"];
        }
        if (json.find("centripetal_radius") != json.end()) {
            centripetal_radius_ = json["centripetal_radius"];
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["x"] = x_;
        json["y"] = y_;
        json["yaw"] = yaw_;
        json["is_kick_with_target"] = is_kick_with_target_;
        json["is_centripetal"] = is_centripetal_;
        json["centripetal_radius"] = centripetal_radius_;
        return json;
    }

public:
    float x_;
    float y_;
    float yaw_;
    bool is_kick_with_target_ = false;
    bool is_centripetal_ = false;
    float centripetal_radius_;
};

class HandActionParameter {
public:
    HandActionParameter() = default;
    HandActionParameter(int hand_action_index) :
        hand_action_index_(hand_action_index) {
    }

public:
    void FromJson(nlohmann::json &json) {
        hand_action_index_ = json["hand_action_index"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["hand_action_index"] = hand_action_index_;
        return json;
    }

public:
    int hand_action_index_;
};

class SetHandActionParamsParameter {
public:
    SetHandActionParamsParameter() = default;
    SetHandActionParamsParameter(int hand_action_index, bool reset_default, std::vector<double> &hand_action_params) :
        hand_action_index_(hand_action_index),
        reset_default_(reset_default),
        hand_action_params_(hand_action_params) {
    }

public:
    void FromJson(nlohmann::json &json) {
        hand_action_index_ = json["hand_action_index"];
        reset_default_ = json["reset_default"];

        try {
            hand_action_params_ = json["hand_action_params"].get<std::vector<double>>();
        } catch (const nlohmann::json::exception &e) {
            std::cerr << "cannot parse hand_action_params in SetHandActionParamsParameter: " << e.what() << std::endl;
        }
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["hand_action_index"] = hand_action_index_;
        json["reset_default"] = reset_default_;
        json["hand_action_params"] = hand_action_params_;
        return json;
    }

public:
    int hand_action_index_;
    bool reset_default_;
    std::vector<double> hand_action_params_;
};

enum STANCE_ACTION_INDEX {
    IDLE = -1,
};

class StanceActionParameter {
public:
    StanceActionParameter() = default;
    StanceActionParameter(int stance_action_index) :
        stance_action_index_(stance_action_index) {
    }

public:
    void FromJson(nlohmann::json &json) {
        stance_action_index_ = json["stance_action_index"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["stance_action_index"] = stance_action_index_;
        return json;
    }

public:
    int stance_action_index_;
};

class SquatParameter {
public:
    SquatParameter() = default;
    SquatParameter(booster_internal::robot::b1::SquatDirection squat_direction) :
        squat_direction_(squat_direction) {
    }

public:
    void FromJson(nlohmann::json &json) {
        squat_direction_ = static_cast<booster_internal::robot::b1::SquatDirection>(json["squat_direction"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["squat_direction"] = static_cast<int>(squat_direction_);
        return json;
    }

public:
    SquatDirection squat_direction_;
};

class ControlModeParameter {
public:
    ControlModeParameter() = default;
    ControlModeParameter(booster_internal::robot::b1::ControlMode control_mode) :
        control_mode_(control_mode) {
    }

public:
    void FromJson(nlohmann::json &json) {
        control_mode_ = static_cast<booster_internal::robot::b1::ControlMode>(json["control_mode"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["control_mode"] = static_cast<int>(control_mode_);
        return json;
    }

public:
    ControlMode control_mode_;
};

class GoalieSquatDownParameter {
public:
    GoalieSquatDownParameter() = default;
    GoalieSquatDownParameter(booster_internal::robot::b1::SquatDirection squat_direction, booster_internal::robot::b1::SquatSide squat_side) :
        squat_direction_(squat_direction),
        squat_side_(squat_side) {
    }
    SquatDirection squat_direction_;
    SquatSide squat_side_;

    void FromJson(nlohmann::json &json) {
        squat_direction_ = static_cast<booster_internal::robot::b1::SquatDirection>(json["squat_direction"]);
        squat_side_ = static_cast<booster_internal::robot::b1::SquatSide>(json["squat_side"]);
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["squat_direction"] = static_cast<int>(squat_direction_);
        json["squat_side"] = static_cast<int>(squat_side_);
        return json;
    }
};

class RLKickBallParameter {
public:
    RLKickBallParameter() = default;
    RLKickBallParameter(float kick_speed, float kick_dir) :
        kick_speed_(kick_speed),
        kick_dir_(kick_dir) {
    }
    float kick_speed_;
    float kick_dir_;

    void FromJson(nlohmann::json &json) {
        kick_speed_ = json["kick_speed"];
        kick_dir_ = json["kick_dir"];
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["kick_speed"] = kick_speed_;
        json["kick_dir"] = kick_dir_;
        return json;
    }
};

class RLFancyKickBallParameter {
public:
    RLFancyKickBallParameter() = default;
    RLFancyKickBallParameter(float kick_speed, float kick_dir) :
        kick_speed_(kick_speed),
        kick_dir_(kick_dir) {
    }
    float kick_speed_;
    float kick_dir_;

    void FromJson(nlohmann::json &json) {
        kick_speed_ = json["kick_speed"];
        kick_dir_ = json["kick_dir"];
    }
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["kick_speed"] = kick_speed_;
        json["kick_dir"] = kick_dir_;
        return json;
    }
};

}
}
} // namespace booster_internal::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_INTERNAL_API_HPP__