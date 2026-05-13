#pragma once

#include <tuple>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>
#include <string>
#include <rclcpp/rclcpp.hpp>

// Forward declaration
class Brain;

using namespace std;
using namespace BT;

// ==================== Decision & Strategy Nodes ====================

class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir; 
    rclcpp::Time timeLastTick; 
};

class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// ==================== Ball Tracking Nodes ====================

class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    
    rclcpp::Time _timeLastCmd;   
    int _cmdIndex;                
    long _cmdIntervalMSec;        
    long _cmdRestartIntervalMSec; 
    Brain *brain;
};

class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    double _turnDir; 
    Brain *brain;
};

class CamFastScan : public StatefulActionNode
{
public:
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};

private:
    double _cmdSequence[7][2] = {
        {0.45, 1.1}, {0.45, 0.0}, {0.45, -1.1},
        {1.0, -1.1}, {1.0, 0.0}, {1.0, 1.1}, {0.45, 0.0},
    };    
    rclcpp::Time _timeLastCmd;    
    int _cmdIndex = 0;               
    Brain *brain;
};

class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain);

    static BT::PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

// ==================== Movement & Navigation Nodes ====================

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override {};

private:
    double _lastAngle; 
    double _angle;
    double _cumAngle; 
    double _msecLimit = 5000;  
    rclcpp::Time _timeStart;
    Brain *brain;
};

class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     
    double _dir = 1.0; 
};

class Positioning : public SyncActionNode
{
public:
    Positioning(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

class Adjust : public StatefulActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
};

class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
    string _state = "kick";
    int _msecKick = 1000;    
    double _speed; 
    double _minRange; 
    tuple<double, double, double> _calcSpeed();
};

class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
};

class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoToReadyPosition : public SyncActionNode
{
public:
    GoToReadyPosition(const std::string &name, const NodeConfig &config, Brain *_brain);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class Assist : public SyncActionNode
{
public:
    Assist(const std::string &name, const NodeConfig &config, Brain *_brain);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    Brain *brain;
};

class CheckAndStandUp : public SyncActionNode
{
public:
    CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    bool _isInFinalAdjust = false; 
};

class GoBackInField : public SyncActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

// ==================== Utility Nodes ====================

class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain);

    static PortsList providedPorts();
    NodeStatus tick() override;

private:
    Brain *brain;
};

class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class PlaySound : public SyncActionNode
{
public:
    PlaySound(const std::string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    Brain *brain;
};

class Speak : public SyncActionNode
{
public:
    Speak(const std::string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    Brain *brain;
};
