// BehaviorTree.CPP Plugin Registration
// This file registers all custom nodes with the BehaviorTree factory
// flex_bt_server will load this plugin dynamically via .so

#include "behaviortree_cpp/bt_factory.h"
#include "brain.h"

// Global Brain instance for plugin nodes
// NOTE: This is a workaround - ideally Brain would be passed from flex_bt_server
static std::shared_ptr<Brain> g_brain_instance;

// Initialize Brain instance if not already created
static void ensure_brain_initialized() {
    if (!g_brain_instance) {
        g_brain_instance = std::make_shared<Brain>();
        // NOTE: Brain needs proper initialization with ROS node
        // This is a minimal initialization - may need to be enhanced
    }
}

// Plugin export macro with custom registration
BT_REGISTER_NODES(factory)
{
    // Initialize Brain instance
    ensure_brain_initialized();
    Brain* brain = g_brain_instance.get();

    // Register nodes using lambda builders (same pattern as BrainTree::init)
    #define REGISTER_BUILDER(Name) \
        factory.registerBuilder<Name>(#Name, \
            [brain](const std::string& name, const BT::NodeConfig& config) { \
                return std::make_unique<Name>(name, config, brain); \
            });

    // ==================== Decision & Strategy Nodes ====================
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(GoalieDecide)

    // ==================== Ball Tracking Nodes ====================
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)

    // ==================== Movement & Navigation Nodes ====================
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(Chase)
    // Positioning - NOT IMPLEMENTED in brain package, skip
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoToReadyPosition)
    REGISTER_BUILDER(GoToGoalBlockingPosition)
    REGISTER_BUILDER(Assist)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(MoveHead)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(GoToFreekickPosition)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(SimpleChase)

    // ==================== Utility Nodes ====================
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)
    REGISTER_BUILDER(PlaySound)
    REGISTER_BUILDER(Speak)

    // ==================== Localization Nodes ====================
    // NOTE: These were commented out in brain/src/brain_tree.cpp
    // BUT they are used in game.xml, so enabling them here
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(SelfLocateEnterField)
    REGISTER_BUILDER(SelfLocate1M)
    REGISTER_BUILDER(SelfLocate2T)
    REGISTER_BUILDER(SelfLocatePT)
    REGISTER_BUILDER(SelfLocateLT)
    REGISTER_BUILDER(SelfLocate2X)
    REGISTER_BUILDER(SelfLocateBorder)

    #undef REGISTER_BUILDER
}

