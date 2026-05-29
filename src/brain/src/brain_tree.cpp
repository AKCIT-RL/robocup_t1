#include "brain_tree.h"
#include "brain.h"
#include "locator.h"
#include "std_msgs/msg/string.hpp"
#include "utils/math.h"
#include "utils/misc.h"
#include "utils/print.h"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <ios>

/**
 * 这里使用宏定义来缩减 RegisterBuilder 的代码量
 * REGISTER_BUILDER(Test) 展开后的效果是
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *     [this](const string& name, const NodeConfig& config) { return
 * make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)                                                 \
  factory.registerBuilder<Name>(                                               \
      #Name, [this](const string &name, const NodeConfig &config) {            \
        return make_unique<Name>(name, config, brain);                         \
      });

void BrainTree::init() {
  BehaviorTreeFactory factory;

  // Action Nodes
  REGISTER_BUILDER(RobotFindBall)
  REGISTER_BUILDER(Chase)
  REGISTER_BUILDER(SimpleChase)
  REGISTER_BUILDER(Adjust)
  REGISTER_BUILDER(Kick)
  REGISTER_BUILDER(StandStill)
  REGISTER_BUILDER(CalcKickDir)
  REGISTER_BUILDER(StrikerDecide)
  REGISTER_BUILDER(CamTrackBall)
  REGISTER_BUILDER(CamFindBall)
  REGISTER_BUILDER(CamFastScan)
  REGISTER_BUILDER(CamScanField)
  // REGISTER_BUILDER(SelfLocate)
  // REGISTER_BUILDER(SelfLocateEnterField)
  // REGISTER_BUILDER(SelfLocate1M)
  // REGISTER_BUILDER(SelfLocateBorder)
  // REGISTER_BUILDER(SelfLocate2T)
  // REGISTER_BUILDER(SelfLocateLT)
  // REGISTER_BUILDER(SelfLocatePT)
  // REGISTER_BUILDER(SelfLocate2X)
  REGISTER_BUILDER(SetVelocity)
  REGISTER_BUILDER(StepOnSpot)
  REGISTER_BUILDER(GoToFreekickPosition)
  REGISTER_BUILDER(GoToReadyPosition)
  REGISTER_BUILDER(GoToGoalBlockingPosition)
  REGISTER_BUILDER(TurnOnSpot)
  REGISTER_BUILDER(MoveToPoseOnField)
  REGISTER_BUILDER(GoBackInField)
  REGISTER_BUILDER(GoalieDecide)
  REGISTER_BUILDER(WaveHand)
  REGISTER_BUILDER(MoveHead)
  REGISTER_BUILDER(CheckAndStandUp)
  REGISTER_BUILDER(Assist)

  // 注册 Locator 相关的节点
  brain->registerLocatorNodes(factory);

  // Action Nodes for debug
  REGISTER_BUILDER(CalibrateOdom)
  REGISTER_BUILDER(PrintMsg)
  REGISTER_BUILDER(PlaySound)
  REGISTER_BUILDER(Speak)

  factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
  tree = factory.createTree("MainTree");

  // 构造完成后，初始化 blackboard entry
  initEntry();

  // Initialize Groot2Publisher for real-time BT visualization
  if (brain->config->enableGrootMonitoring) {
      try {
          publisher = std::make_unique<BT::Groot2Publisher>(tree, brain->config->grootZmqServerPort);
          RCLCPP_INFO(brain->get_logger(), 
                     "Groot2Publisher enabled on ports (publisher: %d, server: %d)", 
                     brain->config->grootZmqPublisherPort,
                     brain->config->grootZmqServerPort);
      } catch (const std::exception& e) {
          RCLCPP_WARN(brain->get_logger(), 
                     "Failed to initialize Groot2Publisher: %s. Continuing without monitoring.",
                     e.what());
      }
  } else {
      RCLCPP_INFO(brain->get_logger(), "Groot2Publisher disabled via configuration");
  }
}

void BrainTree::initEntry() {
  setEntry<string>("player_role", brain->config->playerRole);
  setEntry<bool>("ball_location_known", false);
  setEntry<bool>("tm_ball_pos_reliable", false);
  setEntry<bool>("ball_out", false);
  setEntry<bool>("track_ball", true);
  setEntry<bool>("odom_calibrated", false);
  setEntry<string>("decision", "");
  setEntry<string>("defend_decision", "chase");
  setEntry<double>("ball_range", 0);

  setEntry<bool>("gamecontroller_isKickOff", true);
  setEntry<string>("gc_game_state", "");
  setEntry<string>("gc_game_sub_state_type", "NONE");
  setEntry<string>("gc_game_sub_state", "");
  setEntry<bool>("gc_is_kickoff_side", false);
  setEntry<bool>("gc_is_sub_state_kickoff_side", false);
  setEntry<bool>("gc_is_under_penalty", false);

  setEntry<bool>("need_check_behind", false);

  setEntry<bool>("is_lead", true);
  setEntry<string>("goalie_mode", "attack");

  setEntry<int>("test_choice", 0);
  setEntry<int>("control_state", 0);
  setEntry<bool>("assist_chase", false);
  setEntry<bool>("assist_kick", false);
  setEntry<bool>("go_manual", false);

  setEntry<bool>("we_just_scored", false);
  setEntry<bool>("wait_for_opponent_kickoff", false);

  // 自动视觉校准相关
  setEntry<string>("calibrate_state", "pitch");
  setEntry<double>("calibrate_pitch_center", 0.0);
  setEntry<double>("calibrate_pitch_step", 1.0);
  setEntry<double>("calibrate_yaw_center", 0.0);
  setEntry<double>("calibrate_yaw_step", 1.0);
  setEntry<double>("calibrate_z_center", 0.0);
  setEntry<double>("calibrate_z_step", 0.01);
}

void BrainTree::tick() { tree.tickOnce(); }

NodeStatus SetVelocity::tick() {
  double x, y, theta;
  vector<double> targetVec;
  getInput("x", x);
  getInput("y", y);
  getInput("theta", theta);

  auto res = brain->client->setVelocity(x, y, theta);
  return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick() {
  std::srand(std::time(0));
  double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

  auto res = brain->client->setVelocity(vx, 0, 0);
  return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick() {
  double pitch, yaw, ballX, ballY, deltaX, deltaY;
  const double pixToleranceX = brain->config->camPixX / 4.;
  const double pixToleranceY = brain->config->camPixY / 4.;
  const double xCenter = brain->config->camPixX / 2;
  const double yCenter = brain->config->camPixY / 2;

  auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/CamTrackBall", rerun::TextLog(msg));
  };
  auto logTrackingBox = [=](int color, string label) {
    brain->log->setTimeNow();
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    mins.push_back(
        rerun::Vec2D{xCenter - pixToleranceX, yCenter - pixToleranceY});
    sizes.push_back(rerun::Vec2D{pixToleranceX * 2, pixToleranceY * 2});
    brain->log->log("image/track_ball",
                    rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                        .with_labels({label})
                        .with_colors(color));
  };

  bool iSeeBall = brain->data->ballDetected;
  bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
  bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
  if (!(iKnowBallPos || tmBallPosReliable))
    return NodeStatus::SUCCESS;

  if (!iSeeBall) {
    if (iKnowBallPos) {
      pitch = brain->data->ball.pitchToRobot;
      yaw = brain->data->ball.yawToRobot;
    } else if (tmBallPosReliable) {
      pitch = brain->data->tmBall.pitchToRobot;
      yaw = brain->data->tmBall.yawToRobot;
    } else {
      log("reached impossible condition");
    }
    logTrackingBox(0x000000FF, "ball not detected");
  } else {
    ballX = mean(brain->data->ball.boundingBox.xmax,
                 brain->data->ball.boundingBox.xmin);
    ballY = mean(brain->data->ball.boundingBox.ymax,
                 brain->data->ball.boundingBox.ymin);
    deltaX = ballX - xCenter;
    deltaY = ballY - yCenter;

    if (std::fabs(deltaX) < pixToleranceX &&
        std::fabs(deltaY) < pixToleranceY) {
      auto label =
          format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f", ballX,
                 ballY, deltaX, deltaY);
      logTrackingBox(0x00FF00FF, label);
      return NodeStatus::SUCCESS;
    }

    double smoother = 1.5;
    double deltaYaw =
        deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
    double deltaPitch =
        deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

    pitch = brain->data->headPitch + deltaPitch;
    yaw = brain->data->headYaw - deltaYaw;
    auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f, "
                        "pitch: %.1f, yaw: %.1f",
                        ballX, ballY, deltaX, deltaY, pitch, yaw);
    logTrackingBox(0xFF0000FF, label);
  }

  brain->client->moveHead(pitch, yaw);
  return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config,
                         Brain *_brain)
    : SyncActionNode(name, config), brain(_brain) {
  double lowPitch = 1.0;
  double highPitch = 0.45;
  double leftYaw = 1.1;
  double rightYaw = -1.1;

  _cmdSequence[0][0] = lowPitch;
  _cmdSequence[0][1] = leftYaw;
  _cmdSequence[1][0] = lowPitch;
  _cmdSequence[1][1] = 0;
  _cmdSequence[2][0] = lowPitch;
  _cmdSequence[2][1] = rightYaw;
  _cmdSequence[3][0] = highPitch;
  _cmdSequence[3][1] = rightYaw;
  _cmdSequence[4][0] = highPitch;
  _cmdSequence[4][1] = 0;
  _cmdSequence[5][0] = highPitch;
  _cmdSequence[5][1] = leftYaw;

  _cmdIndex = 0;
  _cmdIntervalMSec = 800;
  _cmdRestartIntervalMSec = 50000;
  _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick() {
  if (brain->data->ballDetected) {
    return NodeStatus::SUCCESS;
  }

  auto curTime = brain->get_clock()->now();
  auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
  if (timeSinceLastCmd < _cmdIntervalMSec) {
    return NodeStatus::SUCCESS;
  } else if (timeSinceLastCmd > _cmdRestartIntervalMSec) {
    _cmdIndex = 0;
  } else {
    _cmdIndex =
        (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
  }

  brain->client->moveHead(_cmdSequence[_cmdIndex][0],
                          _cmdSequence[_cmdIndex][1]);
  _timeLastCmd = brain->get_clock()->now();
  return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick() {
  auto sec = brain->get_clock()->now().seconds();
  auto msec = static_cast<unsigned long long>(sec * 1000);
  double lowPitch, highPitch, leftYaw, rightYaw;
  getInput("low_pitch", lowPitch);
  getInput("high_pitch", highPitch);
  getInput("left_yaw", leftYaw);
  getInput("right_yaw", rightYaw);
  int msecCycle;
  getInput("msec_cycle", msecCycle);

  int cycleTime = msec % msecCycle;
  double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
  double yaw =
      cycleTime < (msecCycle / 2.0)
          ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw
          : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) +
                rightYaw;

  brain->client->moveHead(pitch, yaw);
  return NodeStatus::SUCCESS;
}

NodeStatus Chase::tick() {
  auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/Chase4", rerun::TextLog(msg));
  };
  log("ticked");

  double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
  getInput("vx_limit", vxLimit);
  getInput("vy_limit", vyLimit);
  getInput("vtheta_limit", vthetaLimit);
  getInput("dist", dist);
  getInput("safe_dist", safeDist);

  bool avoidObstacle;
  brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
  double oaSafeDist;
  brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

  if (brain->config->limitNearBallSpeed &&
      brain->data->ball.range < brain->config->nearBallRange) {
    vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
  }

  double ballRange = brain->data->ball.range;
  double ballYaw = brain->data->ball.yawToRobot;
  double kickDir = brain->data->kickDir;
  double theta_br =
      atan2(brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
            brain->data->robotPoseToField.x - brain->data->ball.posToField.x);
  double theta_rb = brain->data->robotBallAngleToField;
  auto ballPos = brain->data->ball.posToField;

  double vx, vy, vtheta;
  Pose2D target_f, target_r;
  static string targetType = "direct";
  static double circleBackDir = 1.0;
  double dirThreshold = M_PI / 2;
  if (targetType == "direct")
    dirThreshold *= 1.2;

  // 计算目标点
  if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
    log("targetType = direct");
    targetType = "direct";
    target_f.x = ballPos.x - dist * cos(kickDir);
    target_f.y = ballPos.y - dist * sin(kickDir);
  } else {
    targetType = "circle_back";
    double cbDirThreshold = 0.0;
    cbDirThreshold -= 0.2 * circleBackDir;
    circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
    log(format("targetType = circle_back, circleBackDir = %.1f",
               circleBackDir));
    double tanTheta =
        theta_br +
        circleBackDir * acos(min(1.0, safeDist / max(ballRange, 1e-5)));
    target_f.x = ballPos.x + safeDist * cos(tanTheta);
    target_f.y = ballPos.y + safeDist * sin(tanTheta);
  }
  target_r = brain->data->field2robot(target_f);
  brain->log->setTimeNow();
  brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}),
                      0xFFFFFFFF, false, false);

  double targetDir = atan2(target_r.y, target_r.x);
  double distToObstacle = brain->distToObstacle(targetDir);
  if (avoidObstacle && distToObstacle < oaSafeDist) {
    log("avoid obstacle");
    auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
    const double speed = 0.5;
    vx = speed * cos(avoidDir);
    vy = speed * sin(avoidDir);
    vtheta = ballYaw;
  } else {
    vx = min(vxLimit, brain->data->ball.range);
    vy = 0;
    vtheta = targetDir;
    if (fabs(targetDir) < 0.1 && ballRange > 2.0)
      vtheta = 0.0;
    vx *= sigmoid((fabs(vtheta)), 1, 3);
  }

  vx = cap(vx, vxLimit, -vxLimit);
  vy = cap(vy, vyLimit, -vyLimit);
  vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

  static double smoothVx = 0.0;
  static double smoothVy = 0.0;
  static double smoothVtheta = 0.0;
  smoothVx = smoothVx * 0.7 + vx * 0.3;
  smoothVy = smoothVy * 0.7 + vy * 0.3;
  smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

  // brain->client->setVelocity(smoothVx, smoothVy, smoothVtheta, false, false,
  // false);
  brain->client->setVelocity(vx, vy, vtheta, false, false, false);
  return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick() {
  double stopDist, stopAngle, vyLimit, vxLimit;
  getInput("stop_dist", stopDist);
  getInput("stop_angle", stopAngle);
  getInput("vx_limit", vxLimit);
  getInput("vy_limit", vyLimit);

  if (!brain->tree->getEntry<bool>("ball_location_known")) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  double vx = brain->data->ball.posToRobot.x;
  double vy = brain->data->ball.posToRobot.y;
  double vtheta = brain->data->ball.yawToRobot * 4.0;

  double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range *
                                          fabs(brain->data->ball.yawToRobot)) -
                                     3));
  vx *= linearFactor;
  vy *= linearFactor;

  vx = cap(vx, vxLimit, -1.0);
  vy = cap(vy, vyLimit, -vyLimit);

  if (brain->data->ball.range < stopDist) {
    vx = 0;
    vy = 0;
    // if (fabs(brain->data->ball.yawToRobot) < stopAngle) vtheta = 0;
  }

  brain->client->setVelocity(vx, vy, vtheta, false, false, false);
  return NodeStatus::SUCCESS;
}

NodeStatus GoToFreekickPosition::onStart() {
  // brain->log->log("debug/freekick_position/onStart",
  // rerun::TextLog(format("stage onStart")));
  _isInFinalAdjust = false;
  return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
    };
    log("running");

    string side;
    getInput("side", side);
    string freeKickType = brain->data->realGameSubState;
    if (side != "attack" && side != "defense")
        return NodeStatus::SUCCESS;

    Pose2D targetPose;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    if (side == "attack") {
        Point2D goalAreaMidPoint;
        goalAreaMidPoint.x = fd.length / 2.0 - fd.goalAreaLength;
        goalAreaMidPoint.y = 0;
        double targetDir = atan2(goalAreaMidPoint.y - ballPos.y, goalAreaMidPoint.x - ballPos.x);
        double dist = brain->config->adjustDists.kickinho;

        targetPose.x = ballPos.x - dist * cos(targetDir);
        targetPose.y = ballPos.y - dist * sin(targetDir);
        targetPose.theta = targetDir;

        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2) {
            if (!brain->isPrimaryStriker()) {
                targetPose.y = 0;
                targetPose.x -= 1.5;
                if (targetPose.x < -fd.length / 2.0 + fd.goalAreaLength)
                targetPose.x = -fd.length / 2.0 + fd.goalAreaLength;
                auto buffer = 2.0;
                auto targetXPose = brain->config->fieldDimensions.length / 2 - buffer;
                if (targetPose.x > targetXPose) {
                targetPose.x = targetXPose;
                targetPose.theta = 0;
                }
            }
        }
    }
    else if (side == "defense") {
        brain->log->log("debug/freekick_position", rerun::TextLog(format("I'm defending, kick type: %s", freeKickType.c_str())));
        if(freeKickType == "PENALTY_KICK" && brain->config->playerRole == "striker") {    //penalti
    
            brain->log->log("debug/freekick_position", rerun::TextLog(format("I'm a striker and i'm defending, kick type: %s", freeKickType.c_str()))); //dos strikers
            
            targetPose.y = brain->isPrimaryStriker() ? ballPos.y - 1.0 : ballPos.y + 1.0;
            targetPose.x = 0;
            targetPose.theta = atan2(
            brain->data->ball.posToField.y - brain->data->robotPoseToField.y,
            brain->data->ball.posToField.x - brain->data->robotPoseToField.x); 

        }
        else {
            brain->log->log("debug/freekick_position", rerun::TextLog(format("Other free kick type: %s", freeKickType.c_str())));
            double targetDir = atan2(ballPos.y, ballPos.x + fd.length / 2);
            double dist;
            getInput("defense_dist", dist);
            targetPose.x = ballPos.x - dist * cos(targetDir);
            targetPose.y = ballPos.y - dist * sin(targetDir);
            targetPose.theta = targetDir;
            if (ballPos.x < -fd.length / 2 + 1.0)  targetPose.x = -fd.length / 2 + 1.5;

            if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
            {
                if (!brain->isPrimaryStriker()) {
                    targetPose.y = targetPose.y > 0 ? targetPose.y - 1.0 : targetPose.y + 1.0;
                }
            }
        }
    }

  double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
  double deltaDir = toPInPI(targetPose.theta - robotPose.theta);

  if (dist < 0.2 && fabs(deltaDir) < 0.1) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  if (!brain->get_parameter("obstacle_avoidance.enable_freekick_avoid")
           .as_bool() ||
      dist < 1.0 || _isInFinalAdjust) {
    _isInFinalAdjust = true;
    auto targetPose_r = brain->data->field2robot(targetPose);

    double vx = targetPose_r.x;
    double vy = targetPose_r.y;
    double vtheta = brain->data->ball.yawToRobot * 4.0;

    double linearFactor =
        1 / (1 + exp(3 * (brain->data->ball.range *
                          fabs(brain->data->ball.yawToRobot)) -
                     3));
    vx *= linearFactor;
    vy *= linearFactor;

    Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
    if (pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.5 &&
        brain->data->ball.range < 1.0) {
      vx = min(0.0, vx);
      vy = vy >= 0 ? vy + 0.1 : vy - 0.1;
    }

    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -1.0);
    vy = cap(vy, vyLimit, -vyLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::RUNNING;
  }

  double longRangeThreshold = 1.0;
  double turnThreshold = 0.4;
  double vxLimit = 0.6;
  double vyLimit = 0.5;
  double vthetaLimit = 1.5;
  bool avoidObstacle = true;
  // brain->log->log("debug/freekick_position", rerun::TextLog(format("stage
  // move: targetPose: (%.2f, %.2f, %.2f)", targetPose.x, targetPose.y,
  // targetPose.theta)));
  brain->client->moveToPoseOnField3(targetPose.x, targetPose.y,
                                    targetPose.theta, longRangeThreshold,
                                    turnThreshold, vxLimit, vyLimit,
                                    vthetaLimit, 0.2, 0.2, 0.1, avoidObstacle);

  return NodeStatus::RUNNING;
}

void GoToFreekickPosition::onHalted() {
  // brain->log->log("debug/freekick_position/onHault",
  // rerun::TextLog(format("stage OnHalted")));
}

NodeStatus GoToGoalBlockingPosition::tick() {
  auto log = [=](string msg) {
    // brain->log->setTimeNow();
    // brain->log->log("debug/GoToGoalBlockingPosition", rerun::TextLog(msg));
  };
  log("GoToGoalBlockingPosition ticked");

  // brain->log->setTimeNow();
  // brain->log->log("tree/GoToGoalBlockingPosition",
  // rerun::TextLog("GoToGoalBlockingPosition tick"));

  double distTolerance = getInput<double>("dist_tolerance").value();
  double thetaTolerance = getInput<double>("theta_tolerance").value();
  double distToGoalline = getInput<double>("dist_to_goalline").value();

  auto fd = brain->config->fieldDimensions;
  auto ballPos = brain->data->ball.posToField;
  auto robotPose = brain->data->robotPoseToField;

  string curRole = brain->tree->getEntry<string>("player_role");

  Pose2D targetPose;
  targetPose.x =
      curRole == "striker"
          ? (std::max(-fd.length / 2.0 + distToGoalline, ballPos.x - 1.5))
          : (-fd.length / 2.0 + distToGoalline);
  if (ballPos.x + fd.length / 2.0 < distToGoalline) {
    targetPose.y =
        curRole == "striker"
            ? (ballPos.y > 0 ? fd.goalWidth / 2.0 : -fd.goalWidth / 2.0)
            : (ballPos.y > 0 ? fd.goalWidth / 4.0 : -fd.goalWidth / 4.0);
  } else {
    targetPose.y = ballPos.y * distToGoalline / (ballPos.x + fd.length / 2.0);
    targetPose.y =
        curRole == "striker"
            ? (cap(targetPose.y, fd.goalWidth / 2.0, -fd.goalWidth / 2.0))
            : (cap(targetPose.y, fd.penaltyAreaWidth / 2.0,
                   -fd.penaltyAreaWidth / 2.0));
  }

  double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
  if ( // 认为到达了目标位置
      dist < distTolerance &&
      fabs(brain->data->ball.yawToRobot) < thetaTolerance) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  auto targetPose_r = brain->data->field2robot(targetPose);
  double vx = targetPose_r.x;
  double vy = targetPose_r.y;
  double vtheta = brain->data->ball.yawToRobot * 4.0;

  double vxLimit, vyLimit;
  getInput("vx_limit", vxLimit);
  getInput("vy_limit", vyLimit);
  vx = cap(vx, vxLimit, -vxLimit);
  vy = cap(vy, vyLimit, -vyLimit);

  brain->client->setVelocity(vx, vy, vtheta, false, false, false);
  return NodeStatus::SUCCESS;
}

NodeStatus Assist::tick() {
  auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/Assist", rerun::TextLog(msg));
  };
  log("ticked");

  double distTolerance = getInput<double>("dist_tolerance").value();
  double thetaTolerance = getInput<double>("theta_tolerance").value();
  double distToGoalline = getInput<double>("dist_to_goalline").value();

  auto fd = brain->config->fieldDimensions;
  auto ballPos = brain->data->ball.posToField;
  auto robotPose = brain->data->robotPoseToField;
  string curRole = brain->tree->getEntry<string>("player_role");

  bool isSecondary = false;
  bool has2Assists = false;
  int selfIdx = brain->config->playerId - 1;
  for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
    if (i == selfIdx)
      continue;

    auto tmStatus = brain->data->tmStatus[i];
    if (!tmStatus.isAlive)
      continue;
    if (tmStatus.isLead)
      continue;
    if (tmStatus.role != "striker")
      continue;

    has2Assists = true;
    log("2 assists found");
    if (tmStatus.robotPoseToField.x > robotPose.x) {
      log("i am secondary");
      isSecondary = true;
    }
  }
  log(format("has2Assists: %d, isSecondary: %d", has2Assists, isSecondary));

  Pose2D targetPose;
  targetPose.x = isSecondary ? ballPos.x - 4.0 : ballPos.x - 2.0;
  targetPose.x = max(targetPose.x, -fd.length / 2.0 + distToGoalline);
  targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) /
                 (ballPos.x + fd.length / 2.0);
  if (has2Assists) {
    targetPose.y += isSecondary ? -0.5 : 0.5;
  }

  double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
  if (dist < distTolerance &&
      fabs(brain->data->ball.yawToRobot) < thetaTolerance) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  double vx, vy, vtheta;
  auto targetPose_r = brain->data->field2robot(targetPose);
  double targetDir = atan2(targetPose_r.y, targetPose_r.x);
  double distToObstacle = brain->distToObstacle(targetDir);

  bool avoidObstacle;
  brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
  double oaSafeDist;
  brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

  if (avoidObstacle && distToObstacle < oaSafeDist) {
    log("avoid obstacle");
    auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
    const double speed = 0.5;
    vx = speed * cos(avoidDir);
    vy = speed * sin(avoidDir);
    vtheta = brain->data->ball.yawToRobot;
  } else {
    vx = targetPose_r.x;
    vy = targetPose_r.y;
    vtheta = brain->data->ball.yawToRobot * 4.0;
  }

  double vxLimit, vyLimit;
  getInput("vx_limit", vxLimit);
  getInput("vy_limit", vyLimit);
  vx = cap(vx, vxLimit, -1.0);
  vy = cap(vy, vyLimit, -vyLimit);

  brain->client->setVelocity(vx, vy, vtheta, false, false, false);
  return NodeStatus::SUCCESS;
}

NodeStatus Adjust::onStart() {
  if (brain->data->kickType == "drible")
  {
    auto log = [=](string msg) { 
        brain->log->setTimeNow();
        brain->log->log("debug/adjust5", rerun::TextLog(msg)); 
    };
    log("enter");
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f", brain->data->ball.posToRobot.x, brain->data->ball.posToRobot.y, brain->data->ball.yawToRobot));
    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);


    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    // double st = cap(fabs(deltaDir), st_far, st_near);
    double st = st_far; 
    double R = ballRange; 
    double r = range;
    double sr = cap(R - r, 0.5, 0); 
    log(format("R: %.2f, r: %.2f, sr: %.2f", R, r, sr));

    log(format("deltaDir = %.1f", deltaDir));
    if (fabs(deltaDir) * R < NEAR_THRESHOLD) {
        log("use near speed");
        st = st_near;
        // sr = 0.;
        // vxLimit = 0.1;
    }

    double theta_robot_f = brain->data->robotPoseToField.theta; 
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; 
    double thetar_r = dir_rb_f - theta_robot_f; 

    vx = st * cos(thetat_r) + sr * cos(thetar_r); 
    vy = st * sin(thetat_r) + sr * sin(thetar_r); 
    // vtheta = toPInPI(ballYaw + st / R * (deltaDir > 0 ? 1.0 : -1.0)); 
    vtheta = ballYaw;
    vtheta *= vtheta_factor; 

    if (fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.; 
    if (
        fabs(ballYaw) > TURN_FIRST_THRESHOLD 
        && fabs(deltaDir) < M_PI / 4
    ) { 
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
    
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::RUNNING;
}

NodeStatus Adjust::onRunning() {
  auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/adjust5", rerun::TextLog(msg));
  };
  log("enter");
  if (!brain->tree->getEntry<bool>("ball_location_known")) {
    return NodeStatus::SUCCESS;
  }

  Point2D target_pass, ball_pos, target_pos, robot_pos;

  target_pass.x = brain->config->fieldDimensions.length / 2.0; // Gol
  target_pass.y = 0;

  ball_pos.x = brain->data->ball.posToField.x;
  ball_pos.y = brain->data->ball.posToField.y;

  robot_pos.x = brain->data->robotPoseToField.x;
  robot_pos.y = brain->data->robotPoseToField.y;

  double dist; // 1.45 para fancyKick, 0.2 para highKick

  if(brain->data->kickType == "kickinho") {
    dist = brain->config->adjustDists.kickinho;
  }
  else if(brain->data->kickType == "kick"){
    dist = brain->config->adjustDists.kick;
  }
  else if(brain->data->kickType == "kickao"){
    dist = brain->config->adjustDists.kickao;
  }
  else if(brain->data->kickType == "finesse"){
    dist = brain->config->adjustDists.finesse;
  }
  else if(brain->data->kickType == "conduzir"){
    dist = brain->config->adjustDists.kickinho;
    target_pass.x = brain->config->fieldDimensions.length / 2.0 - brain->config->fieldDimensions.goalAreaLength; // Gol
  }
  else if(brain->data->kickType == "shoot"){
    dist = brain->config->adjustDists.kickinho;
  }
  else{
    dist = brain->config->adjustDists.drible;
    // target_pass = direção de drible
    vector<Point2D> dummy;
    target_pass = brain->findDribleDir(dummy);
  }

  // double dynamicDist = min(max(dist, robot_pos.distanceToPoint(ball_pos) - 0.2), max(0.5, dist));
  double dynamicDist = dist;
  
  target_pos = ((target_pass - ball_pos).normalized()) * -dynamicDist + ball_pos;

  double ballYaw = atan2(brain->data->ball.posToField.y - target_pos.y,
                         brain->data->ball.posToField.x - target_pos.x);
  double distTol = brain->config->adjustDists.tolerance,
         contourDist = 0.6;

  if(ball_pos.interceptLine(robot_pos, target_pos, 0.30)) // Contornar
  {
      target_pos = (robot_pos - ball_pos).perpendVector(ball_pos - target_pos).normalized() * contourDist + ball_pos;
      if (target_pos.distanceToPoint(robot_pos) < distTol)
      {
        target_pos = ((target_pass - ball_pos).normalized()) * -dynamicDist + ball_pos;
      }
  }

  Point2D leftPost, rightPost;
  leftPost.x = brain->config->fieldDimensions.length / 2.0;
  leftPost.y = brain->config->fieldDimensions.goalWidth / 2.0;
  rightPost.x = brain->config->fieldDimensions.length / 2.0;
  rightPost.y = -brain->config->fieldDimensions.goalWidth / 2.0;

  auto goalposts = brain->data->getGoalposts();
  for (int i = 0; i < goalposts.size(); i++)
  {
      auto post = goalposts[i];
      if (post.name == "OL")
      {
          leftPost.x = post.posToField.x;
          leftPost.y = post.posToField.y;
      }
      else if (post.name == "OR")
      {
          rightPost.x = post.posToField.x;
          rightPost.y = post.posToField.y;
      }
  }
          
  if (ball_pos.x > brain->config->fieldDimensions.length / 2.0 - 0.5 &&
      ball_pos.y > -brain->config->fieldDimensions.goalWidth / 2.0 - 0.1 && 
      ball_pos.y < brain->config->fieldDimensions.goalWidth / 2.0 + 0.1)
      // (ball_pos.distanceToPoint(leftPost) < 0.4 ||
      //  ball_pos.distanceToPoint(rightPost) < 0.4))
  {
      target_pass = ball_pos;
      target_pass.x += 1.0;
      target_pos = ((target_pass - ball_pos).normalized()) * -dynamicDist + ball_pos;
      ballYaw = atan2(brain->data->ball.posToField.y - target_pos.y,
                      brain->data->ball.posToField.x - target_pos.x);
  } // Bola no gol


  brain->client->moveToPoseOnField3(target_pos.x, target_pos.y, ballYaw,
                                    1.5, 0.4,
                                    1.0, 0.7, 0.6, // limites de velocidade
                                    distTol*0.8, distTol*0.8, deg2rad(5)); // limites de tolerância

  brain->log->setTimeNow();
  brain->log->logBall("field/adjust_target", Point({target_pos.x, target_pos.y, 0}),
                      0xFFFFFFFF, false, false);

  double robot_dist_target = robot_pos.distanceToPoint(target_pos),
         robot_angle_error = fabs(ballYaw - brain->data->robotPoseToField.theta) * 180.0/M_PI;

  brain->log->log("striker/adjustSucessCond", rerun::TextLog(format("DistTarget: %.3f < 0.1 | RobotAngle: %.3f < 10",
                                                            robot_dist_target, robot_angle_error)));

  if (robot_dist_target < distTol && robot_angle_error < 7)
  {

    brain->client->setVelocity(0.0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  return NodeStatus::RUNNING;
}

void Adjust::onHalted() {
}

NodeStatus CalcKickDir::tick() {
  // 读取和处理参数
  double crossThreshold;
  getInput("cross_threshold", crossThreshold);

  string lastKickType = brain->data->kickType;
  if (lastKickType == "cross")
    crossThreshold += 0.1;

  auto gpAngles = brain->getGoalPostAngles(0.0);
  auto thetal = gpAngles[0];
  auto thetar = gpAngles[1];
  Point2D bPos = Point2D{brain->data->ball.posToField.x,
                      brain->data->ball.posToField.y};
  auto fd = brain->config->fieldDimensions;
  auto color = 0xFFFFFFFF; // for log
  auto goalPostAngles = brain->getGoalPostAngles(brain->config->goalPostMargin);
  double theta_l = goalPostAngles[0]; 
  double theta_r = goalPostAngles[1]; 

  bool angleGood = brain->isAngleGood() && fabs(theta_l - theta_r) > brain->config->avoidAngleDegrees * M_PI / 180.0;

  // if (thetal - thetar < crossThreshold &&
  //     brain->data->ball.posToField.x > fd.circleRadius) 
  // {
  //   brain->data->kickType = "cross";
  //   color = 0xFF00FFFF;
  //   brain->data->kickDir =
  //       atan2(-bPos.y, fd.length / 2 - fd.penaltyDist / 2 - bPos.x);
  // }
  // else if (brain->isDefensing()) 
  // {
  //   brain->data->kickType = "block";
  //   color = 0xFFFF00FF;
  //   brain->data->kickDir = atan2(bPos.y, bPos.x + fd.length / 2);
  //
  // } 
  bool drible = false;
  auto opponents = brain->data->getRobots();
  Point2D oppPos;
  vector<Point2D> dummy;


  brain->log->setTimeNow();
  vector<rerun::LineStrip2D> circles = {};
  circles.push_back(brain->log->circle(bPos.x,
                                -bPos.y, brain->config->dribleRadius));
  brain->log->log("striker/drible_radius",
      rerun::LineStrips2D(rerun::Collection<rerun::components::LineStrip2D>(circles))
      .with_radii(0.01)
      .with_colors(0xFF00FFFF));

  for (auto &opp : opponents)
  {
    oppPos.x = opp.posToField.x;
    oppPos.y = opp.posToField.y;

    drible = drible | oppPos.pointInsideCircle(brain->config->dribleRadius,
                                               bPos,
                                               brain->config->dribleCircleDegrees,
                                               brain->data->robotPoseToField.theta,
                                               dummy);
    if (drible)
      break;
  }

  bool enableKick = brain->config->enableKick;

  if (!drible)
  {
    if (bPos.x < 0 || fabs(theta_l - theta_r) < brain->config->avoidAngleDegrees * M_PI / 180.0)
    {
      brain->data->kickType = "conduzir";
    }
    else if (bPos.isInsideKickinho(fd.length, brain->config->kickRadiusToGoal))
    {
      brain->data->kickType = "kickinho";
    }
    else if (bPos.isInsideKick(fd.length, brain->config->kickRadiusToGoal, angleGood))
    {
      if (enableKick)
      {
        brain->data->kickType = "kick";
      }
      else
      {
        brain->data->kickType = "kickinho";
      }
    }
    else {
      brain->data->kickType = "kickinho";
      color = 0x00FF00FF;
      brain->data->kickDir = atan2(-bPos.y, fd.length / 2 - bPos.x);
      // if (brain->data->ball.posToField.x >
      //     brain->config->fieldDimensions.length / 2)
      //   brain->data->kickDir = 0;
    }

    if (brain->data->doingPenaltyKick && bPos.x > 0){
      brain->data->kickType = "kickao";
    }

    if (brain->data->doingKickingOff && bPos.x > 0){
      brain->data->kickType = "finesse";
    }
  }
  else
  {
      brain->data->kickType = "drible";
      brain->log->setTimeNow();
      brain->log->logDrible("striker/drible_dir");
  }

  brain->log->setTimeNow();
  brain->log->log("striker/kickType", rerun::TextLog(format("KickType: %s",
                                                            brain->data->kickType.c_str())));

  brain->log->setTimeNow();
  brain->log->log(
      "field/kick_dir",
      rerun::Arrows2D::from_vectors(
          {{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
          .with_origins({{brain->data->ball.posToField.x,
                          -brain->data->ball.posToField.y}})
          .with_colors({color})
          .with_radii(0.01)
          .with_draw_order(31));

  return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick() {
  auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/striker_decide", rerun::TextLog(msg));
  };

  double chaseRangeThreshold;
  getInput("chase_threshold", chaseRangeThreshold);
  string lastDecision, position;
  getInput("decision_in", lastDecision);
  getInput("position", position);

  double kickDir = brain->data->kickDir;
  double dir_rb_f = brain->data->robotBallAngleToField;
  auto ball = brain->data->ball;
  double ballRange = ball.range;
  double ballYaw = ball.yawToRobot;
  double ballX = ball.posToRobot.x;
  double ballY = ball.posToRobot.y;

  const double goalpostMargin = 0.3;
  bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");

  bool avoidPushing;
  double kickAoSafeDist;
  brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
  brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
  bool avoidKick =
      avoidPushing &&
      brain->data->robotPoseToField.x <
          brain->config->fieldDimensions.length / 2 -
              brain->config->fieldDimensions.goalAreaLength &&
      brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

  log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: "
             "%.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
             ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f,
             angleGoodForKick));

  double deltaDir = toPInPI(kickDir - dir_rb_f);
  auto now = brain->get_clock()->now();
  auto dt = brain->msecsSince(timeLastTick);
  bool reachedKickDir =
      deltaDir * lastDeltaDir <= 0 && fabs(deltaDir) < M_PI / 6 && dt < 100;
  reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
  timeLastTick = now;
  lastDeltaDir = deltaDir;

  string newDecision;
  auto color = 0xFFFFFFFF;
  bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
  bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");

  double currentBallKickDist = 0.0, distTol = brain->config->adjustDists.tolerance;
  auto goalPostAngles = brain->getGoalPostAngles(brain->config->goalPostMargin);
  double theta_l = goalPostAngles[0]; 
  double theta_r = goalPostAngles[1]; 

  if(brain->data->kickType == "kickinho"){
    currentBallKickDist = brain->config->adjustDists.kickinho;
  }
  else if(brain->data->kickType == "conduzir"){
    currentBallKickDist = brain->config->adjustDists.kickinho;
    reachedKickDir = fabs(theta_l - theta_r) < deg2rad(brain->config->avoidAngleDegrees);
  }
  else if(brain->data->kickType == "kick"){
    currentBallKickDist = brain->config->adjustDists.kick;
  }
  else if(brain->data->kickType == "kickao"){
    currentBallKickDist = brain->config->adjustDists.kickao;
  }
  else if(brain->data->kickType == "finesse"){
    currentBallKickDist = brain->config->adjustDists.finesse;
  }


  if (!(iKnowBallPos || tmBallPosReliable)) {
    newDecision = "find";
    color = 0xFFFFFFFF;
  } else if (!brain->data->tmImLead) {
    newDecision = "assist";
    color = 0x00FFFFFF;
  } else if (ballRange >
             chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0)) {
    newDecision = "chase";
    color = 0x0000FFFF;
  } 
  else if (((angleGoodForKick && !brain->data->isFreekickKickingOff) || reachedKickDir) &&
             brain->data->ballDetected && fabs(brain->data->ball.yawToRobot) < 15.0 * M_PI/180.0 && 
            !avoidKick && ball.range < currentBallKickDist + distTol)  // ball.range = 1.6 para fancyKick
  {
    if (brain->data->kickType == "cross")
      newDecision = "kick";
    else
      newDecision = "kick";
    color = 0x00FF00FF;
    brain->data->isFreekickKickingOff = false;
  } else {
    newDecision = "adjust";
    color = 0xFFFF00FF;
  }

  setOutput("decision_out", newDecision);
  brain->log->logToScreen(
      "tree/Decide",
      format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: "
             "%.2f angleGoodForKick: %d lead: %d",
             newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f,
             angleGoodForKick, brain->data->tmImLead),
      color);
  return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick() {

  double chaseRangeThreshold;
  getInput("chase_threshold", chaseRangeThreshold);
  string lastDecision, position;
  getInput("decision_in", lastDecision);

  double kickDir = atan2(brain->data->ball.posToField.y,
                         brain->data->ball.posToField.x +
                             brain->config->fieldDimensions.length / 2);
  double dir_rb_f = brain->data->robotBallAngleToField;
  auto goalPostAngles = brain->getGoalPostAngles(0.3);
  double theta_l = goalPostAngles[0];
  double theta_r = goalPostAngles[1];
  bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
  double ballRange = brain->data->ball.range;
  double ballYaw = brain->data->ball.yawToRobot;

  string newDecision;
  auto color = 0xFFFFFFFF;
  bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
  bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
  if (!(iKnowBallPos || tmBallPosReliable)) {
    newDecision = "find";
    color = 0x0000FFFF;
  } else if (brain->data->ball.posToField.x >
             0 - static_cast<double>(lastDecision == "retreat")) {
    newDecision = "retreat";
    color = 0xFF00FFFF;
  } else if (ballRange >
             chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0)) {
    newDecision = "chase";
    color = 0x00FF00FF;
  } else if (angleIsGood) {
    newDecision = "kick";
    color = 0xFF0000FF;
  } else {
    newDecision = "adjust";
    color = 0x00FFFFFF;
  }

  setOutput("decision_out", newDecision);
  brain->log->logToScreen("tree/Decide",
                          format("Decision: %s ballrange: %.2f ballyaw: %.2f "
                                 "kickDir: %.2f rbDir: %.2f angleIsGood: %d",
                                 newDecision.c_str(), ballRange, ballYaw,
                                 kickDir, dir_rb_f, angleIsGood),
                          color);
  return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed() {
  double vx, vy, msecKick;

  double vxLimit, vyLimit;
  getInput("vx_limit", vxLimit);
  getInput("vy_limit", vyLimit);
  int minMSecKick;
  getInput("min_msec_kick", minMSecKick);
  double vxFactor = brain->config->vxFactor;
  double yawOffset = brain->config->yawOffset;

  double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
  double tx = cos(adjustedYaw) * brain->data->ball.range;
  double ty = sin(adjustedYaw) * brain->data->ball.range;

  if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01) {
    vx = vxLimit;
    vy = 0.0;
  } else {
    vy = ty > 0 ? vyLimit : -vyLimit;
    vx = vy / ty * tx * vxFactor;
    if (fabs(vx) > vxLimit) {
      vy *= vxLimit / vx;
      vx = vxLimit;
    }
  }

  double speed = norm(vx, vy);
  msecKick = speed > 1e-5
                 ? minMSecKick +
                       static_cast<int>(brain->data->ball.range / speed * 1000)
                 : minMSecKick;

  return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart() {

  if (brain->data->kickType == "drible")
  {
    _minRange = brain->data->ball.range;
    _speed = 0.5;
    _startTime = brain->get_clock()->now();


    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    string role = brain->tree->getEntry<string>("player_role");
    if (
        avoidPushing
        && (role != "goal_keeper")
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 发布运动指令
    double angle = brain->data->ball.yawToRobot;
    brain->client->crabWalk(angle, _speed);
    return NodeStatus::RUNNING;
  }

  _minRange = brain->data->ball.range;
  _speed = 0.5;
  // _startTime = brain->get_clock()->now();

  bool avoidPushing;
  double kickAoSafeDist;
  brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
  brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
  string role = brain->tree->getEntry<string>("player_role");
  if (avoidPushing && (role != "goal_keeper") &&
      brain->data->robotPoseToField.x <
          brain->config->fieldDimensions.length / 2 -
              brain->config->fieldDimensions.goalAreaLength &&
      brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist) {
    brain->client->setVelocity(-0.1, 0, 0);
    return NodeStatus::SUCCESS;
  }

  //---- Kick Set Velocity
  if(brain->data->kickType == "kickinho" || brain->data->kickType == "shoot" || brain->data->kickType == "conduzir"){
    _startTime = brain->get_clock()->now();
    double vxLimit = 1.2, vyLimit = 0.6;
    if(brain->data->kickType == "conduzir")
    {
      vxLimit = 0.6;
      auto goalPostAngles = brain->getGoalPostAngles(brain->config->goalPostMargin);
      double theta_l = goalPostAngles[0]; 
      double theta_r = goalPostAngles[1]; 
      if(fabs(theta_l - theta_r) < brain->config->avoidAngleDegrees * M_PI / 180.0)
      {
        vxLimit = 1.2;
      }
    }

    // getInput("vx_limit", vxLimit);
    // getInput("vy_limit", vyLimit);
    int minMSecKick = 1000;
    // getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;
    double yawOffset = brain->config->yawOffset;

    double adjustedYaw = brain->data->ball.yawToRobot - yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range;
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    double vx, vy;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    {
        vx = vxLimit;
        vy = 0.0;
    }
    else
    {
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    double speed = norm(vx, vy);

    _msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;

    brain->client->setVelocity(vx, vy, 0, false, false, false);
    brain->log->setTimeNow();
    brain->log->log("striker/KickinhoVel",
                    rerun::TextLog(format("VX: %.3f, VY: %.3f", vx, vy)));
    //---- Kick Set Velocity
  }else{
    _msecKick = 3000;
    _startTime = brain->get_clock()->now() - rclcpp::Duration(100, 0); 
  }

  return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning() {
  auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/Kick", rerun::TextLog(msg));
  };

  if (brain->data->kickType == "drible")
  {
    bool enableAbort;
    brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;
    const double BALL_LOST_THRESHOLD = 1000;  
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) 
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD 
        )
    ) {
        log("ball moved, abort kick");
        return NodeStatus::SUCCESS;
    }


    if (ballRange < _minRange) _minRange = ballRange;    

    
    bool avoidPushing;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    if (
        avoidPushing
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double msecs = getInput<double>("min_msec_kick").value();
    double speed = getInput<double>("speed_limit").value();
    msecs = msecs + brain->data->ball.range / speed * 1000;
    if (brain->msecsSince(_startTime) > msecs) { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }


    if (brain->data->ballDetected) { 
        double angle = brain->data->ball.yawToRobot;
        double speed = getInput<double>("speed_limit").value();
        _speed += 0.1; 
        speed = min(speed, _speed);
        brain->client->crabWalk(angle, speed);
    }

    return NodeStatus::RUNNING;
  }

  bool enableAbort;
  brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
  auto ballRange = brain->data->ball.range;
  const double MOVE_RANGE_THRESHOLD = 0.3;
  const double BALL_LOST_THRESHOLD = 1000;
  if (enableAbort &&
      ((brain->data->ballDetected &&
        ballRange - _minRange > MOVE_RANGE_THRESHOLD) ||
       brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD)) {
    log("ball moved, abort kick");
    return NodeStatus::SUCCESS;
  }

  if (ballRange < _minRange)
    _minRange = ballRange;

  bool avoidPushing;
  brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
  double kickAoSafeDist;
  brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
  if (avoidPushing &&
      brain->data->robotPoseToField.x <
          brain->config->fieldDimensions.length / 2 -
              brain->config->fieldDimensions.goalAreaLength &&
      brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist) {
    brain->client->setVelocity(-0.1, 0, 0);
    return NodeStatus::SUCCESS;
  }

  // double msecs = getInput<double>("min_msec_kick").value();
  // double speed = getInput<double>("speed_limit").value();
  // msecs = msecs + brain->data->ball.range / speed * 1000;
  // if (brain->msecsSince(_startTime) > msecs) {
  //   brain->client->setVelocity(0, 0, 0);
  //   return NodeStatus::SUCCESS;
  // }


  if (brain->msecsSince(_startTime) > _msecKick) 
  {
    brain->client->setVelocity(0, 0, 0);
    if(brain->data->ballDetected && brain->data->kickType == "kick"){
      brain->client->highKick();
    }
    else if(brain->data->ballDetected && brain->data->kickType == "kickao"){
      brain->client->fancyKick(1.0, 0.9);

      Point2D currentBallPos = Point2D{brain->data->ball.posToField.x,
                                       brain->data->ball.posToField.y};

      if (brain->data->doingPenaltyKick && 
        brain->data->ballPosDuringPenalty.distanceToPoint(currentBallPos) > 1.0)
      {
        brain->data->doingPenaltyKick = false;
      }
    }
    else if(brain->data->ballDetected && brain->data->kickType == "finesse"){
      //brain->client->rlKickBall(1.0, 0.4);
      brain->client->highKick();

      Point2D currentBallPos = Point2D{brain->data->ball.posToField.x,
                                       brain->data->ball.posToField.y};

      if (brain->data->doingKickingOff && 
        brain->data->ballPosDuringKickingOff.distanceToPoint(currentBallPos) > 1.0)
      {
        brain->data->doingKickingOff = false;
      }
    }
    _startTime = brain->get_clock()->now();

    return NodeStatus::SUCCESS;
  }

  return NodeStatus::RUNNING;

}

void Kick::onHalted() { _startTime -= rclcpp::Duration(100, 0); }

NodeStatus StandStill::onStart() {

  _startTime = brain->get_clock()->now();

  brain->client->setVelocity(0, 0, 0);
  return NodeStatus::RUNNING;
}

NodeStatus StandStill::onRunning() {
  double msecs;
  getInput("msecs", msecs);
  if (brain->msecsSince(_startTime) < msecs) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::RUNNING;
  }

  return NodeStatus::SUCCESS;
}

void StandStill::onHalted() {
  double msecs;
  getInput("msecs", msecs);
  _startTime -= rclcpp::Duration(-2 * msecs, 0);
}

NodeStatus RobotFindBall::onStart() {
  auto log = [=](string msg) {
    // brain->log->setTimeNow();
    // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
  };
  log("RobotFindBall onStart");

  if (brain->data->ballDetected) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }
  _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;

  return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning() {
  auto log = [=](string msg) {
    // brain->log->setTimeNow();
    // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
  };
  log("RobotFindBall onRunning");

  if (brain->data->ballDetected) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  double vyawLimit;
  getInput("vyaw_limit", vyawLimit);

  double vx = 0;
  double vy = 0;
  double vtheta = 0;
  if (brain->data->ball.range < 0.3) {
    // vx = cap(-brain->data->ball.posToRobot.x, 0.2, -0.2);
    // vy = cap(-brain->data->ball.posToRobot.y, 0.2, -0.2);
  }
  // vtheta = _turnDir > 0 ? vyawLimit : -vyawLimit;
  brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
  return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted() {
  auto log = [=](string msg) {
    // brain->log->setTimeNow();
    // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
  };
  log("RobotFindBall onHalted");
  _turnDir = 1.0;
}

NodeStatus CamFastScan::onStart() {
  _cmdIndex = 0;
  _timeLastCmd = brain->get_clock()->now();
  brain->client->moveHead(_cmdSequence[_cmdIndex][0],
                          _cmdSequence[_cmdIndex][1]);
  return NodeStatus::RUNNING;
}

NodeStatus CamFastScan::onRunning() {
  double interval = getInput<double>("msecs_interval").value();
  if (brain->msecsSince(_timeLastCmd) < interval)
    return NodeStatus::RUNNING;

  // else
  if (_cmdIndex >= 6)
    return NodeStatus::SUCCESS;

  // else
  _cmdIndex++;
  _timeLastCmd = brain->get_clock()->now();
  brain->client->moveHead(_cmdSequence[_cmdIndex][0],
                          _cmdSequence[_cmdIndex][1]);
  return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onStart() {
  _timeStart = brain->get_clock()->now();
  _lastAngle = brain->data->robotPoseToOdom.theta;
  _cumAngle = 0.0;

  bool towardsBall = false;
  _angle = getInput<double>("rad").value();
  getInput("towards_ball", towardsBall);
  if (towardsBall) {
    double ballPixX = (brain->data->ball.boundingBox.xmin +
                       brain->data->ball.boundingBox.xmax) /
                      2;
    _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
  }

  brain->client->setVelocity(0, 0, _angle, false, false, true);
  return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning() {
  double curAngle = brain->data->robotPoseToOdom.theta;
  double deltaAngle = toPInPI(curAngle - _lastAngle);
  _lastAngle = curAngle;
  _cumAngle += deltaAngle;
  double turnTime = brain->msecsSince(_timeStart);
  // brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
  //     "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
  //     _angle, _cumAngle, deltaAngle, turnTime
  // )));
  if (fabs(_cumAngle) - fabs(_angle) > -0.1 || turnTime > _msecLimit) {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  // else
  brain->client->setVelocity(0, 0, (_angle - _cumAngle) * 2);
  return NodeStatus::RUNNING;
}

NodeStatus MoveToPoseOnField::tick() {
  auto log = [=](string msg) {
    // brain->log->setTimeNow();
    // brain->log->log("debug/Move", rerun::TextLog(msg));
  };
  log("Move ticked");

  double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit,
      vthetaLimit, xTolerance, yTolerance, thetaTolerance;
  getInput("x", tx);
  getInput("y", ty);
  getInput("theta", ttheta);
  getInput("long_range_threshold", longRangeThreshold);
  getInput("turn_threshold", turnThreshold);
  getInput("vx_limit", vxLimit);
  getInput("vx_limit", vxLimit);
  getInput("vy_limit", vyLimit);
  getInput("vtheta_limit", vthetaLimit);
  getInput("x_tolerance", xTolerance);
  getInput("y_tolerance", yTolerance);
  getInput("theta_tolerance", thetaTolerance);
  bool avoidObstacle;
  getInput("avoid_obstacle", avoidObstacle);

  brain->client->moveToPoseOnField2(
      tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit,
      vthetaLimit, xTolerance, yTolerance, thetaTolerance, avoidObstacle);
  return NodeStatus::SUCCESS;
}

NodeStatus GoToReadyPosition::tick() {
  auto log = [=](string msg) {
    // brain->log->setTimeNow();
    // brain->log->log("debug/GoToReadyPosition", rerun::TextLog(msg));
  };
  log("GoToReadyPosition ticked");

  double distTolerance, thetaTolerance;
  getInput("dist_tolerance", distTolerance);
  getInput("theta_tolerance", thetaTolerance);
  string role = brain->tree->getEntry<string>("player_role");
  bool isKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
  auto fd = brain->config->fieldDimensions;

  double tx = 0, ty = 0, ttheta = 0;
  double longRangeThreshold = 1.0;
  double turnThreshold = 0.4;
  double vxLimit, vyLimit;
  getInput("vx_limit", vxLimit);
  getInput("vy_limit", vyLimit);
  if (brain->distToBorder() > -1.0) {
    vxLimit = 0.5;
    vyLimit = 0.3;
  }
  double vthetaLimit = 1.3;
  bool avoidObstacle = true;

  if (role == "striker" && isKickoff) {
    tx = -max(fd.circleRadius + 0.5, 1.5);
    ty = 0;
    if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2) {
      if (brain->isPrimaryStriker()) {
        ty = 1.5;
      } else {
        ty = -1.5;
      }
    }
    ttheta = 0;
  } else if (role == "striker" && !isKickoff) {
    tx = -(fd.circleRadius + 0.5) * 1.0;
    ty = 0;
    if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2) {
      if (brain->isPrimaryStriker()) {
        ty = 0.4;
      } else {
        ty = -0.4;
      }
    }
    ttheta = 0;
  } else if (role == "goal_keeper") {
    tx = -fd.length / 2.0 + fd.goalAreaLength;
    ty = 0;
    ttheta = 0;
  }

  brain->client->moveToPoseOnField2(
      tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit,
      vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance,
      avoidObstacle);
  return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick() {
  auto log = [=](string msg) {
    brain->log->setTimeNow();
    brain->log->log("debug/GoBackInField", rerun::TextLog(msg));
  };
  log("GoBackInField ticked");

  double valve;
  getInput("valve", valve);
  double vx = 0;
  double vy = 0;
  double dir = 0;
  auto fd = brain->config->fieldDimensions;
  if (brain->data->robotPoseToField.x > fd.length / 2.0 - valve)
    dir = -M_PI;
  else if (brain->data->robotPoseToField.x < -fd.length / 2.0 + valve)
    dir = 0;
  else if (brain->data->robotPoseToField.y > fd.width / 2.0 + valve)
    dir = -M_PI / 2.0;
  else if (brain->data->robotPoseToField.y < -fd.width / 2.0 - valve)
    dir = M_PI / 2.0;
  else {
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
  }

  double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
  vx = 0.4 * cos(dir_r);
  vy = 0.4 * sin(dir_r);
  brain->client->setVelocity(vx, vy, 0, false, false, false);
  return NodeStatus::SUCCESS;
}

NodeStatus WaveHand::tick() {
  string action;
  getInput("action", action);
  if (action == "start")
    brain->client->waveHand(true);
  else
    brain->client->waveHand(false);
  return NodeStatus::SUCCESS;
}

NodeStatus MoveHead::tick() {
  double pitch, yaw;
  getInput("pitch", pitch);
  getInput("yaw", yaw);
  brain->client->moveHead(pitch, yaw);
  return NodeStatus::SUCCESS;
}

NodeStatus CheckAndStandUp::tick() {
  if (brain->tree->getEntry<bool>("gc_is_under_penalty") ||
      brain->data->currentRobotModeIndex == 1) {
    brain->data->recoveryPerformedRetryCount = 0;
    brain->data->recoveryPerformed = false;
    brain->log->log("recovery", rerun::TextLog("reset recovery"));
    return NodeStatus::SUCCESS;
  }
  brain->log->log(
      "recovery",
      rerun::TextLog(format("Recovery retry count: %d, recoveryPerformed: %d "
                            "recoveryState: %d currentRobotModeIndex: %d",
                            brain->data->recoveryPerformedRetryCount,
                            brain->data->recoveryPerformed,
                            brain->data->recoveryState,
                            brain->data->currentRobotModeIndex)));

  if (!brain->data->recoveryPerformed &&
      brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
      // brain->data->isRecoveryAvailable &&
      brain->data->currentRobotModeIndex == 3 &&
      brain->data->recoveryPerformedRetryCount <
          brain->get_parameter("recovery.retry_max_count").get_value<int>()) {
    brain->client->standUp();
    brain->data->recoveryPerformed = true;
    brain->speak("Trying to stand up");
    brain->log->log("recovery", rerun::TextLog(format(
                                    "Recovery retry count: %d",
                                    brain->data->recoveryPerformedRetryCount)));
    return NodeStatus::SUCCESS;
  }

  if (brain->data->recoveryPerformed &&
      brain->data->currentRobotModeIndex == 12) {
    brain->data->recoveryPerformedRetryCount += 1;
    brain->data->recoveryPerformed = false;
    brain->log->log("recovery", rerun::TextLog(format(
                                    "Add retry count: %d",
                                    brain->data->recoveryPerformedRetryCount)));
  }

  if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
      brain->data->currentRobotModeIndex == 8) {
    brain->data->recoveryPerformedRetryCount = 0;
    brain->data->recoveryPerformed = false;
    brain->log->log("recovery",
                    rerun::TextLog("Reset recovery, recoveryState: " +
                                   to_string(static_cast<int>(
                                       brain->data->recoveryState))));
  }

  return NodeStatus::SUCCESS;
}

NodeStatus CalibrateOdom::tick() {
  double x, y, theta;
  getInput("x", x);
  getInput("y", y);
  getInput("theta", theta);

  brain->calibrateOdom(x, y, theta);
  return NodeStatus::SUCCESS;
}

NodeStatus PrintMsg::tick() {
  Expected<std::string> msg = getInput<std::string>("msg");
  if (!msg) {
    throw RuntimeError("missing required input [msg]: ", msg.error());
  }
  std::cout << "[MSG] " << msg.value() << std::endl;
  return NodeStatus::SUCCESS;
}

NodeStatus PlaySound::tick() {
  string sound;
  getInput("sound", sound);
  bool allowRepeat;
  getInput("allow_repeat", allowRepeat);
  brain->playSound(sound, allowRepeat);
  return NodeStatus::SUCCESS;
}

NodeStatus Speak::tick() {
  const string lastText;
  string text;
  getInput("text", text);
  if (text == lastText)
    return NodeStatus::SUCCESS;

  brain->speak(text, false);
  return NodeStatus::SUCCESS;
}
