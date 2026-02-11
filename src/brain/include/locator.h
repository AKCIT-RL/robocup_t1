
#pragma once

#include <Eigen/Core>
#include <cstdlib> 
#include <ctime>   
#include <limits>
#include <cmath>
#include <chrono>
#include <rerun.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "types.h"

using namespace std;
namespace chr = std::chrono;


class Locator
{
public:
	
	double convergeTolerance = 0.2;	 
	double residualTolerance = 0.4;	 
	double maxIteration = 20;		 
	double muOffset = 2.0;			 
	double numShrinkRatio = 0.85;	 
	double offsetShrinkRatio = 0.8;	 
	int minMarkerCnt = 3;		 
	double enableLog = false;		 
	string logIP = "127.0.0.1:9876"; 

	
	const rerun::RecordingStream log = rerun::RecordingStream("locator", "locator");
	vector<FieldMarker> fieldMarkers;
	FieldDimensions fieldDimensions;
	Eigen::ArrayXXd hypos;				  
	PoseBox2D constraints;				  
	double offsetX, offsetY, offsetTheta; 
	Pose2D bestPose;					  
	double bestResidual;				  

	void init(FieldDimensions fd, int minMarkerCnt = 4, double residualTolerance = 0.4, double muOffsetParam = 2.0, bool enableLog = false, string logIP = "127.0.0.1:9876");

	
	void calcFieldMarkers(FieldDimensions fd);


	LocateResult locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	
	int genInitialParticles(int num = 200);

	
	int genParticles();

	
	FieldMarker markerToFieldFrame(FieldMarker marker, Pose2D pose);

	double minDist(FieldMarker marker);

	vector<double> getOffset(FieldMarker marker);

	double residual(vector<FieldMarker> markers_r, Pose2D pose);

	bool isConverged();

	int calcProbs(vector<FieldMarker> markers_r);

	Pose2D finalAdjust(vector<FieldMarker> markers_r, Pose2D pose);

	
	inline double probDesity(double r, double mu, double sigma)
	{
		if (sigma < 1e-5)
			return 0.0;
		return 1 / sqrt(2 * M_PI * sigma * sigma) * exp(-(r - mu) * (r - mu) / (2 * sigma * sigma));
	};

	void logParticles();
};



class Brain; 
using namespace BT;


void RegisterLocatorNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("mode", "enter_field", "must be one of [trust_direction, face_forward, fall_recovery]"),
            InputPort<double>("msecs_interval", 10000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
        };
    };

private:
    Brain *brain;
};

class SelfLocateEnterField : public SyncActionNode
{
public:
    SelfLocateEnterField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
        };
    };

private:
    Brain *brain;
};

class SelfLocate1M : public SyncActionNode
{
public:
    SelfLocate1M(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "marker 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocate2X : public SyncActionNode
{
public:
    SelfLocate2X(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "penalty point 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocate2T : public SyncActionNode
{
public:
    SelfLocate2T(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "两个 TCross 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocateLT : public SyncActionNode
{
public:
    SelfLocateLT(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "penalty point 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocatePT : public SyncActionNode
{
public:
    SelfLocatePT(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "penalty point 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};

class SelfLocateBorder : public SyncActionNode
{
public:
    SelfLocateBorder(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 1000, "防止过于频繁地校准, 如果上一次校准距离现在小于这个时间, 则不重新校准."),
            InputPort<double>("max_dist", 2.0, "border 距离机器人的距离小于此值时, 才进行校准. (距离小测距更准)"),
            InputPort<double>("max_drift", 1.0, "校准后的位置与原位置距离应小于此值, 否则认为校准失败"),
            InputPort<bool>("validate", true, "校准后, 用其它的 marker 进行验证, 要求小于 locator 的 max residual"),
        };
    };

private:
    Brain *brain;
};
