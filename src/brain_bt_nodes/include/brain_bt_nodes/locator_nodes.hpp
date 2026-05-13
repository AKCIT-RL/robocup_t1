#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <string>

// Forward declaration
class Brain;

using namespace std;
using namespace BT;

// Register all locator nodes with the factory
void RegisterLocatorNodes(BT::BehaviorTreeFactory &factory, Brain* brain);

// ==================== Localization Nodes ====================

class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class SelfLocateEnterField : public SyncActionNode
{
public:
    SelfLocateEnterField(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class SelfLocate1M : public SyncActionNode
{
public:
    SelfLocate1M(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class SelfLocate2X : public SyncActionNode
{
public:
    SelfLocate2X(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class SelfLocate2T : public SyncActionNode
{
public:
    SelfLocate2T(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class SelfLocateLT : public SyncActionNode
{
public:
    SelfLocateLT(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class SelfLocatePT : public SyncActionNode
{
public:
    SelfLocatePT(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};

class SelfLocateBorder : public SyncActionNode
{
public:
    SelfLocateBorder(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;
    static PortsList providedPorts();

private:
    Brain *brain;
};
