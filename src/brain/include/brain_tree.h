#pragma once

#include <tuple>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();

    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value;
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    /**
     * 初始化 blackboard 里的 entry，注意新加字段，在这里设置个默认值
     * init blackboard here. stated new fields with default values here.
     */
    void initEntry();
    std::unique_ptr<BT::FileLogger2> file_logger_;
    std::unique_ptr<BT::Groot2Publisher> publisher; 
};


class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("cross_threshold", 0.2, "可进门的角度范围小于这个值时, 则传中")  //se o angulo for menor que esse valor, cruza a bola (chute longo?)
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),                //se a distancia da bola for maior, da chase
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),                    //usado pra ler a ultima decisao
            InputPort<string>("position", "offense", "offense | defense, 决定了向哪个方向踢球"),   //(nao usado na decisao) pra qual direcao chutar a bola? pra frente | pra tras
            OutputPort<string>("decision_out")};                                                //ultima decisao; usado para nao entrar em loop no chase
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir; 
    rclcpp::Time timeLastTick; 
};


class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),                            //se a distancia for maior, da chase
            InputPort<double>("adjust_angle_tolerance", 0.1, "小于这个角度, 认为 adjust 已经成功"),             //se a diferenca pro angulo for desejado menor, nao precisa girar 
            InputPort<double>("adjust_y_tolerance", 0.1, "y 方向偏移小于这个值, 认为 y 方向 adjust 成功"),       //se a diferenca pra posicao y for menor, nao precisa andar
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),                                //usado pra ler a ultima decisao
            InputPort<double>("auto_visual_kick_enable_dist_min", 2.0, "自动视觉踢球启用时球的最小距离"),        //(nao usado na decisao)
            InputPort<double>("auto_visual_kick_enable_dist_max", 3.0, "自动视觉踢球启用时球的最大距离"),        //(nao usado na decisao)
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "自动视觉踢球启用时球的角度范围"),         //(nao usado na decisao)
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 3.0, "自动视觉踢球障碍物距离阈值，如果该距离内有障碍物，则不执行自动视觉踢球"), //(nao usado na decisao)
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "自动视觉踢球障碍物在前方角度范围内的阈值，如果该角度内有障碍物，则不执行自动视觉踢球"), //(nao usado na decisao)
            OutputPort<string>("decision_out"),                                                             //ultima decisao; usado para nao entrar em loop no chase
        };              //o que e um auto_visual? nao existe em lugar algum do codigo
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }
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
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "转向的速度上限"), //max vtheta 
        };
    }

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
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 300, "在同一个位置停留多少毫秒"),   //quanto tempo fica na mesma posicao de camera durante a localizacao
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _cmdSequence[7][2] = {
        {0.45, 1.1},
        {0.45, 0.0},
        {0.45, -1.1},
        {1.0, -1.1},
        {1.0, 0.0},
        {1.0, 1.1},
        {0.45, 0.0},
    };    
    rclcpp::Time _timeLastCmd;    
    int _cmdIndex = 0;               
    Brain *brain;
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("rad", 0, "转多少弧度, 向左为正"),        //quantos radianos girar, anticlockwise é positivo
            InputPort<bool>("towards_ball", false, "为 true 时, 不考虑 rad 的正负号, 而是转向上一次看到不球的方向.")    //se verdadeiro, ignora o sinal do rad e gira na direcao da ultima posicao conhecida da bola
        };
    }

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
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.6, "追球的最大 x 速度"),                        //max vx chase
            InputPort<double>("vy_limit", 0.4, "追球的最大 y 速度"),                        //max vy chase
            InputPort<double>("vtheta_limit", 1.0, "追球时, 实时调整方向的速度不大于这个值"),   //max vtheta chase
            InputPort<double>("dist", 0.1, "追球的目标是球后面多少距离"),                     //chase dist
            InputPort<double>("safe_dist", 4.0, "circle back 时, 保持的安全距离"),          //ao contornar, mantem uma distancia segura
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     
    double _dir = 1.0; 
};

// Chasing the ball: If the ball is behind the robot, it will move around to the back of the ball.
class Positioning : public SyncActionNode
{
public:
    Positioning(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class Adjust : public StatefulActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 3.25, "球的角度大于这个值, 机器人先转身面向球, 直线运动先暂停"), //se o angulo for maior que esse valor, o robô vira para a bola
            InputPort<double>("vx_limit", 0.05, "调整过过程中 vx 的限制 [-limit, limit]"),                 //max vx adjust
            InputPort<double>("vy_limit", 0.05, "调整过过程中 vy 的限制 [-limit, limit]"),                 //max vy adjust
            InputPort<double>("vtheta_limit", 0.1, "调整过过程中 vtheta 的限制 [-limit, limit]"),          //max vtheta adjust
            InputPort<double>("range", 2.25, "ball  range 保持这个值"),                                  //o intervalo de bolas mantem esse valor
            InputPort<double>("vtheta_factor", 3.0, "调整角度时, vtheta 的乘数, 越大转向越快"),             //ao ajustar theta, o fator vtheta é multiplicado por esse valor
            InputPort<double>("tangential_speed_far", 0.2, "调整角度时, 较远时的切线速度"),                 //velocidade tangencial é maior quando a bola está longe
            InputPort<double>("tangential_speed_near", 0.15, "调整角度时, 较近时的切线速度"),               //velocidade tangencial é menor quando a bola está perto
            InputPort<double>("near_threshold", 0.8, "距离目标小于这个值时, 使用 near speed"),             //se a distância até o alvo for menor que esse valor, usa velocidade perto
            InputPort<double>("no_turn_threshold", 0.02, "角度差小于这个值时, 不转身"),                     //se o angulo for menor que esse valor, não gira
            InputPort<double>("turn_first_threshold", 0.8, "角度差大于这个值时, 先转身, 不移动"),           //se o angulo for maior que esse valor, gira primeiro, não se move

        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
};

// 执行踢球动作
class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_msec_kick", 500, "踢球动作最少执行多少毫秒"),    //tempo mínimo que o robô deve ocupar no kick
            InputPort<double>("msecs_stablize", 1000, "稳定多少毫秒"),            //(nao usado na decisao) tempo para estabiilizar apos o chute
            InputPort<double>("speed_limit", 0.8, "速度最大值"),                  //velocidade que o chute deve ser feito; afeta o tempo usado no processamento do chute. diferente de vx vy e vtheta
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
    string _state = "kick"; // stablize | kick
    int _msecKick = 1000;    
    double _speed;
    double _minRange;
    tuple<double, double, double> _calcSpeed();
    // Recalcula (a cada tick) a velocidade linear vx/vy do "kickinho"/"shoot"/"conduzir"
    // a partir da posição atual da bola. Retorna a velocidade escalar (norm(vx, vy)).
    double _calcKickinhoVel(double &vx, double &vy);
};


class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("msecs", 1000, "站立多少毫秒"),  //quanto tempo fica parado
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
};


class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("low_pitch", 0.6, "向下看时的最大 pitch"),    //limite inferior da visao
            InputPort<double>("high_pitch", 0.45, "向上看时的最小 pitch"),  //limite superior da visao
            InputPort<double>("left_yaw", 0.8, "向左看时的最大 yaw"),       //limite esquerdo da visao
            InputPort<double>("right_yaw", -0.8, "向右看时的最小 yaw"),     //limite direito  da visao
            InputPort<int>("msec_cycle", 4000, "多少毫秒转一圈"),           //tempo necessario para um ciclo completo
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "目标 x 坐标, Field 坐标系"),         //destino em x
            InputPort<double>("y", 0, "目标 y 坐标, Field 坐标系"),         //destino em y
            InputPort<double>("theta", 0, "目标最终朝向, Field 坐标系"),    //destino em theta
            InputPort<double>("long_range_threshold", 1.5, "目标点的距离超过这个值时, 优先走过去, 而不是细调位置和方向"),   //se a distancia ate o alvo for maior, escolhe ir ate la ao inves de se ajustar antes
            InputPort<double>("turn_threshold", 0.4, "长距离时, 目标点的方向超这个数值时, 先转向目标点"),                 //se o angulo ate o alvo for maior, gira ate o alvo primeiro para depois andar
            InputPort<double>("vx_limit", 0.8, "x 限速"),               //limite vx
            InputPort<double>("vy_limit", 0.5, "y 限速"),               //limite vy
            InputPort<double>("vtheta_limit", 0.2, "theta 限速"),       //limite vtheta
            InputPort<double>("x_tolerance", 0.5, "x 容差"),            //tolerancia da posicao x
            InputPort<double>("y_tolerance", 0.5, "y 容差"),            //tolerancia da posicao y
            InputPort<double>("theta_tolerance", 0.5, "theta 容差"),    //tolerancia da orientacao theta
            InputPort<bool>("avoid_obstacle", false, "是否避障")        //desvia? sim | nao
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoToReadyPosition : public SyncActionNode
{
public:
    GoToReadyPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "x tolerance"),        //tolerancia em x para o posicionamento em READY (talvez seja necessario criar o y)
            InputPort<double>("theta_tolerance", 0.5, "theta tolerance"),   //tolerancia theta para o posicionamento em READY
            InputPort<double>("vx_limit", 0.8, "vx limit"),                 //max vx
            InputPort<double>("vy_limit", 0.5, "vy limit"),                 //max vy
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),   //tolerancia x em frente ao gol
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),  //tolerancia theta em frente ao gol
            InputPort<double>("vx_limit", 1.2, "x speed limit"),                                            //max vx
            InputPort<double>("vy_limit", 0.6, "y speed limit"),                                            //max vy
            InputPort<double>("dist_to_goalline", 2.5, "机器人站在门前多少距离"),   //distancia em x da linha do proprio gol, serve como limite para o robo nao ir muito pra tras
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class Assist : public SyncActionNode
{
public:
    Assist(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),   //tolerancia x em frente ao gol
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),  //tolerancia theta em frente ao gol
            InputPort<double>("vx_limit", 0.1, "x speed limit"),                                            //max vx
            InputPort<double>("vy_limit", 0.1, "y speed limit"),                                            //max vy
            InputPort<double>("dist_to_goalline", 2.5, "机器人站在门前多少距离"),   //distancia em x da linha do proprio gol, serve como limite para o robo nao ir muito pra tras
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


/**
 * @brief 设置机器人的速度  explicacao de x y e theta (relativos) abaixo; x e y sao em m/s e theta em rad/s
 *
 * @param x,y,theta double, 机器人在 x，y 方向上的速度（m/s）和逆时针转动的角速度（rad/s), 默认值为 0. 全为 0 时，即相当于给出站立不动指令
 *
 */
class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),            //velocidade default x
            InputPort<double>("y", 0, "Default y is 0"),            //velocidade default y
            InputPort<double>("theta", 0, "Default  theta is 0"),   //theta default
        };
    }

private:
    Brain *brain;
};

// 原地踏步         //parado
class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {};
    }

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("action", "start", "start | stop"),
        };
    }

private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("pitch", 0, "target head pitch"),     //cabeca pra baixo/cima
            InputPort<double>("yaw", 0, "target head yaw"),         //cabeca pra esquerda/direita   
        };
    }

private:
    Brain *brain;
};


class CheckAndStandUp : public SyncActionNode
{
public:
CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("side", "attack", "attack | defense"),                    //free kick e de quem? atacaos ou defendemos?
            InputPort<double>("attack_dist", 0.7, "attack side target dist to ball"),   //se for atacar fica mais perto da bola
            InputPort<double>("defense_dist", 1.9, "defense side target dist to ball"), //se for defender fica mais longe da bola
            InputPort<double>("vx_limit", 1.2, "vx limit"),                             //max vx
            InputPort<double>("vy_limit", 0.5, "vy limit"),                             //max vy

        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    bool _isInFinalAdjust = false; 
};

// 回到场地内           //volta pro campo
class GoBackInField : public SyncActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("valve", 0.5, "回到场内距离边界多远可以停止"),        //distancia que reduz o tamanho efetivo do campo nos 4 lados; se tiver fora, vira pra dentro do campo e entra
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "在距离球多远的距离, 就不再走向球了"),                                  //distancia maxima em relacao a bola
            InputPort<double>("stop_angle", 0.1, "球的角度在多少时, 就不再转向球了"),                                  //(implementado, mas desativado (comentado)) se o angulo da bola em relacao ao robo for menor, para de girar
            InputPort<double>("vy_limit", 0.2, "限制 Y 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 0.4"),    //limite de vy
            InputPort<double>("vx_limit", 0.6, "限制 X 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 1.2"),    //limite de vx
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "x"),         //calibrando x, nao mexe
            InputPort<double>("y", 0, "y"),         //calibrando y, nao mexe
            InputPort<double>("theta", 0, "theta"), //calibrando theta, nao mexe
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


// 向 cout 打印文字     //imprime texto pra cout
class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("msg")};
    }

private:
    Brain *brain;
};

// 播放一些预定义的声音     //toca sons predefinidos
class PlaySound : public SyncActionNode
{
public:
    PlaySound(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("sound", "cheerful", "播放声音的名称"),           //nome do som
            InputPort<bool>("allow_repeat", false, "是否允许重复播放同一个声音"), //permitir repeticao do mesmo som
        };
    }

private:
    Brain *brain;
};

// 使用本地 tts (espeak) 朗读文本 usa tts local (espeak) para ler o texto
class Speak : public SyncActionNode
{
public:
    Speak(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("text", "", "朗读的文本内容, 必须是英文"),    //o texto sera falado em voz alta; deve estar em ingles
        };
    }

private:
    Brain *brain;
};
