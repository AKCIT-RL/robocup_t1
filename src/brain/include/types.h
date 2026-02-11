/**
 * @file type.h
 * @brief 定义 brain 项目中用到的 struct 及 enum
 */
#pragma once

#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

using namespace std;

/* ------------------ Struct ------------------------*/

// 球场尺寸信息, 所有平行于底线的是长度, 所有垂直于底线的是宽度, 与线的长度无关
struct FieldDimensions
{
    double length;            // 球场长度
    double width;             // 球场宽度
    double penaltyDist;       // 罚球点距离底线的直线距离
    double goalWidth;         // 球门的宽度
    double circleRadius;      // 中圈的半径
    double penaltyAreaLength; // 禁区的长
    double penaltyAreaWidth;  // 禁区的宽
    double goalAreaLength;    // 球门区的长
    double goalAreaWidth;     // 球门区的宽
                              // 注意: 禁区比球门区大；禁区和球门区的长与宽实际上要小。这个命名是为了与比赛规则相统一。
};
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};
const FieldDimensions FD_ROBOLEAGUE{22, 14, 3.5, 2.5, 2.0, 5.0, 8.0, 2.0, 5.0};
const FieldDimensions FD_CUSTOMSIZE{5.87, 4.63, 1.37, 2.77, 0.50, 1.99, 3.86, 0.7, 3.09};
const FieldDimensions FD_CAMPUS_PA{4.80, 4.10, 1.61, 2.70, 0.40, 1.20, 3.30, 0.6, 2.70};


// Pose2D, 记录平面上的一个点以及朝向
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // rad, 从 x 轴正方向开始, 逆时针为正
};

// 定位范围约束条件
struct PoseBox2D
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double thetamin;
	double thetamax;
};

// 球场标志点
struct FieldMarker
{
	char type;		   // L|T|X|P, 代表不同的标志点类型，其中 P 代表 penalty mark
	double x, y;	   // 标志点的位置 (m)
	double confidence; // 识别的信心度
};

// 定位结果
struct LocateResult
{
	bool success = false;
	int code = -1;		 // int, 0: 成功； 1: 生成新粒子失败(数量为 0); 2: 收敛后的残差不合理; 3: 未收敛; 4: Marker 数量不够; 5: 所有粒子的概率都过低; -1 初始态
	double residual = 0; // 平均残差
	Pose2D pose;
	int msecs = 0; // 定位耗时
};

// Point, 记录一个三维点
struct Point
{
    double x;
    double y;
    double z;
};

// Point2D, 记录一个二维点
struct Point2D
{
    double x;
    double y;

    Point2D operator+(const Point2D &other) const {
        return {x + other.x, y + other.y};
    }

    Point2D operator-(const Point2D &other) const {
        return {x - other.x, y - other.y};
    }
    
    Point2D operator/(const double &value)
    {
        x /= value;
        y /= value;
        return {x, y};
    }

    Point2D operator*(const double &value)
    {
        x *= value;
        y *= value;
        return {x, y};
    }

    friend Point2D operator*(double value, const Point2D &p) {
        return {p.x * value, p.y * value};
    }

    Point2D rotateAround(const Point2D &value, double angle) const
    {
        Point2D aux;
        aux.x = x;
        aux.y = y;

        aux = aux - value;

        // rotate
        angle = angle * M_PI / 180.0;
        aux.x = aux.x * cos(angle) - aux.y * sin(angle);
        aux.y = aux.x * sin(angle) + aux.y * cos(angle);

        return aux + value;
    }

    double norm() const
    {
        return sqrt(x*x + y*y);
    }

    Point2D normalized() const
    {
        Point2D aux{x, y};
        return aux / sqrt(x*x + y*y);
    }

    double distanceToPoint(const Point2D &value) const 
    {
        return sqrt((x - value.x) * (x - value.x) +
                    (y - value.y) * (y - value.y));
    }

    bool interceptLine(const Point2D &p1, const Point2D &p2, double threshold) const
    {
        Point2D v, u, w, a, p3;
        p3.x = x;
        p3.y = y;

        v = p2 - p1;
        u = p3 - p1;
        w = u.perpendVector(v);

        double distance = w.norm();

        if (distance <= threshold)
        {
            Point2D vt2dP, Pa, Pb, Pc;
            float fP_PaPb = 0, fPaPb_PaPb = 0, fP_PcPb = 0, fPcPb_PcPb = 0;

            a = w;
            a = a.normalized(); // A^ = W→/|W→|
            a = a * threshold;

            Pa = a + p1;
            Pb = p2 + a;
            Pc = p2 - a;
            vt2dP = p3 - Pb;

            fP_PaPb = vt2dP.dotProduct(Pa - Pb);
            fPaPb_PaPb = (Pa - Pb).dotProduct(Pa - Pb);
            fP_PcPb = vt2dP.dotProduct(Pc - Pb);
            fPcPb_PcPb = (Pc - Pb).dotProduct(Pc - Pb);

            if (fP_PaPb >= 0 &&
                fP_PaPb <= fPaPb_PaPb && // Se 0 < P→ . (Pa-Pb)→ < |Pa-Pb|² E 0 < P→
                                        // . (Pc-Pb)→ < |Pc-Pb|² o ponto P3 está
                fP_PcPb >= 0 && fP_PcPb <= fPcPb_PcPb) // dentro do retângulo
                return true;
        }
        return false;
    }

    Point2D perpendVector(const Point2D &v) const
    {
        Point2D u;
        u.x = x;
        u.y = y;

        return u - ((u.dotProduct(v) / v.normSquared()) * v); // W→ = u→ - ( ( (u→ . v→)/ |v→|²) * v→)
    }

    double dotProduct(const Point2D &other) const
    {
        return x*other.x + y*other.y;
    }

    double normSquared() const
    {
        return x*x + y*y;
    }

    bool pointInsideCircle(double radius, const Point2D &center,
                           double degreesRange, double orientation,
                           vector<Point2D> &anglesOut) const
    {
        const double angleResolution = 10.0, interceptTol = 0.3;
        Point2D orientPt{radius * cos(orientation), radius * sin(orientation)},
                thisPt{x, y}, testPt;
        orientPt = orientPt + center;

        vector<Point2D> detectedAngles(static_cast<int>(degreesRange / angleResolution + 1),
                                       {0, 0});

        bool isInside = false;

        int n = 0;
        for (double angle = -degreesRange / 2.0; angle <= degreesRange / 2.0;
                angle+= angleResolution)
        {
            testPt = orientPt.rotateAround(center, angle);

            if (thisPt.interceptLine(testPt, center, interceptTol)) {
                isInside = true;
                detectedAngles.at(n) = testPt;
            }
            n++;
        }

        anglesOut = detectedAngles;

        return isInside;
    }

    bool isInsideKickinho(double fieldLength, double kickRadius)
    {
        Point2D goalPoint{fieldLength / 2.0, 0};

        if (x > 0 && goalPoint.distanceToPoint({x, y}) > kickRadius)
        {
            return true;
        }
        return false;
    }

    bool isInsideKick(double fieldLength, double kickRadius, bool isAngleGood)
    {
        Point2D goalPoint{fieldLength / 2.0, 0}, testPoint{x, y};

        if (x > 0 && 
            goalPoint.distanceToPoint(testPoint) < kickRadius && isAngleGood)
        {
            return true;
        }
        return false;
    }
};

// BoundingBox
struct BoundingBox
{
    double xmin, xmax, ymin, ymax;
};

// GameObject, 用于存储比赛中的重要实体信息，如 Ball, Goalpost 等。相比于 /detect 消息中的 detection::DetectedObject，它的信息更为丰富。
struct GameObject
{
    string label;                
    string color;                
    BoundingBox boundingBox;     
    Point2D precisePixelPoint;   
    double confidence;           
    Point posToRobot;            

    Point posToField;                                 
    double range;                                    
    double pitchToRobot, yawToRobot;                  
    rclcpp::Time timePoint;                           

    int id;    
    string name;  
    double idConfidence; 
    string info; 
};

// Line
struct Line {
    double x0, y0, x1, y1;
};

enum class LineHalf {
    NA,
    Self,
    Opponent
};

enum class LineDir {
    NA,
    Horizontal,
    Vertical
};

enum class LineSide {
    NA,
    Left,
    Right
};

enum class LineType {
    NA,
    TouchLine,
    MiddleLine,
    GoalLine,
    GoalArea,
    PenaltyArea
};

struct FieldLine {
    Line posToField;
    Line posToRobot;
    Line posOnCam;
    rclcpp::Time timePoint;                           
    LineDir dir;                                       
    LineHalf half;                                      
    LineSide side;                                      
    LineType type;
    double confidence;                                      
};

struct MapMarking {
    double x;
    double y;
    string type; // TCross | LCross | XCross | PenaltyPoint
    string name; 
};

// VisionBox 用于视野的范围
struct VisionBox {
    vector<double> posToField;
    vector<double> posToRobot;
    rclcpp::Time timePoint;                           // 物体被检测到的时间
};

struct TimestampedData {
    vector<double> data;
    rclcpp::Time timePoint;
};

// 起身
struct RobotRecoveryStateData {
    uint8_t state; 
    uint8_t is_recovery_available; 
    uint8_t current_planner_index;
};

// 用于存储队友间通讯
struct TMStatus {
    string role = "not initialized"; // triker, goal_keeper
    bool isAlive = false; 
    bool ballDetected = false;
    bool ballLocationKnown = false;
    double ballConfidence = 0.;
    double ballRange = 0.;
    double cost = 0.; 
    bool isLead = true; 
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir = 0.; 
    double thetaRb = 0.; 
    int cmd = 0; 
    int cmdId = 0;
    rclcpp::Time timeLastCom; 
};


enum class RobotRecoveryState {
    IS_READY = 0,
    IS_FALLING = 1,
    HAS_FALLEN = 2,
    IS_GETTING_UP = 3
};
