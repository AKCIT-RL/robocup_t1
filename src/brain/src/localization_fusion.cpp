#include "brain.h"
#include "localization_fusion.h"
#include "utils/math.h"
#include <algorithm>

LocalizationFusion::LocalizationFusion(Brain* brain) : brain_(brain) {
    // Initialize field image once
    createFieldImage();
}

void LocalizationFusion::tick() {
    if (!brain_->config->visualizationLocalEnable) {
        return;
    }

    if (field_image_.empty()) {
        createFieldImage();
    }

    cv::Mat img = field_image_.clone();
    drawRobot(img);
    drawBall(img);

    cv::imshow("Localization Fusion", img);
    cv::waitKey(1);
}

cv::Point2f LocalizationFusion::fieldToImg(double x, double y) {
    // Field coordinate system: X+ forward, Y+ left
    // Image coordinate system: X+ right, Y+ down
    // Rotate 90 degrees clockwise visually? 
    // Usually: Field X -> Image X (Right), Field Y -> Image Y (Up is negative, Down is positive)
    // Let's assume standard top-down view:
    // Field X is horizontal on screen (left-right)? Or vertical?
    // User mentions: "Ponto 0,0 is center. Positive X to enemy goal (Right on screen?). Positive Y to left touchline (Top on screen?)."
    
    // So:
    // Screen X = center.x + field.x * scale
    // Screen Y = center.y - field.y * scale (because image Y grows down)
    
    return cv::Point2f(
        center_offset_.x + x * pixels_per_meter_,
        center_offset_.y - y * pixels_per_meter_
    );
}

void LocalizationFusion::createFieldImage() {
    auto fd = brain_->config->fieldDimensions;
    
    // Calculate canvas size
    int width = (fd.length + 2.0) * pixels_per_meter_; // Add 1m margin each side
    int height = (fd.width + 2.0) * pixels_per_meter_;
    
    // Ensure minimum size
    width = std::max(width, 900);
    height = std::max(height, 600);
    
    center_offset_ = cv::Point2f(width / 2.0, height / 2.0);
    
    field_image_ = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 100, 0)); // Dark Green Background

    // Draw Lines
    cv::Scalar lineColor(255, 255, 255); // White
    int thickness = 2;

    // Outer Border (Touchlines and Goal lines)
    double halfLen = fd.length / 2.0;
    double halfWid = fd.width / 2.0;
    
    cv::Point2f p1 = fieldToImg(halfLen, halfWid);
    cv::Point2f p2 = fieldToImg(-halfLen, halfWid);
    cv::Point2f p3 = fieldToImg(-halfLen, -halfWid);
    cv::Point2f p4 = fieldToImg(halfLen, -halfWid);
    
    cv::line(field_image_, p1, p2, lineColor, thickness);
    cv::line(field_image_, p2, p3, lineColor, thickness);
    cv::line(field_image_, p3, p4, lineColor, thickness);
    cv::line(field_image_, p4, p1, lineColor, thickness);
    
    // Center Line
    cv::Point2f midTop = fieldToImg(0, halfWid);
    cv::Point2f midBot = fieldToImg(0, -halfWid);
    cv::line(field_image_, midTop, midBot, lineColor, thickness);
    
    // Center Circle
    cv::Point2f center = fieldToImg(0, 0);
    int radius = fd.circleRadius * pixels_per_meter_;
    cv::circle(field_image_, center, radius, lineColor, thickness);
    
}

void LocalizationFusion::drawRobot(cv::Mat& img) {
    auto data = brain_->data;
    Pose2D pose = data->robotPoseToField;
    bool isGood = true;

    // Check localization reliability
    double timeSinceLoc = brain_->msecsSince(data->lastSuccessfulLocalizeTime);
    if (timeSinceLoc > 2000.0) {
        isGood = false;
    }

    if (!isGood) {
        pose = clampToField(pose);
    }

    cv::Point2f robotPos = fieldToImg(pose.x, pose.y);

    // Logging Robot Position
    // RCLCPP_INFO(brain_->get_logger(), "Robot Field: (%.2f, %.2f) -> Pixel: (%.0f, %.0f)", 
    //     pose.x, pose.y, robotPos.x, robotPos.y);
    
    // Draw Robot Body
    cv::Scalar robotColor = isGood ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 255); // Blue if good, Yellow if bad
    cv::circle(img, robotPos, 10, robotColor, -1); // Filled circle
    
    // Draw Orientation
    double endX = pose.x + 0.3 * cos(pose.theta);
    double endY = pose.y + 0.3 * sin(pose.theta);
    cv::Point2f arrowEnd = fieldToImg(endX, endY);
    cv::arrowedLine(img, robotPos, arrowEnd, cv::Scalar(0, 0, 255), 2);
    
    // Draw Text
    cv::putText(img, "Robot", robotPos + cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
}

void LocalizationFusion::drawBall(cv::Mat& img) {
    auto data = brain_->data;
    
    if (data->ball.confidence > 0.1) {
        cv::Point2f ballPos = fieldToImg(data->ball.posToField.x, data->ball.posToField.y);
        
        // Logging Ball Position
        // RCLCPP_INFO(brain_->get_logger(), "Ball Field: (%.2f, %.2f) -> Pixel: (%.0f, %.0f)", 
        //     data->ball.posToField.x, data->ball.posToField.y, ballPos.x, ballPos.y);

        cv::circle(img, ballPos, 8, cv::Scalar(0, 0, 255), -1); // Red Ball
        cv::putText(img, "Ball", ballPos + cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}

Pose2D LocalizationFusion::clampToField(const Pose2D& pose) {
    auto fd = brain_->config->fieldDimensions;
    double x = pose.x;
    double y = pose.y;
    double halfLen = fd.length / 2.0;
    double halfWid = fd.width / 2.0;
    
    double clampedX = std::max(-halfLen, std::min(halfLen, x));
    double clampedY = std::max(-halfWid, std::min(halfWid, y));
    
    // Project to nearest border
    double distLeft = fabs(x - (-halfLen));
    double distRight = fabs(x - halfLen);
    double distBottom = fabs(y - (-halfWid));
    double distTop = fabs(y - halfWid);
    
    double minDist = std::min({distLeft, distRight, distBottom, distTop});
    
    if (minDist == distLeft) clampedX = -halfLen;
    else if (minDist == distRight) clampedX = halfLen;
    else if (minDist == distBottom) clampedY = -halfWid;
    else if (minDist == distTop) clampedY = halfWid;

    return {clampedX, clampedY, pose.theta};
}
