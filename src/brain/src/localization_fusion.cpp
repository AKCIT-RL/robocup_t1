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
    auto data = brain_->data;
    cv::Mat img = field_image_.clone();

    drawRobot(img);
    drawBall(img);
    drawMarkings(img);
    drawOpponents(img);

    // Robot position
    cv::String textRobot = cv::format("Robot: (%.2f, %.2f) m", data->robotPoseToField.x, data->robotPoseToField.y);
    cv::putText(img, textRobot, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    // Ball position
    cv::String textBall;
    if (data->ball.confidence > 0.1) {
        textBall = cv::format("Ball: (%.2f, %.2f) m", data->ball.posToField.x, data->ball.posToField.y);
    } else {
        textBall = "Ball: Not Visible";
    }
    cv::putText(img, textBall, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // Opponents (Orange)
    cv::circle(img, cv::Point(15, 60), 6, cv::Scalar(0, 165, 255), -1); // BGR for Orange
    cv::putText(img, "Opponent", cv::Point(30, 65), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // True Map Markers (Gray)
    cv::circle(img, cv::Point(15, 80), 6, cv::Scalar(180, 180, 180), -1);
    cv::putText(img, "True Map Marker", cv::Point(30, 85), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    // Perceived Markers (Magenta)
    cv::circle(img, cv::Point(15, 100), 6, cv::Scalar(255, 0, 255), -1);
    cv::putText(img, "Perceived Marker", cv::Point(30, 105), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

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
    int width = (fd.length + 5.0) * pixels_per_meter_; // Adding margin each side
    int height = (fd.width + 5.0) * pixels_per_meter_;
    
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
    
    // Draw Ground Truth Map Markers
    cv::Scalar mapMarkerColor(180, 180, 180); // Light Gray
    for (const auto& mm : brain_->config->mapMarkings) {
        cv::Point2f pt = fieldToImg(mm.x, mm.y);
        cv::circle(field_image_, pt, 5, mapMarkerColor, -1);
        cv::putText(field_image_, mm.type, pt + cv::Point2f(6, -6), cv::FONT_HERSHEY_SIMPLEX, 0.4, mapMarkerColor, 1);
    }

    // Draw Assist Zone
    // Zones: X = -L/4, 0 (already drawn as center line), +L/4
    cv::Scalar zoneLineColor(60, 80, 60); // dark subtle green-gray
    auto drawDashedLine = [&](cv::Point2f from, cv::Point2f to, int dashLen = 8, int gapLen = 6) {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double totalLen = sqrt(dx * dx + dy * dy);
        double ux = dx / totalLen;
        double uy = dy / totalLen;
        double drawn = 0;
        while (drawn < totalLen) {
            double segEnd = std::min(drawn + dashLen, totalLen);
            cv::Point2f p1(from.x + ux * drawn, from.y + uy * drawn);
            cv::Point2f p2(from.x + ux * segEnd, from.y + uy * segEnd);
            cv::line(field_image_, p1, p2, zoneLineColor, 1);
            drawn = segEnd + gapLen;
        }
    };
    // X = -L/4
    drawDashedLine(fieldToImg(-halfLen / 2.0, halfWid), fieldToImg(-halfLen / 2.0, -halfWid));
    // X = +L/4
    drawDashedLine(fieldToImg(halfLen / 2.0, halfWid), fieldToImg(halfLen / 2.0, -halfWid));
}

void LocalizationFusion::drawRobot(cv::Mat& img) {
    auto data = brain_->data;
    auto pose = data->robotPoseToField;
    cv::Point2f robotPos = fieldToImg(pose.x, pose.y);

    // Draw Robot Body
    cv::Scalar robotColor = cv::Scalar(255, 0, 0); // Blue
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
        // Draw the ball based on robot's position
        double ballX = data->robotPoseToField.x + data->ball.posToRobot.x * cos(data->robotPoseToField.theta) - data->ball.posToRobot.y * sin(data->robotPoseToField.theta);
        double ballY = data->robotPoseToField.y + data->ball.posToRobot.x * sin(data->robotPoseToField.theta) + data->ball.posToRobot.y * cos(data->robotPoseToField.theta);
        cv::Point2f ballPos = fieldToImg(ballX, ballY);
        
        cv::circle(img, ballPos, 8, cv::Scalar(0, 0, 255), -1); // Red Ball
        cv::putText(img, "Ball", ballPos + cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}

void LocalizationFusion::drawOpponents(cv::Mat& img) {
    auto data = brain_->data;
    
    for (const auto& r : data->getRobots()) {

        double rx = data->robotPoseToField.x + r.posToRobot.x * cos(data->robotPoseToField.theta) - r.posToRobot.y * sin(data->robotPoseToField.theta);
        double ry = data->robotPoseToField.y + r.posToRobot.x * sin(data->robotPoseToField.theta) + r.posToRobot.y * cos(data->robotPoseToField.theta);
        cv::Point2f pt = fieldToImg(rx, ry);
        
        cv::circle(img, pt, 8, cv::Scalar(0, 165, 255), -1); // Orange
        cv::putText(img, r.label, pt + cv::Point2f(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 165, 255), 1);
        
        // Print coord text
        cv::String textCoord = cv::format("(%.1f, %.1f)", r.posToRobot.x, r.posToRobot.y);
        cv::putText(img, textCoord, pt + cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 165, 255), 1);
    }
}

void LocalizationFusion::drawMarkings(cv::Mat& img) {
    auto data = brain_->data;
    
    for (const auto& m : data->getMarkings()) {

        double mx = data->robotPoseToField.x + m.posToRobot.x * cos(data->robotPoseToField.theta) - m.posToRobot.y * sin(data->robotPoseToField.theta);
        double my = data->robotPoseToField.y + m.posToRobot.x * sin(data->robotPoseToField.theta) + m.posToRobot.y * cos(data->robotPoseToField.theta);
        cv::Point2f pt = fieldToImg(mx, my);
        
        cv::circle(img, pt, 6, cv::Scalar(255, 0, 255), -1); // Magenta
        cv::putText(img, m.label, pt + cv::Point2f(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 255), 1);
        
        // Print coord text below it
        cv::String textCoord = cv::format("(%.1f, %.1f)", m.posToRobot.x, m.posToRobot.y);
        cv::putText(img, textCoord, pt + cv::Point2f(8, 8), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 255), 1);
    }
}
