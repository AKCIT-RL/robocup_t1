#pragma once

#include <opencv2/opencv.hpp>
#include "brain_config.h"
#include "brain_data.h"
#include "brain_log.h"
#include "types.h"
#include <memory>
#include <vector>

class Brain;

class LocalizationFusion {
public:
    explicit LocalizationFusion(Brain* brain);
    ~LocalizationFusion() = default;

    void tick();

private:
    Brain* brain_;
    cv::Mat field_image_;
    double pixels_per_meter_ = 100.0;
    cv::Point2f center_offset_ = {450, 300}; // Half of 900x600

    void createFieldImage();
    void drawRobot(cv::Mat& img);
    void drawBall(cv::Mat& img);
    
    cv::Point2f fieldToImg(double x, double y);

    // Helper to clamp pose to field borders
    Pose2D clampToField(const Pose2D& pose);
};
