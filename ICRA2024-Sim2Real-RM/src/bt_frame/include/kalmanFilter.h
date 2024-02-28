#pragma once
#include <Eigen/Dense>
class KalmanFilter2D {
public:
    KalmanFilter2D() {}

    KalmanFilter2D(const Eigen::Vector2d& initial_state, double process_variance, double measurement_variance)
        : state_(initial_state), process_variance_(process_variance), measurement_variance_(measurement_variance) {
        state_covariance_ = Eigen::Matrix2d::Identity();
    }

    void Predict() {
        state_prediction_ = state_;
        state_covariance_prediction_ = state_covariance_ + process_variance_ * Eigen::Matrix2d::Identity();
    }

    void Update(const Eigen::Vector2d& measurement) {
        kalman_gain_ = state_covariance_prediction_ * (state_covariance_prediction_ + measurement_variance_ * Eigen::Matrix2d::Identity()).inverse();
        state_ = state_prediction_ + kalman_gain_ * (measurement - state_prediction_);
        state_covariance_ = (Eigen::Matrix2d::Identity() - kalman_gain_) * state_covariance_prediction_;
    }

    Eigen::Vector2d GetState() const {
        return state_;
    }

private:
    Eigen::Vector2d state_;
    double process_variance_;
    double measurement_variance_;

    Eigen::Matrix2d state_covariance_;
    Eigen::Vector2d state_prediction_;
    Eigen::Matrix2d state_covariance_prediction_;
    Eigen::Matrix2d kalman_gain_;
};
