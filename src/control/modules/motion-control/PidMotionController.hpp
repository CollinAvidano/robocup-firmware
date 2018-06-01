#pragma once

#include <array>
#include "Pid.hpp"
#include "FPGA.hpp"
#include "RobotModel.hpp"
#include <stdio.h>
#include "MPU6050.h"
#include "pins-control.hpp"

/**
 * Robot controller that runs a PID loop on each of the four wheels.
 */
class PidMotionController {
public:
    PidMotionController()
    {
        setPidValues(3.0, 0, 0, 50, 0);

        x_vel.kp = vel_p;
        x_vel.ki = vel_i;
        x_vel.kd = vel_d;
        x_vel.setWindup(vel_wind);
        x_vel.derivAlpha = 0;
        y_vel.kp = vel_p;
        y_vel.ki = vel_i;
        y_vel.kd = vel_d;
        y_vel.setWindup(vel_wind);
        y_vel.derivAlpha = 0;
        w_vel.kp = vel_p;
        w_vel.ki = vel_i;
        w_vel.kd = vel_d;
        w_vel.setWindup(vel_wind);
        w_vel.derivAlpha = 0;
    }

    void setPidValues(float p, float i, float d, unsigned int windup,
                      float derivAlpha) {
        for (Pid& ctl : _controllers) {
            ctl.kp = p;
            ctl.ki = i;
            ctl.kd = d;
            ctl.setWindup(windup);
            ctl.derivAlpha = derivAlpha;
        }
    }

    void updatePValues(float p) {
        for (Pid& ctl : _controllers) {
            ctl.kp = p;
        }
    }

    void updateIValues(float i) {
        for (Pid& ctl : _controllers) {
            ctl.ki = i;
        }
    }

    void updateDValues(float d) {
        for (Pid& ctl : _controllers) {
            ctl.kd = d;
        }
    }

    void setTargetVel(Eigen::Vector3f target) { _targetVel = target; }

    /**
     * Return the duty cycle values for the motors to drive at the target
     * velocity.
     *
     * @param encoderDeltas Encoder deltas for the four drive motors
     * @param dt Time in ms since the last call to run()
     *
     * @return Duty cycle values for each of the 4 motors
     */
    std::array<int16_t, 4> run(const std::array<int16_t, 4>& encoderDeltas,
                               float gyro_w_deg_s, float dt,
                               Eigen::Vector4d* errors = nullptr,
                               Eigen::Vector4d* wheelVelsOut = nullptr,
                               Eigen::Vector4d* targetWheelVelsOut = nullptr) {
        // convert encoder ticks to rad/s
        Eigen::Vector4d wheelVels;
        wheelVels << encoderDeltas[0], encoderDeltas[1], encoderDeltas[2],
            encoderDeltas[3];
        wheelVels *= 2.0 * M_PI / ENC_TICKS_PER_TURN / dt;

        // Get current velocity from wheel speeds
        Eigen::Vector3d currentVel = RobotModelControl.WheelToBot * wheelVels;
        Eigen::Vector3d vel_error = _targetVel.cast<double>() - currentVel;

        // PID for each
        double x = x_vel.run(vel_error[0]);
        double y = y_vel.run(vel_error[1]);
        double w = w_vel.run(vel_error[2]);

        Eigen::Vector3d targetBodyVel;
        targetBodyVel[0] = x;
        targetBodyVel[1] = y;
        targetBodyVel[2] = w;

        Eigen::Vector4d targetWheelVels =
            RobotModelControl.BotToWheel * _targetVel.cast<double>();


        for (int i = 0; i < 4; i++) {
            if (abs(targetWheelVels[i]) < 1) {
                targetWheelVels[i] = 0;
            }
        }


        if (targetWheelVelsOut) {
            *targetWheelVelsOut = targetWheelVels;
        }

        Eigen::Vector4d wheelVelErr = targetWheelVels - wheelVels;

        if (errors) {
            *errors = wheelVelErr;
        }

        if (wheelVelsOut) {
            *wheelVelsOut = wheelVels;
        }

        // units: degs/second
        float encoder_w_deg_s = currentVel[2] * 180.0f / M_PI;
        bool close = abs(encoder_w_deg_s - gyro_w_deg_s) < 10;

        // if (count++ > 20) {
            // count = 0;
            // std::printf("%d\t%d\r\n",
                   // static_cast<int>(encoder_w_deg_s),
                   // static_cast<int>(gyro_w_deg_s));
        // }

        // Calculated by checking for slippage at max accel, and decreasing appropriately
        // Binary search works really well in this case
        // Caution: This is dependent on the PID values so increasing the agressiveness of that will change this
        double max_error = 3.134765625;
        double scale = 1;

        for (int i = 0; i < 4; i++) {
            if (abs(wheelVelErr[i]) > max_error) {
                scale = max(scale, abs(wheelVelErr[i]) / max_error);
            }
        }

        wheelVelErr /= scale;
        targetWheelVels = wheelVels + wheelVelErr;

        std::array<int16_t, 4> dutyCycles;
        for (int i = 0; i < 4; i++) {
            float dc =
                targetWheelVels[i] * RobotModelControl.DutyCycleMultiplier +
                copysign(4, targetWheelVels[i]);

            dc += _controllers[i].run(wheelVelErr[i]);

            if (std::abs(dc) > FPGA::MAX_DUTY_CYCLE) {
                // Limit to max duty cycle
                dc = copysign(FPGA::MAX_DUTY_CYCLE, dc);
                // Conditional integration indicating open loop control
                _controllers[i].set_saturated(true);
            } else {
                _controllers[i].set_saturated(false);
            }

            dutyCycles[i] = static_cast<int16_t>(dc);
        }

        return dutyCycles;
    }

    // 2048 ticks per turn. Theres is a 3:1 gear ratio between the motor and the
    // wheel.
    static const uint16_t ENC_TICKS_PER_TURN = 2048 * 3;

private:
    /// controllers for each wheel
    std::array<Pid, 4> _controllers{};

    int count = 0;

    double vel_p = 1;
    double vel_i = 0;
    double vel_d = 0;
    double vel_wind = 0;
    Pid x_vel;
    Pid y_vel;
    Pid w_vel;

    Eigen::Vector3f _targetVel{};
};
