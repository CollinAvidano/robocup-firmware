#include <Eigen/Dense>

#include <rc-fshare/robot_model.hpp>

// This isn't LQR anymore. Change name soon.
class LQRController {
public:
    void setTargetVel(Eigen::Vector3f target) { _targetVel = target; }

    void updateGains(float kp, float ki) {
        _kp = kp;
        _ki = ki;
    }

    float getError() {
        return _dbg_error;
    }

    std::array<int16_t, 4> run(const std::array<int16_t, 4>& encoderDeltas, float dt)
    {
        // State estimate
        Eigen::Matrix<double, 4, 1> wheelVels;
        wheelVels << 
            encoderDeltas[0],
            encoderDeltas[1],
            encoderDeltas[2],
            encoderDeltas[3];

        constexpr auto ENC_TICKS_PER_TURN = 2048 * 3;
        wheelVels *= 2.0 * M_PI / ENC_TICKS_PER_TURN / dt;

        auto alpha = 1.0; // high = trust new data more

        _wheelVels = (1 - alpha) * _wheelVels + alpha * wheelVels;

        // Target matrix
        Eigen::Matrix<double, 3, 1> target_mat;
        target_mat << _targetVel[0], _targetVel[1], _targetVel[2];
        Eigen::Matrix<double, 4, 1> targetWheelVels =
            RobotModel::get().BotToWheel * target_mat;

        // Error calculation
        Eigen::Matrix<double, 4, 1> error = targetWheelVels - _wheelVels;

        _dbg_error = error(0,0);

        // Update integrator
        _wheelVelErrors += error * dt;

        // Calculate error torque
        auto kp = _kp;
        auto ki = _ki;
        auto torque_e = kp * error + ki * _wheelVelErrors;

        constexpr auto milli_to_si = 1.0 / 1000;
        constexpr auto km = 25.1 * milli_to_si; // mNm / A -> Nm / A
        constexpr auto R = 0.464; // ohm

        constexpr auto rpm_rad_s = 2.0*M_PI / 60.0; // rpm -> rad/s
        constexpr auto kn = 380 * rpm_rad_s; // rpm / V -> (rad/s) / V
        const auto vcc = RobotModel::get().V_Max;

        std::array<int16_t, 4> control_duties;
        for (std::size_t i = 0; i < control_duties.size(); ++i) {

            double wheel_vel = _wheelVels(i,0);
            printf("wheel_vel: %f\r\n", _wheelVels(i,0));
            // http://wiki.robocup.org/images/e/ef/Small_Size_League_-_RoboCup_2009_-_ETDP_Skuba.pdf
            auto torque_m = (km / R) * vcc - (km / (R * kn)) * wheel_vel;
            auto te = torque_e(i,0);
            printf("te: %f\r\n", te);

            auto control_v = te / torque_m;

            if (abs(control_v) > vcc) control_v = copysign(vcc, control_v);

            // convert voltage to duty cycle
            double dc = control_v * FPGA::MAX_DUTY_CYCLE / vcc;
            control_duties[i] = static_cast<int16_t>(dc);
        }

        printf("Control duties: %d %d %d %d\r\n",
               control_duties[0], control_duties[1],
               control_duties[2], control_duties[3]);

        return control_duties;
    }

private:
    Eigen::Vector3f _targetVel{};
    Eigen::Matrix<double, 4, 1> _wheelVelErrors = {0, 0, 0, 0};
    Eigen::Matrix<double, 4, 1> _wheelVels = {0, 0, 0, 0};

    float _kp;
    float _ki;
    float _dbg_error;
};
