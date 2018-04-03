#include <Eigen/Dense>
#include "Geometry2d/Util.hpp"
#include "RobotModel.hpp"

// This is an "offline" LQR controller because the calculations for finding
// the gain matrix, K, are done offline (at compile time) and stored in a
// lookup table.  Calculating K on the MBED would be very slow...
class LQRController {
   public:
    void setTargetVel(Eigen::Vector3f target) { _targetVel = target; }
    /// Weighting matrices for the LQR cost function.
    //
    // LQR gain matrix.
    // typedef Eigen::Matrix<float, 4, 3> KType;


    /// u = -K*(currVel-cmdVel) - pinv(B)*A*cmdVel
    std::array<int16_t, 4> run(const std::array<int16_t, 4>& encoderDeltas, float dt)
    {

        // get wheel velocities from encoder readings
        // we should probably build a proper estimator rather than taking the
        // lastest measurement as truth, but this will do for now
        Eigen::Matrix<double, 4, 1> wheelVels;
        wheelVels << encoderDeltas[0], encoderDeltas[1], encoderDeltas[2],
            encoderDeltas[3];
        wheelVels *= 2.0 * M_PI / ENC_TICKS_PER_TURN / dt;

        Eigen::Matrix<double, 3, 1> target_mat;
        target_mat << _targetVel[0], _targetVel[1], _targetVel[2];
        Eigen::Matrix<double, 4, 1> targetWheelVels =
            RobotModelControl.BotToWheel * target_mat;

        // auto steadyStateTerm = -(RobotModelControl.PinvB * RobotModelControl.A * targetWheelVels);
        // auto correctionTerm = -RobotModelControl.K * (wheelVels - targetWheelVels);
        // Eigen::Matrix<double, 4, 1> controlValues =  correctionTerm + steadyStateTerm;

        auto error = wheelVels - targetWheelVels;
        _wheelVelErrors += error * dt;

        // Eigen::Matrix combined_state;
        Eigen::Matrix<double, 8, 1> combined_state;
        combined_state << wheelVels,
                          _wheelVelErrors;

        // printf("Combined state made\r\n");
        Eigen::Matrix<double, 4, 1> control_v = -RobotModelControl.K * combined_state;
        // printf("Control v calculated\r\n");

        // control_v += controlValues;
        // printf("Control_v: %f\r\n", control_v(1,1));
        std::array<int16_t, 4> control_duties;
        // limit output voltages to V_max
        for (std::size_t i = 0; i < control_duties.size(); ++i) {
            control_duties[i] = control_v(i,1) * FPGA::MAX_DUTY_CYCLE / RobotModelControl.V_Max;
        }
        printf("Duties: %d %d %d %d\r\n", control_duties[0], control_duties[1], control_duties[2], control_duties[3]);

        return control_duties;
    }

   private:
    Eigen::Vector3f _targetVel{};
    Eigen::Matrix<double, 4, 1> _wheelVelErrors = {0, 0, 0, 0};

    // Eigen::Matrix<double, 4, 1> control_v = {0, 0, 0, 0};

    static const uint16_t ENC_TICKS_PER_TURN = 2048 * 3;
};
