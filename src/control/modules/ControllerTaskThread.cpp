#include <cmath>

#include <configuration/ConfigStore.hpp>
#include "Assert.hpp"
#include "Console.hpp"
#include "ConfigStore.hpp"
#include "FPGA.hpp"
#include "Logger.hpp"
#include "PidMotionController.hpp"
#include "RPCVariable.h"
#include "RobotDevices.hpp"
#include "Rtos.hpp"
#include "RtosTimerHelper.hpp"
#include "TaskSignals.hpp"
#include "io-expander.hpp"
#include "motors.hpp"
#include "MPU6050_6Axis_MotionApps20.h"
#include "stall/stall.hpp"
using namespace std;

// Keep this pretty high for now. Ideally, drop it down to ~3 for production
// builds. Hopefully that'll be possible without the console
constexpr auto CONTROL_LOOP_WAIT_MS = 5;

// initialize PID controller
PidMotionController pidController;

/** If this amount of time (in ms) elapses without
 * Task_Controller_UpdateTarget() being called, the target velocity is reset to
 * zero.  This is a safety feature to prevent robots from doing unwanted things
 * when they lose radio communication.
 */
constexpr uint32_t COMMAND_TIMEOUT_INTERVAL = 250;

std::unique_ptr<RtosTimerHelper> commandTimeoutTimer = nullptr;
std::array<WheelStallDetection, 4> wheelStallDetection{};
bool commandTimedOut = true;

std::array<int16_t, 4> enc_counts = {0, 0, 0, 0};

void Task_Controller_UpdateTarget(Eigen::Vector3f targetVel) {
    pidController.setTargetVel(targetVel);

    // reset timeout
    commandTimedOut = false;
    if (commandTimeoutTimer)
        commandTimeoutTimer->start(COMMAND_TIMEOUT_INTERVAL);
}

uint8_t dribblerSpeed = 0;
void Task_Controller_UpdateDribbler(uint8_t dribbler) {
    dribblerSpeed = dribbler;
}

std::array<int16_t, 4> Task_Controller_EncGetClear() {
    // copy
    std::array<int16_t, 4> ret_enc(enc_counts);
    // perform reset
    for (auto& e : enc_counts) {
        e = 0;
    }
    return ret_enc;
}

/**
 * initializes the motion controller thread
 */
void Task_Controller(const void* args) {
    const auto mainID =
        reinterpret_cast<const osThreadId>(const_cast<void*>(args));

    // Store the thread's ID
    const auto threadID = Thread::gettid();
    ASSERT(threadID != nullptr);

    // Store our priority so we know what to reset it to after running a command
    const auto threadPriority = osThreadGetPriority(threadID);
    (void)threadPriority;  // disable warning if unused

#if 1 /* enable whenever the imu is actually used */

    int16_t ax = 0, ay = 0, az = 0,
            gx = 0, gy = 0, gz = 0;

    int ax_offset = 0, ay_offset = 0, az_offset = 0,
        gx_offset = 0, gy_offset = 0, gz_offset = 0;

    ax_offset = 534;
    ay_offset = -567;
    az_offset = 6650;
    gx_offset = 86;
    gy_offset = -48;
    gz_offset = -140;

    MPU6050 imu(MPU6050_DEFAULT_ADDRESS, RJ_I2C_SDA, RJ_I2C_SCL);
    imu.initialize();

    // FILE *fp = fopen("/local/OFFSETS.txt", "r");  // Open "out.txt" on the local file system for writing
    int success = -1;

    // printf("open\r\n");
    // if (fp != nullptr) {
        // success = fscanf(fp, "%d %d %d %d %d %d", &ax_offset, &ay_offset, &az_offset,
                                                  // &gx_offset, &gy_offset, &gz_offset);
        // // printf("fscanf done\r\n");
        // fclose(fp);
        // // printf("closed\r\n");
    // }

    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

    imu.setXAccelOffset(ax_offset);
    imu.setYAccelOffset(ay_offset);
    imu.setZAccelOffset(az_offset);
    imu.setXGyroOffset(gx_offset);
    imu.setYGyroOffset(gy_offset);
    imu.setZGyroOffset(gz_offset);

    // if (success == 6) {
        // printf("Successfully imported offsets from offsets.txt\r\n");
    // } else {
        // printf("Failed to import offsets from offsets.txt, defaulting to 0\r\n");
    // }

    // Thread::wait(2000);

    // while (true) {
        // imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", ax, ay, az, gx, gy, gz);
        // Thread::wait(2);
    // }

#endif

    // signal back to main and wait until we're signaled to continue
    osSignalSet(mainID, MAIN_TASK_CONTINUE);
    Thread::signal_wait(SUB_TASK_CONTINUE, osWaitForever);

    std::array<int16_t, 5> duty_cycles{};

    pidController.setPidValues(3.0, 10, 2, 30, 0);

    // initialize timeout timer
    commandTimeoutTimer = make_unique<RtosTimerHelper>(
        [&]() { commandTimedOut = true; }, osTimerPeriodic);

    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    float rot = 0.0f;

    while (true) {
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", ax, ay, az, gx, gy, gz);

        // vel_x += ax;// * 0.001f;
        // vel_y += ay;// * 0.001f;

        // pos_x += vel_x * 1/1000.0f;
        // pos_y += vel_y * 1/1000.0f;
        // rot += gz * .001f;

        // printf("% 10f % 10f % 10f\r\n", pos_x, pos_y, rot);

#if 0 /* enable whenever the imu is actually used */
        imu.getGyro(gyroVals);
        imu.getAccelero(accelVals);
#endif

        if (DebugCommunication::configStoreIsValid
                [DebugCommunication::ConfigCommunication::PID_P]) {
            pidController.updatePValues(DebugCommunication::configValueToFloat(
                DebugCommunication::ConfigCommunication::PID_P,
                DebugCommunication::configStore
                    [DebugCommunication::ConfigCommunication::PID_P]));
        }
        if (DebugCommunication::configStoreIsValid
                [DebugCommunication::ConfigCommunication::PID_I]) {
            pidController.updateIValues(DebugCommunication::configValueToFloat(
                DebugCommunication::ConfigCommunication::PID_I,
                DebugCommunication::configStore
                    [DebugCommunication::ConfigCommunication::PID_I]));
        }
        if (DebugCommunication::configStoreIsValid
                [DebugCommunication::ConfigCommunication::PID_D]) {
            pidController.updateDValues(DebugCommunication::configValueToFloat(
                DebugCommunication::ConfigCommunication::PID_D,
                DebugCommunication::configStore
                    [DebugCommunication::ConfigCommunication::PID_D]));
        }

        // note: the 4th value is not an encoder value.  See the large comment
        // below for an explanation.
        std::array<int16_t, 5> enc_deltas{};

        // zero out command if we haven't gotten an updated target in a while
        if (commandTimedOut) duty_cycles = {0, 0, 0, 0, 0};

        auto statusByte = FPGA::Instance->set_duty_get_enc(
            duty_cycles.data(), duty_cycles.size(), enc_deltas.data(),
            enc_deltas.size());

        /*
         * The time since the last update is derived with the value of
         * WATCHDOG_TIMER_CLK_WIDTH in robocup.v
         *
         * The last encoder reading (5th one) from the FPGA is the watchdog
         * timer's tick since the last SPI transfer.
         *
         * Multiply the received tick count by:
         *     (1/18.432) * 2 * (2^WATCHDOG_TIMER_CLK_WIDTH)
         *
         * This will give you the duration since the last SPI transfer in
         * microseconds (us).
         *
         * For example, if WATCHDOG_TIMER_CLK_WIDTH = 6, here's how you would
         * convert into time assuming the fpga returned a reading of 1265 ticks:
         *     time_in_us = [ 1265 * (1/18.432) * 2 * (2^6) ] = 8784.7us
         *
         * The precision would be in increments of the multiplier. For
         * this example, that is:
         *     time_precision = 6.94us
         *
         */
        const float dt = enc_deltas.back() * (1 / 18.432e6) * 2 * 128;

        // take first 4 encoder deltas
        std::array<int16_t, 4> driveMotorEnc;
        for (auto i = 0; i < 4; i++) {
            driveMotorEnc[i] = enc_deltas[i];
            enc_counts[i] += enc_deltas[i];
        }

        Eigen::Vector4d errors{};
        Eigen::Vector4d wheelVelsOut{};
        Eigen::Vector4d targetWheelVelsOut{};
        // run PID controller to determine what duty cycles to use to drive the
        // motors.
        std::array<int16_t, 4> driveMotorDutyCycles = pidController.run(
            driveMotorEnc, dt, &errors, &wheelVelsOut, &targetWheelVelsOut);

        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::PIDError0] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::PIDError0, errors[0]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::PIDError1] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::PIDError1, errors[1]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::PIDError2] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::PIDError2, errors[2]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::PIDError3] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::PIDError3, errors[3]);

        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::MotorDuty0] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::MotorDuty0,
                    duty_cycles[0]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::MotorDuty1] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::MotorDuty1,
                    duty_cycles[1]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::MotorDuty2] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::MotorDuty2,
                    duty_cycles[2]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::MotorDuty3] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::MotorDuty3,
                    duty_cycles[3]);

        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::WheelVel0] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::WheelVel0,
                    wheelVelsOut[0]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::WheelVel1] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::WheelVel1,
                    wheelVelsOut[1]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::WheelVel2] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::WheelVel2,
                    wheelVelsOut[2]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::WheelVel3] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::WheelVel3,
                    wheelVelsOut[3]);

        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::StallCounter0] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::StallCounter0,
                    wheelStallDetection[0].stall_counter);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::StallCounter1] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::StallCounter1,
                    wheelStallDetection[1].stall_counter);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::StallCounter2] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::StallCounter2,
                    wheelStallDetection[2].stall_counter);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::StallCounter3] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::StallCounter3,
                    wheelStallDetection[3].stall_counter);

        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::TargetWheelVel0] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::TargetWheelVel0,
                    targetWheelVelsOut[0]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::TargetWheelVel1] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::TargetWheelVel1,
                    targetWheelVelsOut[1]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::TargetWheelVel2] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::TargetWheelVel2,
                    targetWheelVelsOut[2]);
        DebugCommunication::debugStore
            [DebugCommunication::DebugResponse::TargetWheelVel3] =
                DebugCommunication::debugResponseToValue(
                    DebugCommunication::DebugResponse::TargetWheelVel3,
                    targetWheelVelsOut[3]);


        // assign the duty cycles, zero out motors that the fpga returns an
        // error for
        static_assert(wheelStallDetection.size() == driveMotorDutyCycles.size(),
                      "wheelStallDetection Size should be the same as "
                      "driveMotorDutyCycles");
        for (int i = 0; i < driveMotorDutyCycles.size(); i++) {
            const auto& vel = driveMotorDutyCycles[i];
            bool didStall = wheelStallDetection[i].stall_update(
                duty_cycles[i], wheelVelsOut[i]);

            const bool hasError = (statusByte & (1 << i)) || didStall;
            duty_cycles[i] = (hasError ? 0 : vel);
        }

        // limit duty cycle values, while keeping sign (+ or -)
        for (auto dc : duty_cycles) {
            if (std::abs(dc) > FPGA::MAX_DUTY_CYCLE) {
                dc = copysign(FPGA::MAX_DUTY_CYCLE, dc);
            }
        }

        // dribbler duty cycle
        duty_cycles[4] = dribblerSpeed;

        Thread::wait(CONTROL_LOOP_WAIT_MS);
    }
}
