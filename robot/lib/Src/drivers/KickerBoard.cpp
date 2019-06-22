#include "drivers/KickerBoard.hpp"

#include "delay.hpp"
#include "device-bins/kicker_bin.h"
#include <tuple>

using namespace std;

KickerBoard::KickerBoard(shared_ptr<SPI> spi, std::shared_ptr<DigitalOut> nCs,
                         PinName nReset, PinName ball_led)
    : AVR910(spi, nCs, nReset), _nCs(nCs), _spi(spi)  {}

bool KickerBoard::verify_param(const char* name, char expected,
                               int (AVR910::*paramMethod)(), char mask,
                               bool verbose) {
    if (verbose) printf("Checking %s...", name);
    int val = (*this.*paramMethod)();
    bool success = ((val & mask) == expected);
    if (verbose) {
        if (success)
            printf("done\r\n");
        else
            printf("Got unexpected value: 0x%X\r\n", val);
    }

    return success;
}

bool KickerBoard::flash(const string& proFilename,
                        bool onlyIfDifferent, bool verbose) {
    // Check a few parameters before attempting to flash to ensure that we have
    // the right chip and it's connected correctly.
    auto checks = {
        make_tuple("Vendor ID", ATMEL_VENDOR_CODE, &AVR910::readVendorCode,
                   0xFF),
        make_tuple("Part Family", AVR_FAMILY_ID,
                   &AVR910::readPartFamilyAndFlashSize, AVR_FAMILY_MASK),
        make_tuple("Device ID", ATTINY_DEVICE_ID, &AVR910::readPartNumber,
                   0xFF),
    };
    for (auto& check : checks) {
        if (!verify_param(get<0>(check), get<1>(check), get<2>(check),
                          get<3>(check), verbose)) {
            return false;
        }
    }

    //  Open binary file to write to AVR.
    FILE* fp = fopen(proFilename.c_str(), "r");

    if (fp == nullptr) {
        //LOG(WARN, "Failed to open kicker binary, check path: '%s'",
        //    proFilename.c_str());
        exitProgramming();
        return false;
    } else {
        // Program it!
        //LOG(INFO, "Opened kicker binary, attempting to program kicker.");
        bool shouldProgram = true;
        if (onlyIfDifferent &&
            (checkMemory(ATTINY_PAGESIZE, ATTINY_NUM_PAGES, fp, false) == 0))
            shouldProgram = false;

        if (!shouldProgram) {
            //LOG(INFO, "Kicker up-to-date, no need to flash.");

            // exit programming mode by bringing nReset high
            exitProgramming();
        } else {
            bool success = program(fp, ATTINY_PAGESIZE, ATTINY_NUM_PAGES);

            if (!success) {
                //LOG(WARN, "Failed to program kicker.");
            } else {
                //LOG(INFO, "Kicker successfully programmed.");
            }
        }

        fclose(fp);
    }

    return true;
}

bool KickerBoard::flash(bool onlyIfDifferent, bool verbose) {
    // Check a few parameters before attempting to flash to ensure that we have
    // the right chip and it's connected correctly.
    auto checks = {
        make_tuple("Vendor ID", ATMEL_VENDOR_CODE, &AVR910::readVendorCode,
                   0xFF),
        make_tuple("Part Family", AVR_FAMILY_ID,
                   &AVR910::readPartFamilyAndFlashSize, AVR_FAMILY_MASK),
        make_tuple("Device ID", ATTINY_DEVICE_ID, &AVR910::readPartNumber,
                   0xFF),
    };
    for (auto& check : checks) {
        if (!verify_param(get<0>(check), get<1>(check), get<2>(check),
                          get<3>(check), verbose)) {
            return false;
        }
    }

    const uint8_t* progBinary = KICKER_BYTES;
    unsigned int length = KICKER_BYTES_LEN;

    // Program it!
    printf("Attempting to program kicker.\r\n");
    bool shouldProgram = true;
    if (onlyIfDifferent &&
        (checkMemory(ATTINY_PAGESIZE, ATTINY_NUM_PAGES, progBinary, length, false) == 0))
        shouldProgram = false;

    if (!shouldProgram) {
        //LOG(INFO, "Kicker up-to-date, no need to flash.");
        printf("Kicker up-to-date, no need to flash.\r\n");

        // exit programming mode by bringing nReset high
        exitProgramming();
    } else {
        bool success = program(progBinary, length, ATTINY_PAGESIZE, ATTINY_NUM_PAGES);

        if (!success) {
            //LOG(WARN, "Failed to program kicker.");
            printf("Failed to program kicker.\r\n");
        } else {
            //LOG(INFO, "Kicker successfully programmed.");
            printf("Kicker successfully programmed.\r\n");
        }
    }

    return true;
}

void KickerBoard::service() {
    _spi->frequency(100'000);

    uint8_t command = 0x00;

    if (_is_kick)
        command |= TYPE_KICK;
    else
        command |= TYPE_CHIP;

    if (_kick_immediate) {
        command |= KICK_IMMEDIATE;
        _kick_immediate = false;
    }

    if (_kick_breakbeam) {
        command |= KICK_ON_BREAKBEAM;
        _kick_breakbeam = false;
    }

    if (_cancel_kick) {
        command |= CANCEL_KICK;
        _cancel_kick = false;
    }

    if (_charge_allowed) {
        command |= CHARGE_ALLOWED;
    }

    command |= _kick_strength & KICK_POWER_MASK;

    _nCs->write(0);
    DWT_Delay(10);

    uint8_t resp = _spi->transmitReceive(command);

    DWT_Delay(10);
    _nCs->write(1);


    _current_voltage = (resp & VOLTAGE_MASK) * VOLTAGE_SCALE;

    _ball_sensed = resp & BREAKBEAM_TRIPPED;

    // Assume healthy if we get some voltage back
    _is_healthy = _current_voltage > 0;
}

void KickerBoard::kickType(bool isKick) {
    _is_kick = isKick;
}

void KickerBoard::kick(uint8_t strength) {
    _kick_immediate = true;
    _kick_breakbeam = false;
    _kick_strength = strength;
    _cancel_kick = false;
}

void KickerBoard::kickOnBreakbeam(uint8_t strength) {
    _kick_immediate = false;
    _kick_breakbeam = true;
    _kick_strength = strength;
    _cancel_kick = false;
}

void KickerBoard::cancelBreakbeam() {
    _kick_immediate = false;
    _kick_breakbeam = false;
    _cancel_kick = true;
}

bool KickerBoard::isBallSensed() { return _ball_sensed; }

bool KickerBoard::isHealthy() { return _is_healthy; }

uint8_t KickerBoard::getVoltage() { return _current_voltage; }

bool KickerBoard::isCharged() { return getVoltage() > isChargedCutoff; }

void KickerBoard::setChargeAllowed(bool chargeAllowed) {
    _charge_allowed = chargeAllowed;
}
