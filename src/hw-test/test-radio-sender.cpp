/**
 * This program repeatedly sends a control packet to robot 1.  It prints a '-->'
 * to the console for every packet sent and a '<--' for every packet received.
 * This program was created to test the cc1201 configuration/driver in order to
 * ensure that everything works and to tune the register settings.  It is meant
 * to be used along with the radio-receiver-test program.
 */

#include <cmsis_os.h>
#include <memory>
#include "Mbed.hpp"

#include "CC1201Radio.hpp"
#include "Logger.hpp"
#include "SharedSPI.hpp"
#include "pins-control.hpp"

using namespace std;

// how often to send a packet (in seconds)
const float TRANSMIT_INTERVAL = 1.0f / 100.0f;

bool initRadio() {
    // setup SPI bus
    shared_ptr<SharedSPI> sharedSPI =
        make_shared<SharedSPI>(RJ_SPI_MOSI, RJ_SPI_MISO, RJ_SPI_SCK);
    sharedSPI->format(8, 0);  // 8 bits per transfer

    // RX/TX leds
    auto rxTimeoutLED = make_shared<FlashingTimeoutLED>(LED1);
    auto txTimeoutLED = make_shared<FlashingTimeoutLED>(LED2);

    // Startup the CommModule interface
    CommModule::Instance = make_shared<CommModule>(rxTimeoutLED, txTimeoutLED);

    // Construct an object pointer for the radio
    constexpr auto settingsSize =
        sizeof(preferredSettings) / sizeof(registerSetting_t);
    globalRadio = std::make_unique<CC1201>(
        sharedSPI, RJ_RADIO_nCS, RJ_RADIO_INT, preferredSettings, settingsSize);

    return globalRadio->isConnected();
}

void radioRxHandler(rtp::Packet pkt) {
    static int rxCount = 0;
    ++rxCount;
    printf("<-- %d\r\n", rxCount);
}

int main() {
    // set baud rate to higher value than the default for faster terminal
    Serial s(RJ_SERIAL_RXTX);
    s.baud(57600);

    // Set the default logging configurations
    isLogging = RJ_LOGGING_EN;
    rjLogLevel = INIT;

    printf("****************************************\r\n");
    LOG(INFO, "Radio test sender starting...");

    if (initRadio()) {
        LOG(OK, "Radio interface ready on %3.2fMHz!", globalRadio->freq());

        // register handlers for any ports we might use
        for (rtp::Port port :
             {rtp::Port::CONTROL, rtp::Port::PING, rtp::Port::LEGACY}) {
            CommModule::Instance->setRxHandler(&radioRxHandler, port);
            CommModule::Instance->setTxHandler((CommLink*)globalRadio,
                                               &CommLink::sendPacket, port);
        }
    } else {
        LOG(SEVERE, "No radio interface found!");
    }

    DigitalOut senderIndicator(LED3, 1);
    DigitalOut radioStatusLed(LED4, globalRadio->isConnected());

    // send packets every @TRANSMIT_INTERVAL forever
    while (true) {
        static int txCount = 0;

        rtp::Packet pkt;
        pkt.header.port = rtp::Port::CONTROL;
        pkt.header.address = rtp::BROADCAST_ADDRESS;

        // create control message and add it to the packet payload
        rtp::ControlMessage msg;
        msg.uid = 1;  // address message to robot 1
        msg.bodyX = 2;
        msg.bodyY = 3;
        msg.bodyW = 4;
        rtp::SerializeToVector(msg, &pkt.payload);

        // transmit!
        CommModule::Instance->send(std::move(pkt));
        txCount++;
        printf("--> %d\r\n", txCount);

        Thread::wait(TRANSMIT_INTERVAL * 1e3);
    }
}
