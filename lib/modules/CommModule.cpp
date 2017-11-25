#include "CommModule.hpp"

#include "Assert.hpp"
#include "CommPort.hpp"
#include "Console.hpp"
#include "HelperFuncs.hpp"
#include "Logger.hpp"

#include <ctime>

osPoolDef(CommModuleTxPool, 4, rtp::Packet);
osPoolDef(CommModuleRxPool, 4, rtp::Packet);

osMessageQDef(CommModuleTxQueue, 3, rtp::Packet);
osMessageQDef(CommModuleRxQueue, 3, rtp::Packet);

std::shared_ptr<CommModule> CommModule::Instance = nullptr;

CommModule::CommModule()
    : m_txPoolId(osPoolCreate(osPool(CommModuleTxPool))),
      m_rxPoolId(osPoolCreate(osPool(CommModuleRxPool))),
      m_txMessageQueue(osMessageCreate(osMessageQ(CommModuleTxQueue), nullptr)),
      m_rxMessageQueue(osMessageCreate(osMessageQ(CommModuleRxQueue), nullptr)),
      m_rxThread(&CommModule::rxThreadHelper, this, RX_PRIORITY, STACK_SIZE),
      m_txThread(&CommModule::txThreadHelper, this, TX_PRIORITY, STACK_SIZE) {}

void CommModule::txThread() {
    m_txThreadId = osThreadGetId();

    // Only continue once we know there's 1+ hardware link(s) available
    osSignalWait(SIGNAL_START, osWaitForever);

    // Store our priority so we know what to reset it to if ever needed
    const auto threadPriority = osThreadGetPriority(m_txThreadId);

    LOG(OK, "TX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        reinterpret_cast<P_TCB>(m_rxThread.gettid())->task_id, threadPriority);

    // Signal to the RX thread that it can begin
    osSignalSet(m_rxThreadId, SIGNAL_START);

    while (true) {
        // Wait until new data is placed in the RX queue
        auto event = osMessageGet(m_txMessageQueue, osWaitForever);

        if (event.status == osEventMessage) {
            auto p = reinterpret_cast<rtp::Packet*>(event.value.p);

            [[gnu::unused]] auto tState = osThreadSetPriority(m_txThreadId, osPriorityHigh);
            ASSERT(tState == osOK);

            // grab the port number
            const auto portNum = p->header.type;

            // grab an iterator to the port, lookup only once
            const auto portIter = m_ports.find(portNum);

            // invoke callback if port exists and has an attached function
            if (portIter != m_ports.end()) {
                if (portIter->second.hasTxCallback()) {
                    portIter->second.getTxCallback()(p);

                    // LOG(INFO,
                    //     "Transmission:\r\n"
                    //     "    Port:\t%u\r\n",
                    //     portNum);
                }
            }

            // destruct and free from the memory pool
            p->~Packet();
            osPoolFree(m_txPoolId, p);

            tState = osThreadSetPriority(m_txThreadId, threadPriority);
            ASSERT(tState == osOK);

        } else {
            std::printf(
                "osMessageGet for TX returned unexpected status: %d\r\n",
                event.status);
            fflush(stdout);
            ASSERT(false);
        }

        osThreadYield();
    }

    ASSERT(!"Execution is at an unreachable line!");
}

void CommModule::rxThread() {
    m_rxThreadId = osThreadGetId();

    // Only continue once we know there's 1+ hardware link(s) available
    osSignalWait(SIGNAL_START, osWaitForever);

    // set this true immediately after we are released execution
    m_isRunning = true;

    // Store our priority so we know what to reset it to if ever needed
    const auto threadPriority = osThreadGetPriority(m_rxThreadId);

    LOG(OK, "RX communication module ready!\r\n    Thread ID: %u, Priority: %d",
        reinterpret_cast<P_TCB>(m_rxThread.gettid())->task_id, threadPriority);

    size_t rxCount = 0;

    while (true) {
        // Wait until new data is placed in the RX queue
        auto event = osMessageGet(m_rxMessageQueue, osWaitForever);

        if (event.status == osEventMessage) {
            auto p = reinterpret_cast<rtp::Packet*>(event.value.p);

            // Bump up the thread's priority
            [[gnu::unused]] auto tState = osThreadSetPriority(m_rxThreadId, osPriorityHigh);
            ASSERT(tState == osOK);

            // grab the port number
            const auto portNum = p->header.type;

            // grab an iterator to the port, lookup only once
            const auto portIter = m_ports.find(portNum);

            // invoke callback if port exists and has an attached function
            if (portIter != m_ports.end()) {
                if (portIter->second.hasRxCallback()) {
                    rxCount++;
                    if (rxCount % 2000 == 0) std::printf("%u\r\n", rxCount);

                    portIter->second.getRxCallback()(*p);

                    // LOG(INFO,
                    //     "Reception:\r\n"
                    //     "    Port:\t%u\r\n",
                    //     portNum);
                }
            }

            // destruct and free from the memory pool
            p->~Packet();
            osPoolFree(m_rxPoolId, p);

            tState = osThreadSetPriority(m_rxThreadId, threadPriority);
            ASSERT(tState == osOK);

        } else {
            std::printf(
                "osMessageGet for RX returned unexpected status: %d\r\n",
                event.status);
            fflush(stdout);
            ASSERT(false);
        }

        osThreadYield();
    }

    ASSERT(!"Execution is at an unreachable line!");
}

void CommModule::send(rtp::Packet packet) {
    const auto portNum = packet.header.type;
    const auto portExists = m_ports.find(portNum) != m_ports.end();
    const auto hasCallback = m_ports[portNum].hasTxCallback();

    // Check to make sure a socket for the port exists
    if (portExists && hasCallback) {
        // Place the passed packet into the txQueue.
        auto block = osPoolAlloc(m_txPoolId);
        // Drop packet if unable to allocate
        if (block) {
            auto ptr = new (block) rtp::Packet(std::move(packet));
            // Signal worker thread
            osMessagePut(m_txMessageQueue, reinterpret_cast<uint32_t>(ptr),
                         osWaitForever);
        }
        osThreadYield();
    } else {
        LOG(WARN,
            "Failed to send %u byte packet: No TX socket on port %u exists",
            packet.payload.size(), packet.header.type);
    }
}

void CommModule::receive(rtp::Packet packet) {
    const auto portNum = packet.header.type;
    const auto portExists = m_ports.find(portNum) != m_ports.end();
    const auto hasCallback = m_ports[portNum].hasRxCallback();

    // Check to make sure a socket for the port exists
    if (portExists && hasCallback) {
        // Place the passed packet into
        auto block = osPoolAlloc(m_rxPoolId);
        // Drop packet if unable to allocate
        if (block) {
            auto ptr = new (block) rtp::Packet(std::move(packet));
            // Signal worker thread
            osMessagePut(m_rxMessageQueue, reinterpret_cast<uint32_t>(ptr),
                         osWaitForever);
        }
        osThreadYield();
    } else {
        // We think this printout was jamming up the threads and preventing
        // the RX socket from ever opening.
        /*
        LOG(WARN,
            "Failed to send %u byte packet: No RX socket on port %u exists",
            packet.payload.size(), packet.header.port);
        */
    }
}

void CommModule::setRxHandler(RxCallbackT callback, rtp::MessageType portNbr) {
    m_ports[portNbr].setRxCallback(std::bind(callback, std::placeholders::_1));
    ready();
}

void CommModule::setTxHandler(TxCallbackT callback, rtp::MessageType portNbr) {
    m_ports[portNbr].setTxCallback(std::bind(callback, std::placeholders::_1));
    ready();
}

void CommModule::ready() {
    if (m_isReady) m_txThread.signal_set(SIGNAL_START);
    m_isReady = true;
}

void CommModule::close(rtp::MessageType portNbr) noexcept {
    if (m_ports.count(portNbr)) m_ports.erase(portNbr);
}

#ifndef NDEBUG
unsigned int CommModule::numOpenSockets() const {
    auto count = 0;
    for (const auto& kvpair : m_ports) {
        if (kvpair.second.hasRxCallback() || kvpair.second.hasTxCallback())
            ++count;
    }
    return count;
}

unsigned int CommModule::numRxPackets() const {
    auto count = 0;
    for (const auto& kvpair : m_ports) count += kvpair.second.getRxCount();
    return count;
}

unsigned int CommModule::numTxPackets() const {
    auto count = 0;
    for (const auto& kvpair : m_ports) count += kvpair.second.getTxCount();
    return count;
}

void CommModule::resetCount(rtp::MessageType portNbr) {
    m_ports[portNbr].resetCounts();
}

void CommModule::printInfo() const {
    printf("PORT\t\tIN\tOUT\tRX CBCK\t\tTX CBCK\r\n");

    for (const auto& kvpair : m_ports) {
        const PortT& p = kvpair.second;
        printf("%d\t\t%u\t%u\t%s\t\t%s\r\n", kvpair.first, p.getRxCount(),
               p.getTxCount(), p.hasRxCallback() ? "YES" : "NO",
               p.hasTxCallback() ? "YES" : "NO");
    }

    printf(
        "==========================\r\n"
        "Total:\t\t%u\t%u\r\n",
        numRxPackets(), numTxPackets());

    Console::Instance->Flush();
}
#endif
