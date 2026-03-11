//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// USB PD sink
//

#include "PDSink.h"
#include "TaskScheduler.h"
#include "stm32g4xx_hal.h"

PDSink PowerSink;

PDSink::PDSink(bool busPowered)
    : isBusPowered(busPowered), eventCallback(nullptr), ppsIndex(-1), desiredVoltage(5000),
      desiredCurrent(0), capabilitiesChanged(false), flagSourceCapChanged(false),
      flagVoltageChanged(false), flagPowerRejected(false) {}

void PDSink::start(EventCallbackFunction callback) {
    Scheduler.start();
    eventCallback = callback;
    reset(false);
    PowerController.startController([this](auto event) { handleEvent(event); });
}

void PDSink::reset(bool connected) {
    activeVoltage = isBusPowered || connected ? 5000 : 0;
    activeCurrent = isBusPowered || connected ? 900 : 0;
    requestedVoltage = 5000;
    requestedCurrent = 900;
    numSourceCapabilities = 0;
    flagSourceCapChanged = true;
    flagVoltageChanged = true;
    flagPowerRejected = false;
    capabilitiesChanged = false;
    Scheduler.cancelTask(rerequestPPSCallback);
    Scheduler.cancelTask(requestSourceCapsCallback);
    if (connected)
        Scheduler.scheduleTaskAfter(requestSourceCapsCallback, SourceCapsRequestDelayUs);
}

void PDSink::Loop() {
    if (eventCallback == nullptr)
        return;

    while (flagSourceCapChanged || flagVoltageChanged || flagPowerRejected) {
        if (flagSourceCapChanged) {
            flagSourceCapChanged = false;
            eventCallback(PDSinkEventType::sourceCapabilitiesChanged);
        }
        if (flagVoltageChanged) {
            flagVoltageChanged = false;
            eventCallback(PDSinkEventType::voltageChanged);
        }
        if (flagPowerRejected) {
            flagPowerRejected = false;
            eventCallback(PDSinkEventType::powerRejected);
        }
    }
}

bool PDSink::isConnected() {
    return numSourceCapabilities > 0;
}

bool PDSink::TryRequestPreferredVoltages(std::vector<int> desiredVoltagesDescendingPriority_mV) {
    for (int voltage_mV : desiredVoltagesDescendingPriority_mV) {
        if (canProvideVoltage(voltage_mV)) {
            return requestPower(voltage_mV);
        }
    }

    return false;
}

bool PDSink::requestPower(int voltage, int maxCurrent) {
    desiredVoltage = voltage;
    desiredCurrent = maxCurrent;

    while (true) {
        ErrorCode err = requestPowerCore(voltage, maxCurrent);

        if (err == ok)
            return true;

        if (err == controllerBusy) {
            // retry after delay
            HAL_Delay(1);
            continue;
        }

        return false;
    }

    // loop until message could be transmitted
    // (controller could be busy)
    while (isConnected()) {
        bool successful = requestPowerCore(voltage, maxCurrent);
        if (successful)
            break;
        HAL_Delay(1);
    }
}

PDSink::ErrorCode PDSink::requestPowerCore(int voltage, int maxCurrent) {

    if (!isConnected())
        return notConnected;

    // create 'Request' message
    int capabilityIndex = -1;
    uint32_t requestObject = PDSourceCapability::powerRequestObject(capabilityIndex, voltage, maxCurrent,
                                                                    numSourceCapabilities, sourceCapabilities);

    if (requestObject == 0)
        return noMatchingCapability;

    // send message
    bool successful = PowerController.sendDataMessage(PDMessageType::dataRequest, 1, &requestObject);
    if (!successful)
        return controllerBusy;

    Scheduler.cancelTask(rerequestPPSCallback);

    ppsIndex = -1;
    if (sourceCapabilities[capabilityIndex].supplyType == PDSupplyType::pps)
        ppsIndex = capabilityIndex;

    requestedVoltage = voltage;
    requestedCurrent = maxCurrent;

    return ok;
}

void PDSink::handleEvent(const PDControllerEvent& event) {
    switch (event.type) {

    case PDControllerEventType::reset:
    case PDControllerEventType::connected:
    case PDControllerEventType::disconnected:
        reset(event.type == PDControllerEventType::connected);
        break;

    case PDControllerEventType::messageReceived:
        onMessageReceived(event.message);
        break;

    default:; // ignore
    }
}

bool PDSink::canProvideVoltage(int voltage) {
    for (int i = 0; i < numSourceCapabilities; i++) {
        if (sourceCapabilities[i].minVoltage <= voltage && sourceCapabilities[i].maxVoltage >= voltage)
            return true;
    }
    return false;
}

void PDSink::onMessageReceived(const PDMessage* message) {
    switch (message->type()) {

    case PDMessageType::dataSourceCapabilities:
        onSourceCapabilities(message);
        break;

    case PDMessageType::controlReject:
        flagPowerRejected = true;
        break;

    case PDMessageType::controlPsReady:
        onPsReady();
        break;

    default:; // nothing to do
    }
}

void PDSink::onSourceCapabilities(const PDMessage* message) {
    // parse source capabilities message
    PDSourceCapability::parseMessage(message, numSourceCapabilities, sourceCapabilities);

    Scheduler.cancelTask(requestSourceCapsCallback);

    capabilitiesChanged = true;

    ErrorCode err = requestPowerCore(desiredVoltage, desiredCurrent);
    if (err != ok)
        requestPowerCore(5000, 0); // fallback: select 5V
}

void PDSink::onPsReady() {
    if (capabilitiesChanged) {
        flagSourceCapChanged = true;
        capabilitiesChanged = false;
    }
    
    if (activeVoltage != requestedVoltage || activeCurrent != requestedCurrent)
        flagVoltageChanged = true;
    activeVoltage = requestedVoltage;
    activeCurrent = requestedCurrent;

    // PPS voltages need to be requested at least every 10s; otherwise
    // the power supply returns to 5V
    if (ppsIndex >= 0)
        Scheduler.scheduleTaskAfter(rerequestPPSCallback, 8000000);
}

void PDSink::rerequestPPSCallback() {
    PowerSink.onRerequestPPS();
}

void PDSink::onRerequestPPS() {
    // request the same voltage again
    uint32_t requestObject = PDSourceCapability::powerRequestObject(ppsIndex, requestedVoltage, requestedCurrent,
                                                                    numSourceCapabilities, sourceCapabilities);
    bool successful = PowerController.sendDataMessage(PDMessageType::dataRequest, 1, &requestObject);
    if (!successful)
        Scheduler.scheduleTaskAfter(rerequestPPSCallback, 100000);
}

void PDSink::requestSourceCapsCallback() {
    PowerSink.onRequestSourceCaps();
}

void PDSink::onRequestSourceCaps() {
    if (PowerController.ccPin == 0 || numSourceCapabilities > 0)
        return;

    bool sent = PowerController.sendControlMessage(PDMessageType::controlGetSourceCap);
    if (!sent) {
        Scheduler.scheduleTaskAfter(requestSourceCapsCallback, 10000);
    } else {
        Scheduler.scheduleTaskAfter(requestSourceCapsCallback, SourceCapsRetryDelayUs);
    }
}
