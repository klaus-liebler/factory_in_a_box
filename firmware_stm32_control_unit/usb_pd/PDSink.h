//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//
// USB PD sink
//

#pragma once

#include "PDController.h"
#include "PDSourceCapability.h"
#include <functional>
#include <vector>

enum class PDSinkEventType {
    sourceCapabilitiesChanged,
    voltageChanged,
    powerRejected
};

/**
 * Power Delivery Sink
 *
 * Communicates with a power source (power supply) to control the voltage
 * and current being supplied.
 *
 * A single global instance of this class exists. It is called 'PowerSink'.
 */
class PDSink {
public:

    /**
    * Checks if the power supply can provide the specified voltage.
    */    
    bool canProvideVoltage(int voltage);
    /**
     * Type of function that will be called when an event has occurred.
     */
    typedef std::function<void(PDSinkEventType eventType)> EventCallbackFunction;

    /**
     * @brief Tries to request the first available voltage from a priority-sorted list.
     *
     * Voltages must be ordered from highest to lowest priority.
     *
     * @param desiredVoltagesDescendingPriority_mV desired voltages in mV, sorted by priority
        * @return true if one of the desired voltages was successfully requested, otherwise false
     */
        bool TryRequestPreferredVoltages(std::vector<int> desiredVoltagesDescendingPriority_mV);

    /**
     * @brief Construct a new instance
     * 
     * For correct voltage reporting, the power sink needs to know if it is
     * bus powered (MCU is powered from the same USB connection that it
     * interacts with for the USB-PD communication), or self-powered (MCU has
     * power supply separate from the one it interacts with).
     * 
     * @param busPowered 'true' if power sink is bus powered, 'false' otherwise
     */
    PDSink(bool busPowered = false);

    /**
     * Starts USB Power Delivery as a power sink.
     *
     * The callback function will be called when 'Loop()' is called and
     * an event has occurred since the last call of 'Loop()'.
     *
     * @param callback event callback function
     */
    void start(EventCallbackFunction callback = nullptr);

    /**
     * Polls for new events.
     * 
     * Call this function frequently from 'loop()'.
     */
    void Loop();

    /// Indicates if the sink is connected to a USB PD power supply
    bool isConnected();

    /**
     * Requests a new voltage and optionally a maximum current.
     * 
     * If the method is successful, itwill also set 'requestedVoltage' and
     * 'requestedCurrent' to reflect what has actually been requested from
     * the power supply. These values might differ because of rounding and
     * unspecified current.
     * 
     * If the method is not successful, it will still remember the voltage
     * and current values. If the power supply becomes available or changes,
     * the sink will try to request these values.
     * 
     * @param voltage voltage, in mV
     * @param maxCurrent maximum current, in mA (or 0 to selected maximum current)
     * @return 'true' if the desired voltage and current are available
     */
    bool requestPower(int voltage, int maxCurrent = 0);

    /// Number of valid elements in `sourceCapabilities` array
    int numSourceCapabilities = 0;

    /// Array of supply capabilities
    PDSourceCapability sourceCapabilities[7];

    /// Active voltage in mV
    int activeVoltage;

    /// Active maximum current in mA
    int activeCurrent;

    /// Requested voltage in mV
    int requestedVoltage;

    /// Requested current in mA
    int requestedCurrent;

private:
    enum ErrorCode {
        ok,
        notConnected,
        noMatchingCapability,
        controllerBusy
    };

    bool isBusPowered;

    EventCallbackFunction eventCallback;
    int ppsIndex;
    int desiredVoltage;
    int desiredCurrent;

    bool capabilitiesChanged;

    // event flags
    bool flagSourceCapChanged;
    bool flagVoltageChanged;
    bool flagPowerRejected;

    static constexpr uint32_t SourceCapsRequestDelayUs = 100000;
    static constexpr uint32_t SourceCapsRetryDelayUs = 200000;

    ErrorCode requestPowerCore(int voltage, int maxCurrent);
    void handleEvent(const PDControllerEvent& event);

    void onMessageReceived(const PDMessage* message);
    void onSourceCapabilities(const PDMessage* message);
    void onPsReady();

    void reset(bool connected);

    void onRerequestPPS();
    static void rerequestPPSCallback();

    void onRequestSourceCaps();
    static void requestSourceCapsCallback();
};

/// Global object for controlling USB power sink
extern class PDSink PowerSink;
