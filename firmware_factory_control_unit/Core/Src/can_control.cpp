// ============================================================================
// CAN -- FDCAN1-Basisstatistik (empfangene/gesendete Frames, Fehlerzaehler, Bus-State). Kein
// eigenes Protokoll auf dem CAN-Bus implementiert (kein aktueller Verwendungszweck bekannt) --
// reine Low-Level-Zaehler fuers Register-Map, per Polling statt Interrupt (FDCAN1 NVIC ist
// aktuell nicht in CubeMX freigegeben).
//
// TX_COUNT bleibt bei 0, solange keine Stelle im Code tatsaechlich CAN-Frames sendet -- hier
// wird nichts von sich aus transmittiert.
// ============================================================================
#include "can_control.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "log.h"

extern "C" FDCAN_HandleTypeDef hfdcan1;

static uint32_t s_rx_count = 0;
static uint32_t s_tx_count = 0;

void can_setup() {
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000; // Mask 0 -> alle Standard-IDs treffen den Filter
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    // Ohne konfigurierten Filter verwirft FDCAN nicht passende Frames per Default (RejectRemote/
    // RejectNonMatching) -- globalen Filter bewusst permissiv setzen, da hier nur mitgezaehlt
    // statt gefiltert werden soll.
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                  FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        log_warn("FDCAN1: HAL_FDCAN_Start failed");
    }
}

void can_update() {
    ModbusTcpServer& server = *g_app_state.modbus_server;

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            s_rx_count++;
        } else {
            break;
        }
    }

    FDCAN_ProtocolStatusTypeDef proto_status;
    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &proto_status);

    FDCAN_ErrorCountersTypeDef error_counters;
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters);
    uint32_t error_count = error_counters.TxErrorCnt + error_counters.RxErrorCnt;

    // Bus-State: 0=Error-Active, 1=Error-Passive, 2=Bus-Off
    uint16_t bus_state = proto_status.BusOff ? 2 : (proto_status.ErrorPassive ? 1 : 0);

    server.write_input_register(ModbusRegisters::Input::CAN_TX_COUNT_HI, (uint16_t)(s_tx_count >> 16));
    server.write_input_register(ModbusRegisters::Input::CAN_TX_COUNT_LO, (uint16_t)(s_tx_count & 0xFFFF));
    server.write_input_register(ModbusRegisters::Input::CAN_RX_COUNT_HI, (uint16_t)(s_rx_count >> 16));
    server.write_input_register(ModbusRegisters::Input::CAN_RX_COUNT_LO, (uint16_t)(s_rx_count & 0xFFFF));
    server.write_input_register(ModbusRegisters::Input::CAN_ERROR_COUNT_HI, (uint16_t)(error_count >> 16));
    server.write_input_register(ModbusRegisters::Input::CAN_ERROR_COUNT_LO, (uint16_t)(error_count & 0xFFFF));
    server.write_input_register(ModbusRegisters::Input::CAN_LAST_ERROR, (uint16_t)proto_status.LastErrorCode);
    server.write_input_register(ModbusRegisters::Input::CAN_BUS_STATE, bus_state);
}
