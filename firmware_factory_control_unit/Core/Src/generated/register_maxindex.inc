// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.
//
// Nur per #include aus Core/Src/modbus_register_model.hh eingebunden (innerhalb des dort
// bereits geoeffneten namespace-Blocks), nicht eigenstaendig uebersetzbar -- enthaelt bewusst
// nur constexpr-Zeilen, keine eigene namespace-Deklaration/Klammern.
//
// Hoechster belegter Index je Register-Bank -- bestimmt die Groesse der
// ModbusRegisterModel-Arrays (s. modbus_register_model.hh). Bei Erweiterung der Register-Map
// (neue Region ans Ende angehaengt) hier mitziehen. Referenziert Input::/Holding::, die im
// umschliessenden modbus_register_model.hh bereits vor dieser #include definiert sind.
constexpr uint16_t INPUT_REGISTER_MAX_INDEX   = Input::STEPPER_STATUS_REGION_END;
constexpr uint16_t HOLDING_REGISTER_MAX_INDEX = Holding::WS2812_REGION_END;
