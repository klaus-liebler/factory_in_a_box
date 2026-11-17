// The PLC works with the FCU's register map (generated/register-map.ts): its I/O variables, command
// registers and layout hash (@fiab/plc ide/io-list.ts). Imported first by app.ts and by the tests.
import { configureRegisterMap } from "@fiab/plc/ide/io-list.ts";
import { REGIONS, REGISTER_MAP_HASH } from "../../generated/register-map.ts";

configureRegisterMap(REGIONS, REGISTER_MAP_HASH);
