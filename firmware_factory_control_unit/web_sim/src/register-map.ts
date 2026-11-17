// The FCU's register map (web/generated/register-map.ts, written by the builder's GenerateRegisterAccessFiles
// step - no hardware needed): the simulated PLC has the same I/O variables, command registers and map hash as
// the device.
import { configureRegisterMap } from "@fiab/plc/ide/io-list.ts";
import { REGIONS, REGISTER_MAP_HASH } from "../../web/generated/register-map.ts";

configureRegisterMap(REGIONS, REGISTER_MAP_HASH);
