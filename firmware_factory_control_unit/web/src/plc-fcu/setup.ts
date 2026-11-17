// The PLC part of the FCU app: the register map and the FCU as the device next to the simulator.
// Imported by app.ts before the PLC workspaces are created.
import "./register-map.ts";
import { registerRemoteDevice } from "@fiab/plc/ide/devices.ts";
import { FcuDevice } from "./fcu-device.ts";

export const fcuDevice = new FcuDevice();
registerRemoteDevice(fcuDevice);
