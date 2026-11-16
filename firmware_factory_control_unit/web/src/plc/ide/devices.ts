// The PLC devices of this browser tab, shared by the IDE and the HMI runtime: one simulator (its program
// keeps running when the page changes) and one FCU connection.
import { FcuDevice } from "./device.ts";
import { Simulator } from "./simulator.ts";

export const simulator = new Simulator();
export const fcuDevice = new FcuDevice();
