// The tests run against the FCU's register map (web/generated/register-map.ts, written by the builder's
// GenerateRegisterAccessFiles step - no hardware needed), as the apps hand it in.
import { configureRegisterMap } from "../src/ide/io-list.ts";
import { REGIONS, REGISTER_MAP_HASH } from "../../../web/generated/register-map.ts";

configureRegisterMap(REGIONS, REGISTER_MAP_HASH);
