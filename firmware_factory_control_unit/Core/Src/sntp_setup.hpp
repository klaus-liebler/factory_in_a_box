#pragma once
// Real wall-clock time via SNTP (see libs/opcua_native/include/opcua/clock.hpp for why: no RTC
// clock source is usable on this board's current wiring). Own dedicated thread (mirrors
// opcua_setup.hpp's reasoning for its own transport, just without a ready-made auto-starting
// server) since it needs to block waiting for the network to come up, then for DNS/SNTP -- must
// not hold up App::AppThread()'s other setup calls.
class App;

void SntpSetup(App *app);
