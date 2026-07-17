#pragma once
// Ethernet-Link-Ueberwachung -- wird jetzt aus dem einen gemeinsamen io_thread heraus
// periodisch aufgerufen (kein eigener Thread mehr, s. io_thread.cpp).

void link_status_update();
