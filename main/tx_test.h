#pragma once

// Transmit synthetic PMV-107J and Toyota TPMS packets through the CC1101 at
// minimum power, for checking the RTL-SDR / rtl_433 setup on the bench.
// GDO0 is temporarily switched to an output and used as the async TX data input.
void tx_test_send(int pin_gdo0, int repeats);
