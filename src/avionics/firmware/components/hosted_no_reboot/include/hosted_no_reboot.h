/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* esp_hosted (the C6 Wi-Fi link) bent to the flight computer's needs (see CMakeLists.txt): a failing link never
 * reboots the P4, its tasks stay off the flight core, and its DMA buffers live in PSRAM. The hooks install from a
 * constructor, before esp_hosted starts; nothing to call. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/* True once the reboot guard is in place (false: the table did not look as expected and nothing was written). */
bool hosted_no_reboot_installed(void);

/* True once esp_hosted creates its tasks pinned to core 0 (false: the table did not look as expected). */
bool hosted_tasks_pinned(void);

/* True once esp_hosted has asked for a host restart: the transport task is parked, Wi-Fi is gone until the
 * next boot, and any further Wi-Fi call waits out an RPC timeout -- so never call Wi-Fi from the control loop. */
bool hosted_link_down(void);

/* True once esp_hosted takes its DMA buffers (frame pool, TX copies, RX stream) from PSRAM (false: the table did not
 * look as expected, and they stay in internal DMA memory, where running out asserts). */
bool hosted_buffers_in_psram(void);

/* How many of those buffers PSRAM had no room for since boot, so they came from internal DMA memory after all. */
uint32_t hosted_internal_fallbacks(void);
