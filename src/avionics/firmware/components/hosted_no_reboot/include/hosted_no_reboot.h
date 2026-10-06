/* esp_hosted (the C6 Wi-Fi link) bent to the flight computer's needs (see CMakeLists.txt): a failing link never
 * reboots the P4, and its tasks stay off the flight core. Both hooks install from a constructor, before
 * esp_hosted starts; nothing to call. */
#pragma once

#include <stdbool.h>

/* True once the reboot guard is in place (false: the table did not look as expected and nothing was written). */
bool hosted_no_reboot_installed(void);

/* True once esp_hosted creates its tasks pinned to core 0 (false: the table did not look as expected). */
bool hosted_tasks_pinned(void);

/* True once esp_hosted has asked for a host restart: the transport task is parked, Wi-Fi is gone until the
 * next boot, and any further Wi-Fi call waits out an RPC timeout -- so never call Wi-Fi from the control loop. */
bool hosted_link_down(void);
