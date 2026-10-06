/* Keep a failing C6 Wi-Fi link from rebooting the P4 (see CMakeLists.txt). */
#pragma once

#include <stdbool.h>

/* Point esp_hosted's restart hook at a parking function instead of esp_restart(). Call early in main. */
void hosted_no_reboot_install(void);

/* True once the guard is in place (false: the table did not look as expected and nothing was written). */
bool hosted_no_reboot_installed(void);

/* True once esp_hosted has asked for a host restart: the transport task is parked, Wi-Fi is gone until the
 * next boot, and any further Wi-Fi call waits out an RPC timeout -- so never call Wi-Fi from the control loop. */
bool hosted_link_down(void);
