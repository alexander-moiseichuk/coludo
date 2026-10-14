/* Coludo project, copyright under MIT license, Alexander Moiseichuk */

#include "safe_restart.h"

#include "esp32p4/rom/cache.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_ipc_isr.h"
#include "esp_macros.h"
#include "esp_memory_utils.h"
#include "esp_newlib.h"
#include "esp_private/rtc_clk.h"
#include "esp_private/system_internal.h"
#include "esp_rom_sys.h"
#include "hal/wdt_hal.h"
#include "riscv/rv_utils.h"
#include "soc/cache_reg.h"
#include "soc/rtc.h"
#include "soc/soc.h"

/* How long the reset sequence may take before the RTC watchdog resets the chip, ms, as IDF's: it takes microseconds. */
#define GUARD_MS 1000

/* How long the restart waits for the cache's sync engine, us: one sync takes microseconds. Bounded: the done bit
 * reads 0 until the first sync since the chip's reset, and the restart must not hang on it. */
#define SYNC_WAIT_US 1000

/* As IDF's system_internal.c defines it. */
#define ALIGN_DOWN(val, align) ((val) & ~((align) - 1))

extern int _bss_end;

/* The RTC watchdog armed and ENABLED, IDF's stages: a system reset after GUARD_MS, an RTC reset after twice that, and
 * flash-boot protection over the next boot. IDF's esp_restart_noos configures the same stages after its wdt_hal_init
 * has disabled the watchdog, and never enables it -- and the app disabled it at startup (init_disable_rtc_wdt,
 * BOOTLOADER_WDT_DISABLE_IN_USER_CODE unset) -- so once it has disabled the timer-group watchdogs too, nothing guards
 * the cache write-back, the module resets and the CPU resets: the taster hung there silently on 2026-10-13. */
static void arm_guard(void)
{
    wdt_hal_context_t rtc;
    wdt_hal_init(&rtc, WDT_RWDT, 0, false);
    uint32_t ticks = (uint32_t)((uint64_t)GUARD_MS * rtc_clk_slow_freq_get_hz() / 1000);
    wdt_hal_write_protect_disable(&rtc);
    wdt_hal_config_stage(&rtc, WDT_STAGE0, ticks, WDT_STAGE_ACTION_RESET_SYSTEM);
    wdt_hal_config_stage(&rtc, WDT_STAGE1, ticks, WDT_STAGE_ACTION_RESET_RTC);
    wdt_hal_set_flashboot_en(&rtc, true);
    wdt_hal_enable(&rtc);
    wdt_hal_write_protect_enable(&rtc);
}

/* IDF v5.5.5's esp32p4 esp_restart_noos_inner, line for line (system_internal.c): the cache written back and
 * disabled, the modules reset, the clocks back to their reset state, both CPUs reset. In IRAM: the cache is off. */
static void IRAM_ATTR __attribute__((noinline, noreturn)) restart_inner(void)
{
    const uint32_t core_id = esp_cpu_get_core_id();
#if CONFIG_SPIRAM
    Cache_WriteBack_All(CACHE_MAP_L1_DCACHE);
#endif
    Cache_Disable_L2_Cache();
    esp_system_reset_modules_on_exit();
#if !CONFIG_IDF_ENV_FPGA
    rtc_clk_cpu_set_to_default_config();
#endif
#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    ets_set_appcpu_boot_addr(0);
#endif
#if CONFIG_SPIRAM_INSTRUCTIONS_RODATA
    REG_CLR_BIT(HP_SYS_HP_PSRAM_FLASH_ADDR_INTERCHANGE_REG,
                HP_SYS_HP_PSRAM_FLASH_ADDR_INTERCHANGE_DMA | HP_SYS_HP_PSRAM_FLASH_ADDR_INTERCHANGE_CPU);
#endif
    if (core_id == 0) {
#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
        esp_cpu_reset(1);
#endif
        esp_cpu_reset(0);
    }
#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    else {
        esp_cpu_reset(0);
        esp_cpu_unstall(0);
        esp_cpu_reset(1);
    }
#endif
    ESP_INFINITE_LOOP();
}

/* IDF v5.5.5's esp32p4 esp_restart_noos, line for line, but for the guard: the RTC watchdog stays enabled through the
 * reset sequence, so a stall in it resets the chip within GUARD_MS (the next boot then reports a watchdog reset). */
static void __attribute__((noreturn)) restart_guarded(void)
{
    rv_utils_intr_global_disable();
    arm_guard();
#if !CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    const uint32_t core_id = esp_cpu_get_core_id();
    const uint32_t other_core_id = (core_id == 0) ? 1 : 0;
    esp_cpu_reset(other_core_id);
    esp_cpu_stall(other_core_id);
#endif
    wdt_hal_context_t wdt0 = {.inst = WDT_MWDT0, .mwdt_dev = &TIMERG0};
    wdt_hal_write_protect_disable(&wdt0);
    wdt_hal_disable(&wdt0);
    wdt_hal_write_protect_enable(&wdt0);
    wdt_hal_context_t wdt1 = {.inst = WDT_MWDT1, .mwdt_dev = &TIMERG1};
    wdt_hal_write_protect_disable(&wdt1);
    wdt_hal_disable(&wdt1);
    wdt_hal_write_protect_enable(&wdt1);
#ifdef CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM
    /* a stack in PSRAM: onto internal RAM before the cache goes off */
    if (esp_ptr_external_ram(esp_cpu_get_sp())) {
        uint32_t new_sp = ALIGN_DOWN((uint32_t)&_bss_end, 16);
        rv_utils_set_sp((void *)new_sp);
    }
#endif
    restart_inner();
}

void safe_restart(void)
{
    esp_sync_timekeeping_timers();
    /* the other core into a high-priority interrupt, spinning in IRAM; this core's interrupts masked. A sync it had
     * started finishes in hardware meanwhile. */
    esp_ipc_isr_stall_other_cpu();
    for (int waited = 0; waited < SYNC_WAIT_US && !REG_GET_BIT(CACHE_SYNC_CTRL_REG, CACHE_SYNC_DONE); waited++) {
        esp_rom_delay_us(1);
    }
    restart_guarded();
}

void safe_restart_hang(void)
{
    rv_utils_intr_global_disable();
    arm_guard();
    ESP_INFINITE_LOOP();
}
