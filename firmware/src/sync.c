/* sync.c -- bus locks and the timer callbacks. */
#include "app.h"

/* The I2C bus is shared by the image sensors, the FPGA, the temperature sensor and the light
 * driver (selected with i2c_select_target()); the SPI flash by the update and user-page paths. */
void i2c_lock(void)     { CyU3PSemaphoreGet(&i2c_sem, CYU3P_WAIT_FOREVER); }
void i2c_unlock(void)   { CyU3PSemaphorePut(&i2c_sem); }
void flash_lock(void)   { CyU3PSemaphoreGet(&flash_sem, CYU3P_WAIT_FOREVER); }
void flash_unlock(void) { CyU3PSemaphorePut(&flash_sem); }

/* Callback of emc_wdg_timer, meant to count seconds (paused during an update) for emc_wdg_thread().
 * The timer is created without activation and never started, so this never runs and the count
 * stays 0. */
void emc_wdg_timer_cb(uint32_t arg)
{
    (void)arg;  /* timer callback argument, unused */
    if (app_state.update_active != 0)
        return;
    emc_wdg_progress++;
}

/* update_timer (2 s, restarted for every update page): an update that stalls is abandoned, but
 * the page and byte counters in update.c are not reset (see README, update retry bug). */
void update_timer_cb(void)
{
    app_state.update_active = 0;
    CyU3PTimerStop(&update_timer);
}
