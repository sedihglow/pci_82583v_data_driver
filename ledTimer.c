/*******************************************************************************
 * filename: ledTimer.c
 *
 * Handles the timer functionality for pciLED driver module
 *
 * Written by: James Ross
 ******************************************************************************/

#include "ledTimer.h"

/* blinks-per-second */ 
static int blinkRate_g = DFT_BLRATE; 
module_param(blinkRate_g, int, S_IRUSR | S_IWUSR); 

                /* static function declarations */
static inline void gbe38v_update_timer(int *blinkTime, unsigned long *jiff)
{
    *blinkTime = blinkRate_g; 
    *jiff      = jiffies;
}/* end gbe38v_update_timer */

static void gbe38v_toggle_led(void __iomem *ledReg)
{
    unsigned long regVal;
    unsigned long toWrite;

    /* read the current value of the LED control register, set toWrite */
    regVal = ioread32(ledReg);
    
    /* write the appropriate bits to toggle LED */
    if(regVal == LED0_ON){
        toWrite = regVal & ~LED0_ON; /* clear LED_ON bits */
        toWrite |= LED0_OFF;
        iowrite32(toWrite, ledReg);
    }
    else{ /* LED_OFF */
        toWrite = regVal & ~LED0_OFF; /* clear LED_OFF bits */
        toWrite |= LED0_ON;
        iowrite32(toWrite, ledReg);
    }
}/* end gbe38v_toggle_led */

static void gbe38v_openBlink_cb(unsigned long data)
{
    struct gbe38v_timer *timer = (struct gbe38v_timer*)data;
    printk(KERN_INFO ":in cb\n"); 
    /* toggle the led */
    gbe38v_toggle_led(timer->ledCtlReg);

    gbe38v_update_timer(&(timer->blinkTime), &(timer->jiff));
    
    mod_timer(&(timer->ktimer), (jiffies + (timer->blinkTime * HZ)));
}/* end gbe38v_timer_ledToggle */

                /* header function declarations */
int gbe38v_init_led_timer(void __iomem *ledCtlReg, struct gbe38v_timer *ledTimer)
{
    gbe38v_update_timer(&(ledTimer->blinkTime), &(ledTimer->jiff));
    setup_timer(&(ledTimer->ktimer), gbe38v_openBlink_cb, (unsigned long)ledTimer);

    /* set timerData and ensure LED register is set to LED_OFF */
    ledTimer->ledCtlReg = ledCtlReg;
    iowrite32(LED0_OFF, ledCtlReg);

    /* if param gets set to 0, do not blink, do not start timer. */
    if(unlikely(blinkRate_g == 0)){
        printk(KERN_WARNING "pciLED: Blink rate has been set to 0, no blink\n");
        return SUCCESS;
    }
    else if(unlikely(blinkRate_g < 0)){
        printk(KERN_WARNING "pciLED: Blink rate paramater changed to negetive,"
               "Invalid parameter.\n");
        return -EINVAL;
    }

    mod_timer(&(ledTimer->ktimer), (jiffies + ((ledTimer->blinkTime) * HZ)));
    return SUCCESS;
}/* end gbe38v_init_timer */

void gbe38v_remove_led_timer(struct gbe38v_timer *ledTimer)
{
    iowrite32(LED0_OFF, ledTimer->ledCtlReg);
    del_timer_sync(&(ledTimer->ktimer));
}/* end gbe38v_remove_led_timer */

int gbe38v_timer_blink_rate(void)
{
    return blinkRate_g;
}/* end get_blink_rate */

void gbe38v_set_timer_blink_rate(int blinkRate)
{
    blinkRate_g = blinkRate;
}/* end gbe38v_set_timer_blink_rate */

/************** EOF ***************/
