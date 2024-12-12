#include "sdkconfig.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"
#include "soc/rtc_cntl_reg.h"

     .bss

/* Stato precedente del segnale */
    .global prev_state
prev_state:
    .long 0

/* Numero del GPIO monitorato */
    .global gpio_number
gpio_number:
    .long 0

/* Contatore debounce */
    .global debounce_counter
debounce_counter:
    .long 0

/* Valore massimo del contatore debounce */
    .global debounce_max_count
debounce_max_count:
    .long 0

    .text
    .global entry
entry:

    /* Read and mask GPIO 9 */
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 9, 16)
    and r0, r0, 1

    /* Confronta con lo stato precedente */
    move r3, prev_state
    ld r3, r3, 0
    sub r1, r0, r3

    jump no_change, eq

    /* Se cambia, decrementa il debounce_counter */
	move r3, debounce_counter
    ld r2, r3, 0
    sub r2, r2, 1
    jump debounce_done, eq

    /* Memorizza il nuovo valore del contatore */
    st r2, r3, 0
    jump entry

debounce_done:
    /* Conferma il cambio di stato */
    move r3, prev_state
    st r0, r3, 0 /* Aggiorna lo stato precedente */

    /* Resetta il contatore debounce */
    move r3, debounce_max_count
    move r2, debounce_counter
    ld r3, r3, 0
    st r3, r2, 0

    /* Sveglia la CPU */
wake_up:
	/* Check if the system is in sleep mode */
	READ_RTC_REG(RTC_CNTL_LOW_POWER_ST_REG, 27, 1)
	move r1, r0

	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	/* If the system is in normal mode or if the system is in sleep mode with ready for wakeup set, we can signal the main CPU to wakeup */
	or r0, r0, r1
	jump wake_up, eq

    wake
    jump entry

no_change:
    /* Reset debounce_counter se non cambia */
    move r3, debounce_max_count
    move r2, debounce_counter
    ld r3, r3, 0
    st r3, r2, 0
    jump entry
