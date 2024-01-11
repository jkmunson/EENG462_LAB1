//TIMERS
#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include "driverlib/rom.h"
#include "driverlib/map.h"
#include <stdint.h>

uint64_t get_uptime_cycles();
void timeKeeperISR ();
void configureDebounceTimer();

#define TIMER1_MULTIPLIER 32 //Number of times timer1 will overflow, and trigger it's interrupt each second
#define TIMER_CYCLES (SysCtlClockGet()/TIMER1_MULTIPLIER)

volatile int32_t uptime_seconds;
volatile uint64_t timer1_overflow_count;

#define TIMER_ISR_IS_PENDING (TIMER1_MIS_R & TIMER_ICR_TATOCINT)

void configureDebounceTimer(void) {

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; //Enable Run Mode Clock Gating Control for Timer 0

    while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R1)) {}

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
    TIMER1_CTL_R |= TIMER_CTL_TASTALL; //Stall for debug
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R |= TIMER_TAMR_TAMR_PERIOD; //Set Timer to count down periodically
    TIMER1_TAILR_R = TIMER_CYCLES-1;
    TIMER1_TAPR_R = 0;
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT; //Clear Interrupt
    TIMER1_IMR_R |= TIMER_IMR_TATOIM; //Enable Interrupt as Timeout
    NVIC_EN0_R = 1 << (INT_TIMER1A - 16);
    TIMER1_CTL_R |= TIMER_CTL_TAEN; //Enable Timer
}

void timeKeeperISR (void) {
    static char second_counter = 0;

    TIMER1_IMR_R &= ~TIMER_IMR_TATOIM; //Disable Interrupt
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT; //Clear Interrupt

    timer1_overflow_count++;

    //Every second
    if(++second_counter == TIMER1_MULTIPLIER) {
        uptime_seconds++;
        second_counter = 0;
    }

    TIMER1_IMR_R |= TIMER_IMR_TATOIM; //Enable Interrupt
}

uint64_t get_uptime_cycles(void) {
    uint64_t overflow_count_now;
    uint64_t cycles_now;

    do {
        if(TIMER_ISR_IS_PENDING) timeKeeperISR();
        overflow_count_now = timer1_overflow_count;
        cycles_now = TIMER_CYCLES - TIMER1_TAR_R;
    // If the counter overflowed during this code block, then our reads of uptime and cycles are invalid. Re-do them.
    } while (TIMER_ISR_IS_PENDING);

    return (TIMER_CYCLES * overflow_count_now) + cycles_now;
}

void configureAdcTimer (void) {

	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0; //Enable Run Mode Clock Gating Control for Timer 0

	while (!(SYSCTL_PRTIMER_R & SYSCTL_RCGCTIMER_R0)) {}

	TIMER0_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
	TIMER0_CTL_R |= TIMER_CTL_TASTALL; //Stall for debug
	TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;
	TIMER0_TAMR_R |= TIMER_TAMR_TAMR_PERIOD; //Set Timer to count down periodically
	TIMER0_TAILR_R = 16000 - 1;
	TIMER0_CTL_R |= TIMER_CTL_TAOTE; //Set as an ADC Trigger
	TIMER0_CTL_R |= TIMER_CTL_TAEN; //Enable Timer
}

//ADC

#define GPIO_PIN4 (1 << 4)
#define POT_TRIGGER_MARGIN 0xA

volatile uint16_t potReading;

void ADCPinConfigure(void) {

    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;                      //Enable ADC Clock
    while(!(SYSCTL_PRADC_R & SYSCTL_PRADC_R0)) {};              //Wait for peripheral to be ready

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;                    //Enable GPIO Pin for ADC (PE5)
    while(!(SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R4)) {};        //Wait fo peripheral to be ready

    GPIO_PORTE_AFSEL_R |= GPIO_PIN4;                            //Set Alternate Function Select
    GPIO_PORTE_DEN_R &= ~GPIO_PIN4;                             //Clear Digital Enable for Pin 5
    GPIO_PORTE_AMSEL_R |= GPIO_PIN4;                            //Set Alternate Mode Select

}

void ADCSampleSequencerConfigure(void) {

    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                           //Disable Sequencer 3
    ADC0_EMUX_R |= ADC_EMUX_EM3_TIMER;                          //Set ADC as Timer Triggered
    ADC0_SSMUX3_R |= 0x9;                                       //Enable AIN9
    ADC0_SSCTL3_R |= ADC_SSCTL3_IE0 | ADC_SSCTL3_END0;          //Sequencer control
    ADC0_SAC_R = 0x6;                                           //Enables x64 Oversampling
    ADC0_ISC_R |= ADC_ISC_IN3;                                  //Clear Interrupt
    ADC0_IM_R |= ADC_IM_MASK3;                                  //Enable Interrupt
    NVIC_EN0_R |= 1 << (INT_ADC0SS3 - 16);                      //Enable NVIC for ADC0 Sequencer 3

    configureAdcTimer();

    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                            //Enable Sequencer

}

void saveADCSample(void){

    ADC0_IM_R &= ~ADC_IM_MASK3;                                 //Disable Interrupt
    ADC0_ISC_R |= ADC_ISC_IN3;                                  //Clear Interrupt

    potReading = (ADC0_SSFIFO3_R & ADC_SSFIFO3_DATA_M);         //Read Potentiometer Value

    ADC0_IM_R |= ADC_IM_MASK3;                                  //Enable Interrupt

}

void main(void) {
    configureDebounceTimer();

    IntRegister(INT_TIMER1A, timeKeeperISR);

    IntRegister(INT_ADC0SS3, saveADCSample);

    ADCPinConfigure();
    ADCSampleSequencerConfigure();

    configureDebounceTimer();

    int32_t last_time = 0;

    while(1) {
        if((uptime_seconds - last_time) < (potReading/409)){
        //Turn LED on
        } else {
        //Turn LED off
        }

        if(uptime_seconds - last_time) >= 10) last_time = uptime_seconds;
    }
}
