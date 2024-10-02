/****************************Proto 1********************************************
 Author : Bruno De Kelper et Alexandre Ceriani
 Date   : 10-06-2016
 Description :
 GPIO : -User button (PB0 et PB1)
        -Reset button (NRST)
        -LED bleu (PC8) et verte (PC9)
        -Led externe au board (PC0-PC7)
        -I2C1 SDA (PB7) et SCL (PB6)
        -ADC (PA4 et PA5)
        -Bit Direction_FB (PA6 et PA7)
        -Bit Direction Gauche (LSB : PB12 MSB : PB13)
        -Bit Direction Droit (LSB : PB14 MSB : PB15)
        -Bit calibration (PA8)
        -PWM sur timer 3 channel 1 (PB4) et 2 (PB5)
        -USART2 RX (PA2) et TX (PA3)
*/

#include <stdio.h> /*sprintf*/
#include "main.h"

// Fr�quence des Ticks du SysTick (en Hz)
#define MillisecondsIT ((uint32_t) 1000)

/*Fonctions main*/
void Configure_Clock(void);

void SysTick_Handler(void) {

}

int main(void){

	// Configure les composantes du robot
	__set_PRIMASK(1);
    Configure_Clock();
    /*Initialisation des p�riph�riques*/
    configuration_USART();
	__set_PRIMASK(0);
	volatile uint32_t counter = 0;
    while (1) {
    	/*Boucle principale du programme*/
    	state_machine();
    	counter++;
    }

    return (0);
}

__INLINE void Configure_Clock(void){
	/*Il faut ajouter le code pour l'initialisation du RCC*/

	init_RCC();
    SystemCoreClockUpdate();	// Met � jour SystemCoreClock avec la config du RCC
    SysTick_Config(SystemCoreClock/MillisecondsIT);	// Configure le SysTick � 1 ms
}

/*End of file*/
