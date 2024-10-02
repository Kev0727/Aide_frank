/*
 * rcc.c
 *
 *  Created on: Sep 13, 2024
 *      Author: ar56190@ens.ad.etsmtl.ca
 */

#include "rcc.h"

void init_RCC(void){

    // Activation du HSE
    RCC->CR |= RCC_CR_HSEON; 
    // Selection source HSE
    RCC->CFGR |= RCC_CFGR_PLLSRC_1; 
    // PLLMUL = 6
    RCC->CFGR |= RCC_CFGR_PLLMUL6;  
    // Activation du PLL
    RCC->CR |= RCC_CR_PLLON; 
    // Attente du PLL      
    while(!(RCC->CR & RCC_CR_PLLRDY)); 
    // SW = PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;   
}
