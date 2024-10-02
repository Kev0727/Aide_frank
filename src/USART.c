#include "USART.h"

// Priorite du USART
#define USART2_PRIORITY (uint32_t) 35
// Taille du buffer
#define BUFFET_TAILLE 8

// Structure pour la state machine
typedef enum {
    COMMANDE,
    VITESSE,
    ANGLE,
} enum_state_t;

typedef struct {
    uint16_t commande; 
    uint16_t vitesse;
    uint16_t angle;
} state_mach_t; 

// Declaration d'une variable temporaire pour transmettre et recevoir le data
volatile uint8_t temp_data = 0;
static   uint8_t* data_input;

uint8_t valeur_commande = 0;
uint8_t valeur_vitesse = 0;
uint8_t valeur_angle = 0;

static enum_state_t state_mach = COMMANDE;
state_mach_t enregistrement_data;

// Creation d'un buffer pour la state machine
BUFFER_NEW(buffer_input,BUFFET_TAILLE);

BUFFER_NEW(buffer_output,BUFFET_TAILLE);



void configuration_USART(void){
   
   /* CONFIGURATION DES PORTS */

   // Activation de l'horloge A 
   RCC-> AHBENR    |= RCC_AHBENR_GPIOAEN; 

   // Configuration du mode des Pins du Port A
   GPIOA->MODER   |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
   GPIOA->MODER   &= ~(GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0); 

   // Configuration du type de sortie des Pins du Port A
   GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_2;
   GPIOA->OTYPER  |= GPIO_OTYPER_OT_3;
   
   // Configuration pull-up/pull-down des Pins du Port A
   GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR3_1);
   GPIOA->PUPDR   |= (GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0);

   // Configuration de la vitesse des Pins du Port A
   GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEEDR2 | GPIO_OSPEEDR_OSPEEDR3);

   // Configuration de la fonction alternative des Pins du Port A
   GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFR2 | GPIO_AFRL_AFR3);
   GPIOA->AFR[0]  |= ((((uint32_t) 0x1) << (4*2)) | (((uint32_t) 0x1) << (4*3)));

   /*  PROTOCOLE DE COMMUNICATION DU USART */

   // Active l'horloge du USART2
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
   RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST; // Reset
   RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST; // Unreset

   // Configuration du protocole (9600 Baud)
   USART2 ->CR1 &= ~(USART_CR1_OVER8 | USART_CR1_M | USART_CR1_PCE);
   USART2 ->CR2 &= ~USART_CR2_STOP;
   USART2 ->BRR = (uint16_t)(48000000 / 9600);

   // Activation de USART2
   USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);
   
   // Activation des interruptions
   USART2->CR1 |= (USART_CR1_TXEIE | USART_CR1_RXNEIE);
   USART2->CR3 |= (USART_CR3_EIE);

   // Configuration des interruptions
   NVIC->ISER[0] = (((uint32_t) 1) << (USART2_IRQn & 0x1F));
   NVIC->IP[(USART2_IRQn >> 2)] &= ~(0xFF << ((USART2_IRQn & 0x03) << 3)); // Reset priority
   NVIC->IP[(USART2_IRQn >> 2)] |= (USART2_PRIORITY << ((USART2_IRQn & 0x03) << 3)); // Set priority

}


/* 
    Il faudrait voir si on peut le mettre dans un autre fichier a la place pour ne pas avoir trop de fonction
    dans un fichier.
    Update : nvm on peut le garder 
*/
void state_machine(void){
    
    // Sort la donner recu en reception
    buffer_pull(&buffer_input, &data_input);

    // Debut de la state machine qui commence a l'etape COMMANDE
    switch (state_mach)
    {
    case COMMANDE : {
        // Verififcation du data en entree 
        if(data_input == 0xf1){
            // Enregistre une copie du data commande dans un struc 
            enregistrement_data.commande = data_input;
            valeur_commande = data_input;
            // Renvoie la valeur de la commande dans le buffer
            buffer_push(&buffer_output, valeur_commande);
            // Lorsque l'operation est fini, on peut passer au prochain etat
            state_mach = VITESSE;
        }
        //verification du data en mode ARRET
        else if (data_input == 0xF0){ 
            // Enregistre une copie du data commande dans un struc 
            enregistrement_data.commande = data_input;
            enregistrement_data.vitesse = 0; 
            enregistrement_data.angle = 0;

            valeur_commande = data_input;
            valeur_vitesse = 0;
            valeur_vitesse = 0;

            // Renvoie la valeur de la vitesse dans le buffer
            buffer_push(&buffer_input, valeur_commande);
            buffer_push(&buffer_input, valeur_vitesse);
            buffer_push(&buffer_input, valeur_angle);

            // Lorsque l'operation est fini, on peut passer au prochain etat
            state_mach = COMMANDE;
        }   
        //Si la donner est non-valide, retour a l'etat commande 
        else 
            state_mach = COMMANDE;
        

    }
    break;
    case VITESSE : {
        if (data_input >= 0 && data_input <= 200){
            // Enregistre une copie du data vitesse dans un struc 
            enregistrement_data.vitesse = data_input-(uint8_t)100;
            valeur_vitesse = data_input;

            // Renvoie la valeur de la vitesse dans le buffer
            buffer_push(&buffer_output, valeur_vitesse);

            // Lorsque l'operation est fini, on peut passer au prochain etat
            state_mach = ANGLE;
        }
        //Si la donner est non-valide, retour a l'etat commande 
        else
            state_mach = COMMANDE;
    }
    break;
    case ANGLE : {
        if (data_input >= 0 && data_input <= 200){
            // Enregistre une copie du data vitesse dans un struc 
            enregistrement_data.vitesse = data_input-(uint8_t)100;
            valeur_vitesse = data_input;

            // Renvoie la valeur de la vitesse dans le buffer
            buffer_push(&buffer_output, valeur_vitesse);

            // Lorsque l'operation est fini, on peut passer au prochain etat
            state_mach = COMMANDE;
        }
        //Si la donner est non-valide, retour a l'etat commande 
        else 
            state_mach = COMMANDE;
    }   
    break; 
    }
    
}

/*
    Alex va voir la partie 2 du document laboratoire 2, sa explique a moitie ce qu'on doit faire
*/

void USART2_IRQHandler(void){

    // Message recu du flag de reception
    if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE){
        buffer_push(&buffer_input,(uint8_t)(USART2->RDR));
    }
    // Message a transmettre du flag de transmission
    if((USART2->ISR & USART_ISR_TXE) == USART_ISR_TXE){
       //Verifier que le buffer a au moins 3 donner dedans avant de retransmettre l'echo
       if (buffer_count(&buffer_output) >= 3){
            buffer_pull(&buffer_output, &temp_data);
            USART2->TDR = temp_data;
       }
       else //Si aucune donner est valide, activation du flag
            USART2->ICR |= USART_ICR_TCCF;
    }
    // S'il y a une erreur de transmission on recommence
    if ((USART2->ISR & USART_ISR_ORE) == USART_ISR_ORE){
        USART2->ICR |= USART_ICR_ORECF;
    }
}

