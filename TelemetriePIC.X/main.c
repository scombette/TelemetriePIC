/******************************************************************************
 *                          TelemetriePIC/main.c
 *  Fichier main du programme de la carte telemetrie
 *  
 *  Config : PIC18F26K80, liaison CAN avec MCP 2551, RS232 avec Radio et GPS, 
 *  SPI avec carte SD
 * 
 *  Brief : Ce programme gere la communication avec la telemetrie et les logs
 * 
 * ***************************************************************************/


/******************************** Includes ***********************************/
#include <p18f26k80.h>
#include <timers.h>
#include "main.h"
//#include <ECANPoll.h>

/********************************* PRAGMA ************************************/
#pragma config  RETEN       = OFF
#pragma config  XINST       = OFF
#pragma config  FOSC        = INTIO2 
#pragma config  PLLCFG      = OFF
#pragma config  FCMEN       = OFF
#pragma config  PWRTEN      = ON
#pragma config  BOREN       = OFF
#pragma config  WDTEN       = OFF
#pragma config  CANMX       = PORTB
#pragma config  MCLRE       = ON
#pragma config  STVREN      = ON

/******************************* Prototypes **********************************/
void interruptions(void);
void tmr0config(void);


/******************************** Variables **********************************/


/******************************** Fonctions **********************************/

/* Gestion des interruptions */
#pragma code high_vector = 0x08

void high_interrupt(void) {
    _asm goto interruptions _endasm
}
#pragma code

#pragma interrupt interruptions
/****************************** interruptions ********************************
 *  @Brief  : Cette fonction gere les differentes interruptions :
 *            - TIMER0  : Envoie la trame
 *            - CAN     : Stocke les donnees reçues
 *            - RX1     : 
 *            - RX2     : 
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void interruptions(void) {
    
    /* Interruption du TIMER0 */
    if (IT_TMR0) {
        IT_TMR0 = 0;
        LED = !LED;
    }
    
}
/*********************************** main *************************************
 *  @Brief  : Boucle principale du programme : configure les peripheriques et 
 *            tourne en boucle.
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void main(void) 
{
    TRISAbits.TRISA0 = 0;
    LED = 0;
    tmr0config();
    
    while(1);
}

/******************************** tmr0config **********************************
 *  @Brief  : Cette fonction configure le timer0 sur ~500ms et active les IT
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void tmr0config(void)
{ 
    IT_TMR0 = 0;
	INTCONbits.TMR0IE = 1;
	INTCONbits.GIEH = 1;
	T0CON = 0x83;
}

