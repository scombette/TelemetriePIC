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

/*********************************** TODO ************************************
 *  @TODO : Ajout reception serie et IT associee
 *  @TODO : Ajout reception CAN et IT associee 
 * 
 */


/******************************** Includes ***********************************/
#include <p18f26k80.h>
#include <timers.h>
#include <usart.h>
#include "main.h"

/********************************* Config ************************************/
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
void generalConfig(void);
void tmr0config(void);
void usart2config(void);


/******************************** Variables **********************************/
DataMot     donneesMoteur;
DataGPS     donneesGPS; 

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
    if (IT_TMR0) 
    {
        /* Reset du flag d'interruption */
        IT_TMR0 = 0;
    }
    
    /* Interruption da la liaison serie USART2 (GPS) */
    else if(IT_RX2)
    {
        /* Reset du flag d'interruption */
        IT_RX2 = 0;
        
        if(Read2USART() == 0b11001100)
        {
            LED = !LED;
        }
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
    /* Configurations */
    generalConfig();
    tmr0config();
    usart2config();
    LED = 0;
  
    /* Boucle principale */
    while(1);
}

/****************************** generalConfig ********************************
 *  @Brief  : Cette fonction configure les registres necessaires pour le bon 
 *            fonctionnement du programme (TRIS, INTCON, ...)
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void generalConfig(void)
{
    /* Configuration de la pin A0 en sortie TOR */
    TRISAbits.TRISA0 = 0;
    
    /* Activation des interruptions generales et des peripheriques */
	INTCONbits.GIEH = 1;
    INTCONbits.PEIE = 1;
    
}

/******************************** tmr0config **********************************
 *  @Brief  : Cette fonction configure le timer0 sur ~500ms et active les IT
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void tmr0config(void)
{ 
    /* Activation de l'interruption du timer0 */
	INTCONbits.TMR0IE = 1;
    
    /* Parametrage du registre de controle du timer0 */
	T0CON = TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_EDGE_RISE & T0_PS_1_16;
    
    /* Reset du flag d'interruption */
    IT_TMR0 = 0;
}

/******************************* usart1config *********************************
 *  @Brief  : Cette fonction configure la premiere liaison serie (Radio)
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void usart1config(void)
{
    /* Ouverture du port serie USART1*/
    Open1USART( USART_TX_INT_OFF    & USART_RX_INT_OFF      & USART_BRGH_HIGH 
                & USART_CONT_RX     & USART_EIGHT_BIT       & USART_ASYNCH_MODE 
                & USART_ADDEN_OFF   , BAUD_IDLE_CLK_HIGH    & BAUD_16_BIT_RATE 
                & BAUD_WAKEUP_OFF   & BAUD_AUTO_ON);
}

/******************************* usart2config *********************************
 *  @Brief  : Cette fonction configure la deuxieme liaison serie (GPS)
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void usart2config(void)
{
    /* Ouverture du port serie USART2*/
    Open2USART( USART_TX_INT_OFF    & USART_RX_INT_ON       & USART_BRGH_HIGH 
                & USART_CONT_RX     & USART_EIGHT_BIT       & USART_ASYNCH_MODE 
                & USART_ADDEN_OFF   , BAUD_IDLE_CLK_HIGH    & BAUD_16_BIT_RATE 
                & BAUD_WAKEUP_OFF   & BAUD_AUTO_ON);
    
    /* Reset du flag d'interruption */
    IT_RX2 = 0;
}
