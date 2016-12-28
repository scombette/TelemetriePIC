/******************************************************************************
 *                          TelemetriePIC/main.h
 *  Fichier header du main du programme de la carte telemetrie
 *  
 *  Config : PIC18F26K80, liaison CAN avec MCP 2551, RS232 avec Radio et GPS, 
 *  SPI avec carte SD
 * 
 * ***************************************************************************/

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

/*********************************** Ports ***********************************/
#define LED LATAbits.LATA0 
    
/********************************** Valeurs **********************************/

    
/********************************** Labels ***********************************/
#define IT_TMR0 INTCONbits.TMR0IF 
#define IT_RX2  PIR3bits.RC2IF
    
/******************************** Structures *********************************/   

typedef struct DataGPS DataGPS;
struct DataGPS
{
    int latitude;
    int longitude;
};

typedef struct DataMot DataMot;
struct DataMot
{
    float   vitesse;
    int     vitesseMoteur;
    float   tempMoteur;
    float   tempAir;
};

#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

