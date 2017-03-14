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
#define GPS_R_FRAME_LENGTH      67
#define RADIO_S_FRAME_LENGTH    16

/********************************** Labels ***********************************/
#define IT_TMR0 INTCONbits.TMR0IF 
#define IT_RADIO  PIR1bits.RC1IF
#define IT_GPS  PIR3bits.RC2IF
#define IT_CAN  PIR5bits.RXB0IF
    
/******************************** Structures *********************************/   

typedef struct DataGPS DataGPS;
struct DataGPS
{
    char heure;
    char minutes;
    char secondes;
    char latitude_deg;
    char latitude_min;
    char latitude_cent_min;
    char latitude_dixmil_min;
    char latitude_card;
    char longitude_deg;
    char longitude_min;
    char longitude_sec;
    char longitude_cent_min;
    char longitude_dixmil_min;
    char longitude_card;
    char altitudeH;
    char altitudeL;
    char nbSats;
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

