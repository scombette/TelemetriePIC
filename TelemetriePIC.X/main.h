/* 
 * File:   main.h
 * Author: Sébastien
 *
 * Created on 27 décembre 2016, 17:26
 */

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

