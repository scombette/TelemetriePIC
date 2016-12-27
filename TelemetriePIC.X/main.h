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
#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

