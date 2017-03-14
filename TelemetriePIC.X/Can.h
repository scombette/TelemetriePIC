/* 
 * File:   Can.h
 * Author: Administrateur
 *
 * Created on August 11, 2016, 5:58 PM
 * Last modified : October 10, 2016, 3:41 PM
 * 
 * ECAN functions for PIC18
 * Compiler : C18
 * 
 */

#ifndef CAN_H
#define	CAN_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// Module indentification number
#define ENGINE_MODULE       0x0016
#define CARTE_VOLANT        0x0002
#define TELEMETRY_MODULE    0x0004
#define SNIFFER_MODULE      0x0008

// Protocol mode
#define LEGACY_MODE             0x00  // Mode0
#define ENHANCED_LEGACY_MODE    0x40  // Mode1
#define ENHANCED_FIFO_MODE      0x80  // Mode2
    
// ECAN mode
#define MASK_MODE           0xE0
#define CONFIGURATION_MODE  0x80
#define LISTEN_ONLY_MODE    0x60
#define LOOPBACK_MODE       0x40
#define DISABLE_MODE        0x20
#define NORMAL_MODE         0x00

// RX message flags
#define ECAN_RX0_FILTER_0   0x00    // Filter0 match (RX buffer0 only)
#define ECAN_RX0_FILTER_1   0x01    // Filter1 match (RX buffer0 only)
    
#define ECAN_RX1_FILTER_2   0x02    // Filter2 match (RX buffer1 only)
#define ECAN_RX1_FILTER_3   0x03    // Filter3 match (RX buffer1 only)
#define ECAN_RX1_FILTER_4   0x04    // Filter4 match (RX buffer1 only)
#define ECAN_RX1_FILTER_5   0x05    // Filter5 match (RX buffer1 only)
    
// RX Masks
#define ECAN_MASK_0 0x00
#define ECAN_MASK_1 0x01
    
// RX filters
#define ECAN_FILTER_0   0x00
#define ECAN_FILTER_1   0x01
#define ECAN_FILTER_2   0x02
#define ECAN_FILTER_3   0x03
#define ECAN_FILTER_4   0x04
#define ECAN_FILTER_5   0x05
    
// Receive buffer mode
#define ALL_MESSAGE     0x60    // All messages
#define ONLY_EXTENDED   0x40    // Only valid messages with extended identifier
#define ONLY_STANDARD   0x20    // Only valid messages with standard identifier
#define VALID_MESSAGE   0x00    // All valid messages
   
// TX message flags    
#define ECAN_TX_PRIORITY_0      0x00    // Lowest priority
#define ECAN_TX_PRIORITY_1      0x01    // ...
#define ECAN_TX_PRIORITY_2      0x02    // ...
#define ECAN_TX_PRIORITY_3      0x03    // Highest priority

// Functions prototypes
void ECANInitialize (unsigned char SJW, unsigned char BRP, unsigned char PHSEG1, 
                     unsigned char PHSEG2, unsigned char PROPSEG);
unsigned short ECAN0Read (unsigned short *id, char data[], 
                          unsigned char *dataLen, unsigned char *messageFlags);
unsigned short ECAN0Write (unsigned short id, char data[], 
                           unsigned char dataLen, unsigned char messageFlags);
void ECAN0SetMask (short value, unsigned char configurationFlags);
void ECAN0SetFilter (unsigned char filterNumber, short value);

#ifdef	__cplusplus
}
#endif

#endif	/* CAN_H */

