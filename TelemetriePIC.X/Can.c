/* 
 * File:   Can.c
 * Author: Francelin ANDRE
 *
 * Created on August 11, 2016, 17:58 PM
 * Last modified : October 10, 2016, 3:41 PM
 * 
 * ECAN functions for PIC18
 * Compiler : XC8
 * 
 */

#include "Can.h"
#include <stdio.h>
#include <stdlib.h>
#include <p18f26k80.h>

// ECAN initialize (Baud rate)
// Entrees :
//      - SJW : see Microchip documentation for more information
//      - BRP : see Microchip documentation for more information
//      - PHSEG1 : see Microchip documentation for more information
//      - PHSEG2 : see Microchip documentation for more information
//      - PROPSEG : see Microchip documentation for more information
// Exits :
void ECANInitialize (unsigned char SJW, unsigned char BRP, unsigned char PHSEG1, 
                     unsigned char PHSEG2, unsigned char PROPSEG)
{
    // Set module in configuration mode
    CANCON = CONFIGURATION_MODE;
    while (!(CANSTAT&CONFIGURATION_MODE));
    
    // Select ECAN mode
    ECANCON = ECANCON | LEGACY_MODE;
    
    // CIOCON register
    CIOCONbits.ENDRHI = 1;  // CANTX pin will drive VDD when recessive
    CIOCONbits.CANCAP = 1;  // Enable CAN capture
    
    // Synchronized Jump Width
    BRGCON1bits.SJW1 = SJW & 0x0002;
    BRGCON1bits.SJW0 = SJW & 0x0001;
    
    // Baud Rate Prescaler bits
    BRGCON1bits.BRP5 = BRP & 0x0020;
    BRGCON1bits.BRP4 = BRP & 0x0010;
    BRGCON1bits.BRP3 = BRP & 0x0008;
    BRGCON1bits.BRP2 = BRP & 0x0004;
    BRGCON1bits.BRP1 = BRP & 0x0002;
    BRGCON1bits.BRP0 = BRP & 0x0001;
    
    // Phase segment 1
    BRGCON2bits.SEG1PH2 = PHSEG1 & 0x0004;
    BRGCON2bits.SEG1PH1 = PHSEG1 & 0x0002;
    BRGCON2bits.SEG1PH0 = PHSEG1 & 0x0001;
    
    // freely programmable SEG2PH
    BRGCON2bits.SEG2PHTS =  1; 
    
    // Phase segment 2
    BRGCON3bits.SEG2PH2 = PHSEG2 & 0x0004;
    BRGCON3bits.SEG2PH1 = PHSEG2 & 0x0002;
    BRGCON3bits.SEG2PH0 = PHSEG2 & 0x0001;
    
    // Propagation time select
    BRGCON2bits.PRSEG2 = PROPSEG & 0x0004;
    BRGCON2bits.PRSEG1 = PROPSEG & 0x0002;
    BRGCON2bits.PRSEG0 = PROPSEG & 0x0001;
    
    // Receive buffer
    RXB0CONbits.RXFUL = 0;    // Reset flag
    RXB0CONbits.RXM1 = 0;   // Receive valid messages with standard identifier
    RXB0CONbits.RXM0 = 1;
    
    // Set mask 
    RXM0SIDH = 0;
    RXM0SIDL = 0;
    
    // Set module in normal mode
    CANCON = NORMAL_MODE;
    while ((CANSTAT&MASK_MODE) != NORMAL_MODE);
    
    //CANCON = LOOPBACK_MODE;
    //while ((CANSTAT&MASK_MODE) != LOOPBACK_MODE);
}

// ECAN receive function for RX buffer #0. 
// Entrees :
// Exits :
//      - id : identifier from source 
//      - data : buffer (0 ... 8)
//      - dataLen : size of buffer (0 ... 8)
//      - messageFlags : reserved, set NULL
//      - return : errorCode, not used
unsigned short ECAN0Read (unsigned short *id, char data[], 
                          unsigned char *dataLen, unsigned char *messageFlags)
{
    unsigned short errorCode = 0;   // Return code
    unsigned char bufferIndex;  // For loop
        
    // Get identifier
    *id = RXB0SIDH<<3;
    *id += RXB0SIDL>>5;

    // Get data length
    *dataLen = RXB0DLC&0x000F;

    // Get data into buffer
    // Load data into buffers
    for (bufferIndex=0; bufferIndex<*dataLen; bufferIndex++)
    {
        switch (bufferIndex)
        {
            case 0:
                data[0] = RXB0D0;
                break;   
            case 1:
                data[1] = RXB0D1;
                break;
            case 2:
                data[2] = RXB0D2;
                break;
            case 3:
                data[3] = RXB0D3;
                break;
            case 4:
                data[4] = RXB0D4;
                break;
            case 5:
                data[5] = RXB0D5;
                break;
            case 6:
                data[6] = RXB0D6;
                break;
            case 7:
                data[7] = RXB0D7;
                break;
            default:
                errorCode = -1;
                break;
        }    
    }
        
    // Reset flags
    RXB0CONbits.RXFUL = 0;
    PIR5bits.RXB0IF = 0;
    
    return errorCode;
}

// ECAN transmit function for TX buffer #0.
// Entrees :
//      - id : identifier from source 
//      - data : buffer (0 ... 8)
//      - dataLen : size of buffer (0 ... 8)
//      - messageFlags : set priotity from <Can.h> header
// Exits :
//      - return : errorCode, not used.
unsigned short ECAN0Write (unsigned short id, char data[], 
                           unsigned char dataLen, unsigned char messageFlags)
{
    unsigned short errorCode = 0;   // Return code
    unsigned char bufferIndex;  // For loop
        
    if (!TXB0CONbits.TXREQ)
    {
        // Load data into buffers
        for (bufferIndex=0; bufferIndex<dataLen; bufferIndex++)
        {
            switch (bufferIndex)
            {
                case 0:
                    TXB0D0 = data[0];
                    break;   
                case 1:
                    TXB0D1 = data[1];
                    break;
                case 2:
                    TXB0D2 = data[2];
                    break;
                case 3:
                    TXB0D3 = data[3];
                    break;
                case 4:
                    TXB0D4 = data[4];
                    break;
                case 5:
                    TXB0D5 = data[5];
                    break;
                case 6:
                    TXB0D6 = data[6];
                    break;
                case 7:
                    TXB0D7 = data[7];
                    break;
                default:
                    errorCode = -1;
                    break;
            }
        }    

        // Set identifier
        TXB0SIDLbits.EXIDE = 0; // Standard ID
        TXB0SIDH = (id&0x07F8)>>3;
        TXB0SIDL = (id&0x0007)<<5;

        // Set data length  
        TXB0DLCbits.DLC3 = (dataLen&0x0008)>>3;
        TXB0DLCbits.DLC2 = (dataLen&0x0004)>>2;
        TXB0DLCbits.DLC1 = (dataLen&0x0002)>>1;
        TXB0DLCbits.DLC0 = (dataLen&0x0001);
        
        TXB0DLCbits.TXRTR = 0;  // TXRTR cleared

        // Set priority
        TXB0CON = TXB0CON | messageFlags;   

        TXB0CONbits.TXREQ = 1;  // Request transmission
    }
    else
    {
        errorCode = -1;
    }

    return errorCode;
}

// Select and set mask to RX0 buffer
// Entrees :
//      - value : mask register value
//      - configurationFlags : optional configuration flags 
//        (see : Receive buffer mode)
void ECAN0SetMask (short value, unsigned char configurationFlags)
{
    // Set module in configuration mode
    CANCON = CONFIGURATION_MODE;
    while (!(CANSTAT&CONFIGURATION_MODE));
    
    // Set mask 
    RXM0SIDH = (value&0x07F8)>>3;
    RXM0SIDL = (value&0x0007)<<5;
    
    // Set configurationFlags
    RXB0CON = RXB0CON | configurationFlags;
    
    // Set module in normal mode
    CANCON = NORMAL_MODE;
    while ((CANSTAT&MASK_MODE) != NORMAL_MODE);
              
}
// Select and set filter to RX0 buffer
// Entrees :
//      - filterNumber : ECAN module filter number 
//        (only ECAN_FILTER_0 and ECAN_FILTER_1)
//      - value : filter register value
void ECAN0SetFilter (unsigned char filterNumber, short value)
{
    // Set module in configuration mode
    CANCON = CONFIGURATION_MODE;
    while (!(CANSTAT&CONFIGURATION_MODE));
    
    // Set mask
    switch (filterNumber)
    {
        case ECAN_FILTER_1:
            RXF1SIDH = (value&0x07F8)>>3;
            RXF1SIDL = (value&0x0007)<<5;
            
            // Filter will only accept standard ID messages
            RXF1SIDLbits.EXIDEN = 0;
            break;
            
        default:
            RXF0SIDH = (value&0x07F8)>>3;
            RXF0SIDL = (value&0x0007)<<5;
            
            // Filter will only accept standard ID messages
            RXF0SIDLbits.EXIDEN = 0;
            break;
    }
    
    // Set module in normal mode
    CANCON = NORMAL_MODE;
    while ((CANSTAT&MASK_MODE) != NORMAL_MODE);
}