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
 *  @TODO : Reception $GPGGA, OK --> Exploitation des resultats
 *  @TODO : Ajout reception CAN et IT associee 
 */


/******************************** Includes ***********************************/
#include <p18f26k80.h>
#include <timers.h>
#include <usart.h>
#include "main.h"
#include "Can.h"

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

/* Configurations */
void generalConfig(void);
void tmr0Config(void);
void usart1Config(void);
void usart2Config(void);
void canBusConfig(void);
void gpsConfig(void);
void canConfig();

/* Exécution */
void sendRXFrame(char* frame, short length, char port);
void gpsReceive(short* compteur, char* buffer);
void extractGPS(char* buffer, short length);
char asciiToByte(char toConvert);
short findNextVirgule(char* table, short begin, short length);

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
 *            - RX1     : Radio
 *            - RX2     : GPS
 *  @Params : Aucun 
 *  @Retval : Aucune
 *****************************************************************************/
void interruptions(void) {
    
    /* Compteur de réception d'octets venant du GPS */
    static short gpsRCompt = 0;
    
    /* Buffer de réception de la trame GPS */
    static char  gpsBuff[80];
    
    /* Interruption du TIMER0 */
    if (IT_TMR0) 
    {
        /* Reset du flag d'interruption */
        IT_TMR0 = 0;
    }
    
    /* Interruption reception sur la liaison serie USART1 (Radio) */
    else if(IT_RADIO)
    {
        /* On vide le buffer de reception */
        Read1USART();
    }
    
    /* Interruption reception sur la liaison serie USART2 (GPS) */
    else if(IT_GPS)
    {   
        /* Appel de la fonction qui gère la réception et le compteur */
        gpsReceive(&gpsRCompt, gpsBuff);

        /* Une fois la reception terminee */
        if(gpsRCompt > 12)
        {
           extractGPS(gpsBuff, gpsRCompt);
        }
    }
    
    /* Interruption liee a une reception sur le bus CAN */
    else if(IT_CAN)
    {
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
    /* Configurations */
    generalConfig();
    tmr0Config();
    usart2Config();
    usart1Config();
    canConfig();
    
    LED = 0;
    Delay10KTCYx(50);
    gpsConfig();
    
  
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
    
    /* Configuration des pins CANTX et CANRX en sortie et entree */
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 1;
    
    /* Activation des interruptions generales et des peripheriques */
	INTCONbits.GIEH = 1;
    INTCONbits.PEIE = 1;
    
}

/******************************** tmr0Config **********************************
 *  @Brief  : Cette fonction configure le timer0 sur ~500ms et active les IT
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void tmr0Config(void)
{ 
    /* Activation de l'interruption du timer0 */
	INTCONbits.TMR0IE = 1;
    
    /* Parametrage du registre de controle du timer0 */
	T0CON = TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_EDGE_RISE & T0_PS_1_16;
    
    /* Reset du flag d'interruption */
    IT_TMR0 = 0;
}

/******************************* usart1Config *********************************
 *  @Brief  : Cette fonction configure la premiere liaison serie (Radio)
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void usart1Config(void)
{
    /* Ouverture du port serie USART1*/
    Open1USART( USART_TX_INT_OFF    & USART_RX_INT_ON       & USART_BRGH_LOW 
                & USART_CONT_RX     & USART_EIGHT_BIT       & USART_ASYNCH_MODE,
               12);
   
    /* Reset du flag d'interruption */
    IT_RADIO = 0;
}

/******************************* usart2Config *********************************
 *  @Brief  : Cette fonction configure la deuxieme liaison serie (GPS)
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void usart2Config(void)
{
    /* Ouverture du port serie USART2*/
    Open2USART( USART_TX_INT_OFF    & USART_RX_INT_ON       & USART_BRGH_LOW 
                & USART_CONT_RX     & USART_EIGHT_BIT       & USART_ASYNCH_MODE,
               25);
   
    /* Reset du flag d'interruption */
    IT_GPS = 0;
}   

/******************************** canConfig ***********************************
 *  @Brief  : Cette fonction configure le module ECAN
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void canConfig()
{
    /* Initialisation du module ECAN avec configuration du baudrate */
    ECANInitialize (0b01, 0b000001, 0b101, 0b001, 0b110);
    /* Configuration du masque de reception */
    ECAN0SetMask (0xFF, ONLY_STANDARD);
    /* Configuration du filtre de reception : le buffer 0 recevra les messages 
     * du module moteur */
    ECAN0SetFilter (ECAN_FILTER_0, TELEMETRY_MODULE);
    
    /* Configuration des interruptions */
    PIE5bits.RXB0IE = 1;    
    RXB0CONbits.RXFUL = 0;    
    IT_CAN = 0;
}

/******************************** gpsConfig ***********************************
 *  @Brief  : Cette fonction configure le GPS pour qu'il n'envoie que des 
 *            trames GPGGA.
 *  @Params : Aucun
 *  @Retval : Aucune
 *****************************************************************************/
void gpsConfig(void)
{
    /* Desactivation du VTG */
    char frame[27] = {'$','P','S','R','F','1','0','3',',','0','5',',','0','0',
                      ',','0','0',',','0','1','*','2','1','\r','\n'};
    sendRXFrame(frame, 27, 2);
    
    /* Desactivation du RMC */
    frame[10] = '4';
    frame[22] = '0';
    sendRXFrame(frame, 25, 2);
    
    /* Desactivation du GSV */
    frame[10] = '3';
    frame[22] = '7';
    sendRXFrame(frame, 25, 2);
    
    /* Desactivation du GSA */
    frame[10] = '2';
    frame[22] = '6';
    sendRXFrame(frame, 25, 2);

    /* Configuration du GPGGA a 1Hz */
    frame[10] = '0';
    frame[16] = '1';
    frame[22] = '5';
    sendRXFrame(frame, 25, 2);
}

/******************************* sendRXFrame *********************************
 *  @Brief  : Cette fonction envoie une trame série sur la liaison série 1 ou 2
 *  @Params : - char* frame  : La trame à envoyer
 *            - short length : La longueur de la trame à envoyer
 *            - char  port   : sur quelle liaison envoyer (1 ou 2)
 *  @Retval : Aucune
 *****************************************************************************/
void sendRXFrame(char* frame, short length, char port)
{
    int i = 0;
    
    /* Tant qu'il reste des octets a envoyer on continue */
    for(i = 0; i < length; i++)
    {
        /* On envoie sur le port serie 1 ou 2 */
        switch(port)
        {
            case 1:  
                Write1USART(frame[i]);
                while(Busy1USART( ));
                break;
                
            case 2:   
                Write2USART(frame[i]);
                while(Busy2USART( ));
                break;
                
            default:
                break;
                
        }
    }
}

/******************************* gpsReceive **********************************
 *  @Brief  : Cette fonction gère la réception de messages du GPS, afin de ne 
 *            recuperer que les trames GPGGA.
 *  @Params : - short* compteur  : Le compteur de nombre d'octets reçus 
 *            - char* buffer : Le buffer de réception de la trame GPS
 *  @Retval : Aucune
 *****************************************************************************/
void gpsReceive(short* compteur, char* buffer)
{   
    static short cnt = 0;
    *compteur = 0;
    
    /* Si la trame n'a pas commence */
    if(cnt == 0)
    {
         /* Si on a un debut de trame NMEA (IE si on a un $) */
        if(Read2USART() == '$')
        {
            buffer[cnt] = '$';
            cnt ++;
        }
    }
    
    /* Si la reception de la trame a debute */
    else
    {
        /* On stocke l'octet reçu et on avance dans le buffer */
        buffer[cnt] = Read2USART();
        cnt ++;
        
        /* Si on arrive à la fin de la trame*/
        if(buffer[cnt - 1] == '\n' || cnt >= 80)
        {
            /* On decremente le compteur */
            cnt --;
            
            /* On verifie de bien avoir une trame GPGGA */
            if(buffer[0] == '$' && buffer[1] == 'G' && buffer[2] == 'P' && 
                    buffer[3] == 'G' && buffer[4] == 'G' && buffer[5] == 'A' && 
                    buffer[6] == ',')
            {
                /* On recupere le nombre d'octets lus */
                *compteur = cnt;
            }
                /* On vide le compteur de reception */
                cnt = 0;  
        }
    }
}

/******************************* extractGPS **********************************
 *  @Brief  : Cette fonction extrait les informations de la trame GPS
 *  @Params : - char* buffer : Le buffer de réception de la trame GPS
 *            - short length : La longueur du buffer de reception
 *  @Retval : Aucune
 *****************************************************************************/
void extractGPS(char* buffer, short length)
{
    /* Compteur pour parcourir le buffer */
    short cnt = 0;
    short nextVirgule = 0;
    
    char s[22];
    
    /* Variable de calcul du CRC*/
    char CRC = 0;
    
    /* On calcule le CRC du premier G jusqu'a l'etoile excluse */
    for(cnt = 1; cnt < length - 4; cnt ++)
    {
        CRC = (unsigned char) (CRC ^ buffer[cnt]);
    }
    
    /* Conversion des caracteres ASCII en valeur hexa*/
    buffer[length - 3] = asciiToByte(buffer[length - 3]);
    buffer[length - 2] = asciiToByte(buffer[length - 2]);

    /* Si le calcul de la checksum correspond a la valeur recue, on extrait */
    if(((buffer[length - 3] << 4) + buffer[length - 2]) == CRC)
    {
        /* Recuperation de l'heure (format hhmmss dans la trame) */
        donneesGPS.heure = asciiToByte(buffer[7]) * 10 + asciiToByte(buffer[8]);
        donneesGPS.minutes = asciiToByte(buffer[9]) * 10 + asciiToByte(buffer[10]);
        donneesGPS.secondes = asciiToByte(buffer[11]) * 10 + asciiToByte(buffer[12]);
        
        /************ Latitude ************/
        
        /* On se place au niveau de la virgule apres l'heure */
        cnt = findNextVirgule(buffer, 13, length);
        /* On cherche la virgule de fin de longitude */
        nextVirgule = findNextVirgule( buffer, cnt + 1, length);
        
        /* On verifie qu'il y ait une valeur */
        if(nextVirgule - cnt > 3)
        {
            cnt ++;
            donneesGPS.latitude_deg = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
            cnt += 2;
            donneesGPS.latitude_min = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
            cnt += 3;
            donneesGPS.latitude_cent_min = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
            cnt += 2;
            donneesGPS.latitude_dixmil_min = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
        }
        
        /* On stocke le point cardinal (N / S) directement en ASCII */
        donneesGPS.latitude_card = buffer[nextVirgule + 1];

        /************ Longitude ************/
        
         /* On se place au niveau de la virgule apres le cardinal */
        cnt = nextVirgule + 2;
        /* On cherche la virgule de fin de longitude */
        nextVirgule = findNextVirgule( buffer, nextVirgule + 3, length);
        
        /* On verifie qu'il y ait une valeur */
        if(nextVirgule - cnt > 3)
        {
            /* On tronque la valeur de la longitude a +/- 64 */
            cnt += 2;
            donneesGPS.longitude_deg = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
            cnt += 2;
            donneesGPS.longitude_min = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
            cnt += 3;
            donneesGPS.longitude_cent_min = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
            cnt += 2;
            donneesGPS.longitude_dixmil_min = asciiToByte(buffer[cnt]) * 10 + asciiToByte(buffer[cnt + 1]);
            cnt = nextVirgule + 1;
        }
        
        /* On stocke le point cardinal (W / E) directement en ASCII */
        donneesGPS.longitude_card = buffer[cnt];
        cnt = findNextVirgule(buffer, cnt, length) + 1;
        
        /* On verifie que le fix est correct */
        if(buffer[cnt] == 0x31)
        {
            /* On recupere le nombre de GPS disponibles */
            cnt = findNextVirgule(buffer, cnt, length) + 1;
            donneesGPS.nbSats = (buffer[cnt] - 0x30) * 10 + (buffer[cnt + 1] - 0x30);
            
            /* On saute le champ horizontal dilution */
            cnt = findNextVirgule(buffer, cnt, length) + 1;
            cnt = findNextVirgule(buffer, cnt, length) + 1;
            
            /************ Altitude ************/
            nextVirgule = findNextVirgule(buffer, cnt, length);
            
            /* On recupere la decimale et la valeur unite */
            donneesGPS.altitude = (buffer[nextVirgule - 1] - 0x30) * 0.1 + buffer[nextVirgule - 3] - 0x30;
            
            /* Selon le nombre de caracteres pour l'altitude */
            switch(nextVirgule - cnt)
            {
                /* 5 caracteres --> xxx.x */
                case 5:
                    donneesGPS.altitude += (buffer[nextVirgule - 5] - 0x30) * 100 + (buffer[nextVirgule - 4] - 0x30) * 10;
                    break;
                    
                /* 4 caracteres --> xx.x */
                case 4:
                    donneesGPS.altitude += (buffer[nextVirgule - 4] - 0x30) * 10;
                    break;
                    
                default:
                    break;                
            }
            
        }
        
        s[0] = donneesGPS.heure;
        s[1] = donneesGPS.minutes;
        s[2] = donneesGPS.secondes;
        s[3] = ',';
        s[4] = donneesGPS.latitude_deg;
        s[5] = donneesGPS.latitude_min;
        s[6] = donneesGPS.latitude_cent_min;
        s[7] = donneesGPS.latitude_dixmil_min;
        s[8] = donneesGPS.latitude_card;
        s[9] = ',';
        s[10] = donneesGPS.longitude_deg;
        s[11] = donneesGPS.longitude_min;
        s[12] = donneesGPS.longitude_cent_min;
        s[13] = donneesGPS.longitude_dixmil_min;
        s[14] = donneesGPS.longitude_card;
        s[15] = ',';
        s[16] = (char) ((short) donneesGPS.altitude >> 8);
        s[17] = (char) ((short) donneesGPS.altitude & 0xFF);
        s[18] = (char) ((donneesGPS.altitude - (short) donneesGPS.altitude) * 10);
        s[19] = ',';
        s[20] = donneesGPS.nbSats;
        s[21] = '\n';
        sendRXFrame(s, 22, 1);
        
        LED = !LED;
    }
}

/******************************* asciiToByte *********************************
 *  @Brief  : Cette fonction convertit un caractère ASCII hexadecimal en sa 
 *            valeur (ex : 'C' devient 0b00001100) 
 *  @Params : - char toConvert : La valeur a convertir
 *  @Retval : - char value     : La valeur convertie
 *****************************************************************************/
char asciiToByte(char toConvert)
{
    char value;
    
    /* On commence par enlever l'offset des chiffres en ascii : 0x30 */
    value = toConvert - 0x30;
    
    /* Si le caractere etait superieur a 9 (A,B,C,D,E,F), on enleve encore 7 */
    if(value > 0x0F)
    {
        value -= 0x07;
    }
    
    return value;
}

/***************************** findNextVirgule *******************************
 *  @Brief  : Cette fonction trouve la position de la prochaine virgule d'un 
 *            tableau a partir d'un index de depart
 *  @Params : - char* table     : Le tableau dans lequel chercher
 *            - short begin     : Le point de depart de la recherche
 *            - short length    : La taille du tableau 
 *  @Retval : - short position  : La position de la prochaine virgule
 *****************************************************************************/
short findNextVirgule(char* table, short begin, short length)
{
    short position = begin;
    
    /* On incremente la position jusqu'a arriver a une virgule ou etre en fin de 
     * tableau */
    while(table[position] != ',' && position < length)
    {
        position ++;
    }
    
    return position;
}