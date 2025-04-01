//***************************************************************************************************
// Vmodbus.cpp                                                                                      *
//***************************************************************************************************
// Simpele modbus driver voor Venus-control project.                                                *
// Ondersteunt alleen lezen van 16 of 32 bits holdingregisters en schrijven van 16 bits             *
// holdingregisters.                                                                                *
//***************************************************************************************************
//                                                                                                  *
// 22-03-2025, ES:  Eerste opzet.                                                                   *
//***************************************************************************************************
#include <stdint.h>
#include "Vmodbus.h"

//******************************************************************************
//                            D U M M Y P R E P O S T                          *
//******************************************************************************
// Vervanging voor pre- of post functie.                                       *
//******************************************************************************
void dummyprepost()
{
}


//******************************************************************************
//                              V M O D B U S                                  *
//******************************************************************************
// Constructor.                                                                *
//******************************************************************************
Vmodbus::Vmodbus ( void )                           // Constructor
{
  _preTransmission = dummyprepost ;                 // Init met dummy function
  _postTransmission = dummyprepost ;
}


//******************************************************************************
//                                B E G I N                                    *
//******************************************************************************
// Start de ModBus driver.                                                     *
//******************************************************************************
void Vmodbus::begin ( Stream &serial )
{
  _serial = &serial;                      // Bewaar serial handle
  _serial->setTimeout ( Timeout ) ;       // Set time-out
}


//******************************************************************************
//                         P R E T R A N S M I S S I O N                       *
//******************************************************************************
// Configureer de functie die moet uitgevoerd worden voor de start van de      *
// transmissie.                                                                *
//******************************************************************************
void Vmodbus::preTransmission ( void (*preTransmission)() )
{
  _preTransmission = preTransmission ;
}


//******************************************************************************
//                       P O S T T R A N S M I S S I O N                       *
//******************************************************************************
// Configureer de functie die moet uitgevoerd worden na het einde van de       *
// transmissie.                                                                *
//******************************************************************************
void Vmodbus::postTransmission ( void (*postTransmission)() )
{
  _postTransmission = postTransmission ;
}


//******************************************************************************
//                                     C R C                                   *
//******************************************************************************
// Checks or generate CRC for Modbus RTU.                                      *
// Note that the length is for the message, excluding CRC.                     *
//******************************************************************************
bool Vmodbus::CRC ( uint8_t* buf, uint16_t len )
{
  uint16_t crc = 0xFFFF ;             // Set startvalue of CRC
  bool     res ;                      // Check result

  while ( len-- )
  {
    crc ^= *buf++ ;                   // XOR byte into least sig. byte of crc
    for ( int i = 0 ; i < 8 ; i++ )
    {                                 // Loop over each bit
      if ( crc & 1 )
      {                               // If the LSB is set
        crc = ( crc >>= 1 ) ^0xA001 ; // Shift right and XOR 0xA001
      }
      else                            // Else LSB is not set
      {
        crc >>= 1 ;                   // Just shift right
      }
    }
  }
  // buf now points to the CRC of the message
  res = *buf ^ ( crc & 0xFF ) ;       // Compare result
  *buf++ = crc & 0xFF ;               // Set first byte of new CRC
  res |= *buf ^ ( crc >> 8 ) ;        // Compare result part 2
  *buf = ( crc >> 8 ) ;               // Set second byte of CRC
  return ( res == 0 ) ;               // Return the result
}


//******************************************************************************
//                     R E A D H R E G I S T E R S                             *
//******************************************************************************
// Lees 1 of 2 holdingregisters van 16 bits.  Gaat fout bij meer dan 2         *
//******************************************************************************
bool Vmodbus::readHRegisters ( uint16_t regnr, uint8_t nwords )
{
  uint8_t  xmitbuf[10] ;                        // Buffer for transmit/receive
  uint8_t* p = xmitbuf ;                        // Pointer in xmitbuf
  uint8_t  brec ;                               // Aantal bytes te ontvangen
  uint8_t  n ;                                  // Aantal bytes ontvangen

  *p++ = _MBSlave ;                             // Slave adres
  *p++ = 0x03 ;                                 // Read holding register = 03
  *p++ = regnr >> 8 ;                           // High order deel van register
  *p++ = regnr & 0xFF ;                         // Lo order deel van register
  *p++ = 0 ;                                    // High order deel van aantal te lezen
  *p++ = nwords ;                               // Lo order deel van aantal
  CRC ( xmitbuf, 6 ) ;                          // Zet CRC erachteraan
  while ( _serial->read() != -1 )               // Flush receive buffer before transmitting request
  {
    delay ( 1 ) ;
  }
  _preTransmission() ;                          // Doe de pre-transmissie functie
  _serial->write ( xmitbuf, 8 ) ;               // Verstuur het request
  _serial->flush() ;                            // Forceer flush van transmit buffer
  _postTransmission() ;                         // Doe de post-transmissie functie
  brec = 5 + 2 * nwords ;
  n = _serial->readBytes ( xmitbuf, brec ) ;    // Lees verwacht aantal bytes
  if ( ( n != brec ) ||                         // Klopt het aantal bytes?
       ( xmitbuf[0] != _MBSlave ) ||            // Klopt het slave adres?
       ( xmitbuf[1] != 0x03 ) ||                // Klopt de functiecode?
       ( CRC ( xmitbuf, brec - 2 ) != true ) )  // Klopt de checksum?
  {
    Serial.printf ( "MB read: n is %d\n", n ) ;
    if ( n > 0 )
    {
      Serial.printf ( "MB read:<%02X><%02X><%02X><%02X><%02X><%02X><%02X>\n", 
                      xmitbuf[0], xmitbuf[1], xmitbuf[2], xmitbuf[3], xmitbuf[4], xmitbuf[5], xmitbuf[6] ) ;
    }
    return false ;                              // Nee, error!
  }
  p = xmitbuf + 3 ;                             // Wijs naar eerste registerinhoud
  _ResponseBuffer[0] = ( *p++ <<8 ) | *p++ ;    // Leest eerste woord
  _ResponseBuffer[1] = ( *p++ <<8 ) | *p++ ;    // Lees eventueel tweede woord, anders checksum
  return true ;
}


//******************************************************************************
//                     G E T R E S P O N S E _ U 1 6                           *
//******************************************************************************
// Geeft het resultaat van het lezen van 1 holdingregisters type uint16_t.     *
//******************************************************************************
uint16_t Vmodbus::getResponse_u16()
{
  return _ResponseBuffer[0] ;                     // Dat is de inhoud
}


//******************************************************************************
//                     G E T R E S P O N S E _ S 1 6                           *
//******************************************************************************
// Geeft het resultaat van het lezen van 1 holdingregisters type int16_t.      *
//******************************************************************************
int16_t Vmodbus::getResponse_s16()
{
  return (int16_t)_ResponseBuffer[0] ;            // Dat is de inhoud als int16
}


//******************************************************************************
//                     G E T R E S P O N S E _ U 3 2                           *
//******************************************************************************
// Geeft het resultaat van het lezen van 1 holdingregisters type uint32_t.     *
//******************************************************************************
uint32_t Vmodbus::getResponse_u32()
{
  return _ResponseBuffer[0] << 16 |                // Dat is de inhoud H.O.
         _ResponseBuffer[1] ;                       // Dat is de inhoud L.O.
}


//******************************************************************************
//                     G E T R E S P O N S E _ S 3 2                           *
//******************************************************************************
// Geeft het resultaat van het lezen van 1 holdingregisters type int32_t.      *
//******************************************************************************
int32_t Vmodbus::getResponse_s32()
{
  return (int32_t)getResponse_u32() ;            // Dat is de inhoud als int16
}


//******************************************************************************
//                       W R I T E R E G I S T E R                             *
//******************************************************************************
// Schrijf 1 holdingregister van 16 bits.                                      *
//******************************************************************************
bool Vmodbus::writeHRegister ( uint16_t regnr, uint16_t word )
{
  uint8_t       xmitbuf[10] ;                     // Buffer for transmit/receive
  uint8_t*      p = xmitbuf ;                     // Pointer in xmitbuf
  const uint8_t reclen = 8  ;                     // Aantal bytes te zenden/ontvangen
  uint8_t       n ;                               // Aantal bytes ontvangen

  *p++ = _MBSlave ;                               // Slave adres
  *p++ = 0x06 ;                                   // Write holding register = 06
  *p++ = regnr >> 8 ;                             // High order deel van register
  *p++ = regnr & 0xFF ;                           // Lo order deel van register
  *p++ = word >> 8 ;                              // High order deel van data
  *p++ = word & 0xFF ;                            // Lo order deel van data
  CRC ( xmitbuf, 6 ) ;                            // Zet CRC erachteraan
  while ( _serial->read() != -1 )                 // Flush receive buffer before xmit request
  {
    delay ( 1 ) ;
  }
  _preTransmission() ;                            // Doe de pre-transmissie functie
  _serial->write ( xmitbuf, reclen ) ;            // Verstuur het request
  _serial->flush() ;                              // Forceer flush van transmit buffer
  _postTransmission() ;                           // Doe de post-transmissie functie
  n = _serial->readBytes ( xmitbuf, reclen ) ;    // Lees verwacht aantal bytes
  if ( ( n != reclen ) ||                         // Klopt het aantal bytes?
       ( xmitbuf[0] != _MBSlave ) ||              // Klopt het slave adres?
       ( xmitbuf[1] != 0x06 ) ||                  // Klopt de functiecode?
       ( CRC ( xmitbuf, reclen - 2 ) != true ) )  // Klopt de checksum?
  {
    Serial.printf ( "MB write: n is %d\n", n ) ;
    if ( n > 0 )
    {
      Serial.printf ( "MB write:<%02X><%02X><%02X><%02X><%02X><%02X>\n", 
                      xmitbuf[0], xmitbuf[1], xmitbuf[2], xmitbuf[3], xmitbuf[4], xmitbuf[5] ) ;
    }
    return false ;                                // Nee, error!
  }
  return true ;
}
