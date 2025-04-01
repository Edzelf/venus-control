//***************************************************************************************************
// Vmodbus.h                                                                                        *
//***************************************************************************************************
// Simpele modbus driver voor Venus-control project.                                                *
// Ondersteunt alleen lezen van 16 of 32 bits holdingregisters en schrijven van 16 bits             *
// holdingregisters.                                                                                *
//***************************************************************************************************
//                                                                                                  *
// 22-03-2025, ES:  Eerste opzet.                                                                   *
//***************************************************************************************************
#ifndef Vmodbus_h
#define Vmodbus_h

#include "Arduino.h"

class Vmodbus
{
  public:
    Vmodbus() ;                                                 // Constructor
    void     begin ( Stream &serial ) ;
    void     preTransmission ( void (*)() ) ;                   // Set pre-transmission function
    void     postTransmission ( void (*)() ) ;                  // Set post-transmission function
    
    bool     CRC ( uint8_t* buf, uint16_t len ) ;
    uint16_t getResponse_u16() ;                                // Haal resultaat op als uint16_t
    int16_t  getResponse_s16() ;                                // Haal resultaat op als int16_t
    uint32_t getResponse_u32() ;                                // Haal resultaat op als uint32_t
    int32_t  getResponse_s32() ;                                // Haal resultaat op als int32_t
    
    bool     readHRegisters ( uint16_t regnr, uint8_t nwords ) ;
    bool     writeHRegister ( uint16_t regnr, uint16_t word ) ;
    
  private:
    Stream*           _serial;                                  // Reference to serial port object
    const uint8_t     _MBSlave = 1 ;                            // Modbus slave, always 1
    void              (*_preTransmission)() ;                   // Called before tyransfer
    void              (*_postTransmission)() ;                  // Called after transfer
    const uint16_t    Timeout = 4000 ;                          // Modbus timeout [milliseconds]
    uint16_t          _ReadQty ;                                // Quantity of words to read
    uint16_t          _ResponseBuffer[2] ;                      // Buffer to store Modbus slave response
} ;
#endif
