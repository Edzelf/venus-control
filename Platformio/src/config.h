// config.h
// File for custom configuration
// Veranderen van de algemene gegevens kan gevaarlijk zijn en is wellicht niet nodig.
#define NAME        "Venus-control"                       // Ident, also used for AP network.
#define UPDATEHOST  "venus.smallenburg.nl"                // Default host for software updates
#define UPDATEDIR   "/download"                           // Download directory
#define FIRMWAREBIN "firmware.bin"                        // Binary file name of firmware for OTA update
#define WEBINTFBIN  "spiffs.bin"                          // Binary file name of webinterface in SPIFFS for OTA update
//
// RS232/485 serial t.b.v. modbus communicatie
#define RS485BR     115200                                // Baudrate Venus ModBus interface
#define RS485MODE   SERIAL_8N1                            // 8 bits data, no parity, 1 stopbit

// Deze definities zijn veiliger om te wijzigen:
//#define FIXEDWIFI   "Nokia-basis-11/DEADC0DE11"           // Should be 2.4 GHz
#define FIXEDWIFI   "ADSL-11/DEADC0DE11"                  // Should be 2.4 GHz
