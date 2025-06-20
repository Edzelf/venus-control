//***************************************************************************************************
//*  Main code for Venus-control.                                                                   *
//***************************************************************************************************
// Connects to the modbus interface to the Marstek Venus.  Exports website and variables to MQTT.   *
//                           By Ed Smallenburg.                                                     *
//***************************************************************************************************
// Board:            ESP32 DecKIt V1                                                                *
// Partition scheme: Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)                               *
//                   SPIFFS is used for web interface files.                                        *
//***************************************************************************************************
// This sketch uses SPIFFS for the web interface files.                                             *
//***************************************************************************************************
// Wiring.                                                                                          *
// s3-fh4r2 Signal  To MAX485    Remarks                                                            *
// -------- ------  ---------    -------------------------------                                    *
// GPIO7    RXD(2)   RO (1)      Received data from MAX485                                          *
// GPIO8    TXD(2)   DI (4)      Transmit data to MAX485                                            *
// GPIO9    485_EN   DE,RE (3,2) Enable RS485 transmit                                              *
// GPIO21   NEOPIXEL --          T.b.v. neopixel                                                    *
// -------  -------- ----------  --------------------------------                                   *
//                                                                                                  *
// Connectors op de huidige print:                                                                  *
// J1 Aansluiting Marstek ModBus                                                                    *
//                                                                                                  *
// History:                                                                                         *
// 11-02-2025, ES: First set-up.                                                                    *
// 20-03-2025, ES: Uitbreiding met handmatige instelling.                                           *
// 21-03-2025, ES: Beveiliging bij uitvallen van P1 dongle.  Batterij gaat dan naar stand-by.       *
// 25-03-2025, ES: Verbeterde ModBus driver, Homewizard P1 dongle.                                  *
// 27-03-2025, ES: Driver voor Shelly Pro 3 energy meter.                                           *
// 28-03-2025, ES: Grotere buffer voor JSON van P1 dongle, correctie ModBus enable.                 *
// 30-03-2025, ES: Correctie inlezen P1 i.v.m. rare header in JSON bij Homewizard.                  *
// 04-04-2025, ES: Grafieken pagina voor web-interface, minimale charge/discharge.                  *
// 05-04-2025, ES: No reset on web-interface refresh.                                               *
// 07-04-2025, ES: Correctie grafieken.                                                             *
// 08-04-2025, ES: Energie meting via MQTT.                                                         *
// 09-04-2025, ES: Correctie bug in ophalen dongle data via http, schaalfactor voor energie.        *
// 10-04-2025, ES: Initiële WiFi credentials verwacht in coredump partition.                        *
// 11-04-2025, ES: Correctie registratie van batterij opladen.  Statistieken in RTC geheugen.       *
// 15-04-2025, ES: Clean-up.  Configuratie van AP password erbij.                                   *
// 16-04-2025, ES: Publiceer MQTT data.                                                             *
// 03-05-2025, ES: Correctie manual setpoint in MQTT.                                               *
// 07-05-2025, ES: Correctie disconnected MQTT broker.                                              *
// 09-05-2025, ES: Andere MQTT library.  Uptime. Option om niet in "nom" mode te starten.           *
// 12-05-2025, ES: Correctie MQTT reconnect. Custum MQTT fields.                                    *
// 20-06-2025, ES: Variabele maximum laadvermogen erbij.                                            *
//***************************************************************************************************

#define VERSION     "Fri, 20 Jun 2025 08:30:00 GMT"       // Current version
#include <Arduino.h>
#include <soc/soc.h>                    // For brown-out detector setting
#include <soc/rtc_cntl_reg.h>           // For brown-out detector setting
#include <nvs_flash.h>
#include <ESPmDNS.h>                    // Voor het rondbazuinen van een mooie naam
#include <esp_wifi.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>               // MTTQ access
#include <SPIFFS.h>
#include <time.h>                       // Tijd functies
#include <esp_sntp.h>                   // Idem voor time server
#include <freertos/task.h>
#include <freertos/queue.h>
#include <nvs.h>
#include <esp_partition.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <ArduinoJson.h>                                  // JSON library
#include <Adafruit_NeoPixel.h>                            // Voor de neopixel op pin 21
#include <base64.h>
#include "Vmodbus.h"                                      // ModBus driver
#include "config.h"                                       // Configuratie definities

#define MAXJSONNAMESIZE    30                             // Maximale lengte van een JSON key
#define DEBUG_BUFFER_SIZE 160                             // Regellengte debugregel, exclusief tijd
#define DEBUG_SAVE_LINES 1000                             // Te bewaren aantal logregels 
#define NVSBUFSIZE        150                             // Longest expected string for nvsgetstr
#define MAXKEYS            40                             // Max. number of NVS keys in table
#define MAXINTF            32                             // Max. number of connected interfaces
                                                          // Highest ID is MAXINTF - 1
#define FSIF             true                             // Format SPIFFS if not existing

#define MAXMQTTCONNECTS    10                             // Maximum number of MQTT reconnects before give-up

#define SERIAL1        Serial                             // Serial via USB
// RS485 interface pins:
#define RS485PINRX          7                             // Default pin for RX of RS485 Serial port
#define RS485PINTX          8                             // Default pin for TX of RS485 Serial port
#define RS485PINDE          9                             // Pin for enabling xmit of MAX485
#define NEOPIN             21                             // Pin voor neopixel

#define MBtaskbit         0b0001                          // Mask for MBtask event
#define readP1taskbit     0b0010                          // Mask for p1task
#define controltaskbit    0b0100                          // Mask for controltask event
#define savetaskbit       0b1000                          // Mask for savetask

#define MODBUS_ENABLE     0x55AA                          // RS485 control mode enable in register 42000
#define MODBUS_DISABLE    0x55BB                          // RS485 control mode disable in register 42000

#define STATSIZ           100                             // Aantal punten in de statinfo_t buffers
#define VALIDDATA         0xFA1500FF                      // Code voor valid data in statistieken
#define CDSECRET          "VenusSecret$777"               // Code voor valid data in coredump partition


//***************************************************************************************************
// Forward declarations.                                                                            *
//***************************************************************************************************
void        dbgprint ( const char* format, ... ) ;
esp_err_t   nvsclear() ;
bool        nvssearch ( const char* key ) ;
String      nvsgetstr ( const char* key ) ;
esp_err_t   nvssetstr ( const char* key, String val ) ;
uint32_t    a2tohex ( const char* hstr ) ;
uint32_t    a8tohex ( const char* hstr ) ;
String      getContentType ( String filename ) ;
String      httpheader ( String contentstype ) ;
String      readprefs ( bool output ) ;
void        readprefs ( AsyncWebServerRequest *request ) ;
void        writeprefs ( AsyncWebServerRequest *request ) ;
const char* analyzeCmd ( const char* str ) ;
const char* analyzeCmd ( const char* par, const char* val ) ;
bool        read_p1_dongle_http() ;
void        claimData ( const char* p ) ;
void        releaseData() ;




//***************************************************************************************************
// Various structs and enums.                                                                       *
//***************************************************************************************************
struct logregel_t                                           // Voor array met logregels
{
   char logregel[DEBUG_SAVE_LINES][DEBUG_BUFFER_SIZE+20] ;  // Ruimte voor flink aantal logregels
} ;

// Structuur voor de statistieken
enum statst_t { SEC10, MIN1, MIN10, UUR1 } ;    // Geeft 1 van de 4 grafieken aan

struct stats2_t                                 // Statistiekbuffer, 1 meting
{
  int16_t pIn ;                                 // Netto energie in Watts + 32768
  int16_t pOut ;                                // Laad/ontlaad setting + 32768
  uint8_t soc ;                                 // Percentage opgeladen
} ;

struct stats1_t                                 // Statistiekbuffer, 1 type (10sec,1min,10min,1uur)
{
  int8_t   inx ;                                // Index in round robin buffer
  int16_t  sumpoints ;                          // Aantal metingen gemiddeld
  int32_t  sum_pIn ;                            // Sum t.b.v. gemiddelde pIn
  int32_t  sum_pOut ;                           // Sum t.b.v. gemiddelde pOut
  int32_t  sum_soc ;                            // Sum t.b.v. gemiddelde soc
  stats2_t buf[STATSIZ] ;                       // De data
} ;

struct stats0_t                                 // De 4 statistiekbuffers (10sec,1min,10min,1uur)
{
  int32_t  valid ;                              // Voor testen validiteit na reset
  stats1_t soort[4] ;                           // Dit zijn de 4 statistieken
  time_t   lasttime[4] ;                        // Tijdstip van laatste meting
} ;


//**************************************************************************************************
// Global data section.                                                                            *
//**************************************************************************************************
//
int                DEBUG = 1 ;                          // Debug on/off
//
bool               have_psram = false ;                 // Neem aan: geen psram
const char*        emptyWifiCred = "myssid/mypasswd" ;  // Credentials indien er geen echte zijn
String             lssid, lpw ;                         // SSID and password from nvs
char               nodename[32] = NAME ;                // Unieke nodenaam, bijvoorbeeld voor AP
String             APpassword ;                         // Password bij Accesspoint mode
bool               NetworkFound = false ;               // True if WiFi network connected in STA mode
char               json_buf[2048] ;                     // Result from P1
char*              json_begin = json_buf ;              // Begin JSON in json_buf
char               controltext[128] ;                   // Status van controltask als een tekst
char               controlmod[4] = "nom";               // Controller mode, "stb", "nom", "off" of "man"
int                man_setpoint ;                       // Setpoint in manual mode
int                RS485_pin_rx = RS485PINRX ;          // Default pin for RX of RS485 Serial port
int                RS485_pin_tx = RS485PINTX ;          // Default pin for TX of RS485 Serial port
int                RS485_pin_de = RS485PINDE ;          // Pin for enabling xmit of MAX485 (config)
uint8_t            venus_mb_adr = 1 ;                   // Modbus address of Marstek Venus
bool               simul = false ;                      // True for Modbus simulation
Vmodbus*           modbus ;                             // Modbus object
bool               modbusOkay = false ;                 // False if Modbus error seen
String             ipaddress ;                          // Own IP-address
AsyncWebServer     cmdserver ( 80 ) ;                   // Instance of embedded webserver, port 80
WiFiClient         P1_client ;                          // TCP client for P1-dongle connection
WiFiClient         wmqttclient ;                        // Instance voor mqtt, publicatie data
PubSubClient       mqttclient ( wmqttclient ) ;         // Client for MQTT subscriber data
String             mqtt ;                               // Username, password voor MQTT broker
String             broker ;                             // IP of domeinnaam van de broker uit mqtt
String             field_pIn ;                          // Field/topic Power delivered voor JSON/MQTT
String             field_pOut ;                         // Field/topic Power returned voor JSON/MQTT
String             dongle ;                             // Type (naam) P1-dongle
String             dongle_host ;                        // Hostnaam of IP-adres van P1-dongle
u_int16_t          dongle_port = 80 ;                   // TCP Port voor de api, default 80
String             dongle_api ;                         // Adres van de api
float              dongle_schaal = 1000.0 ;             // 1000 voor kW, 1 voor Watt
uint16_t           minCharge = 100 ;                    // Minimale laad/ontlaad vermogen
uint16_t           maxCharge = 2500 ;                   // Minimale laad/ontlaad vermogen
bool               P1_host_connected = false ;          // Connection to host is okay
bool               P1_host_error = false ;              // Connection to host in error
WiFiClient         otaclient ;                          // For OTA update vanaf update server
bool               http_reponse_flag = false ;          // Response required
String             http_getcmd ;                        // Contents of last GET command
String             http_rqfile ;                        // Requested file
HardwareSerial*    RS485serial = nullptr ;              // Serial port for RS485 Modbus connection
TaskHandle_t       xmaintask ;                          // Taskhandle for main task
TaskHandle_t       xMBtask ;                            // Task handle for lezen/schrijven modbus
TaskHandle_t       xreadP1task ;                        // Task handle for lezen dongle
TaskHandle_t       xcontroltask ;                       // Task handle for control of the battery
TaskHandle_t       xsavetask ;                          // Task handle vor save statistical data
EventGroupHandle_t taskEvents ;                         // Events for triggering various tasks
uint16_t           xwificount = 0 ;                     // Loop counter for P1 communication task
SemaphoreHandle_t  dbgsem ;                             // For exclusive usage of dbgprint
SemaphoreHandle_t  datasem ;                            // For exclusive access to real-time data
SemaphoreHandle_t  MBsem ;                              // For exclusive modbus usage
hw_timer_t*        timer ;                              // For timer
uint16_t           wdcount = 0 ;                        // Counter for watchdog timer
String             updhost = UPDATEHOST ;               // Default host to update software from WiFi
uint8_t            mac[6] ;                             // Key in POST data (mac address)
tm                 timeinf ;                            // Will be filled by time server
uint32_t           seconds = 0 ;                        // Seconds counter, free running
int                numprefs = 0 ;                       // Aantal gevonden preferences
char               cmd[130] ;                           // Command from Serial
bool               cmd_req = false ;                    // Commando staat klaar in cmd
bool               resetreq = false ;                   // Request to reset the ESP32
bool               updatefirmware = false ;             // OTA update firmware requested
bool               updatewebintf = false ;              // OTA update SPIFFS (webinterface) requested
String             IPstr ;                              // Text with IP address for
uint16_t           errorcount = 0 ;                     // Total errors detected
bool               WiFi_connected = false ;             // WiFi connected or not
char               nvs_val[NVSBUFSIZE] ;                // Buffer for value of one NVS key
JsonDocument       json_doc ;                           // JSON document uit P1
const char*        ntpServer = "pool.ntp.org" ;
const long         gmtOffset_sec = 3600 * 1 ;
const int          daylightOffset_sec = 3600 * 1 ;
int8_t             clk_offset = 1 ;                     // Offset in hours with respect to UTC
int8_t             clk_dst = 1 ;                        // Number of hours shift during DST
logregel_t*        dbglines ;                           // Array met laatste logregels
uint16_t           dbglinx = 0 ;                        // Index in dbglines
bool               mqtt_on = false ;                    // MQTT in use
bool               mqtt_ok = false ;                    // MQTT connectie is in ordel
uint16_t           mqttport = 1883 ;                    // Poort voor MQTT energiemeter, default 1883
String             mqttuser ;                           // User for MQTT energiemeter authentication
String             mqttpasswd ;                         // Password for MQTT energiemeter authentication

const esp_partition_t*    coredump = nullptr ;          // Pointer naar WiFi info in coredump
const esp_partition_t*    nvs = nullptr ;               // Pointer to NVS partition struct
esp_err_t                 nvserr ;                      // Error code from nvs functions
uint32_t                  nvshandle = 0 ;               // Handle for nvs access
char                      nvskeys[MAXKEYS][16] ;        // Space for NVS keys
//
// For Neopixel
Adafruit_NeoPixel neopixel ( 1, NEOPIN, NEO_RGB + NEO_KHZ800 ) ;
//                 ggrrbb
//                 ------
uint32_t red   = 0x001400 ;
uint32_t green = 0x140000 ;
uint32_t blue  = 0x000020 ;
uint32_t white = 0x141414 ;
uint32_t black = 0x000000 ;

// Code voor register type
enum mb_data_t { u16, s16, u32, s32, undef, eot } ;     // uint16_t, int16_t, uint32_t, int32_t,undefined en
                                                        // einde van tabel

// Code voor input registers. Zie volgorde in rtdata[].
enum sw_ireg_nr { SOC, ACPW, INVST,                     // State of charge, AC power, Inverter state
                  CHGCC, DISCC,                         // Charging cutoff capacity, discharging cutoff capacity
                  MAXCHG, MAXDIS,                       // Max charge power, max discharge power
                  PWIN, CURBC, SETBC,                   // Power in, current batterij charge, setpoint battery charge
                  ERRCNT, UPTIME,                       // Error count, uptime (seconds)
                  CF1, CF2, CF3, CF4,                   // Custom fields
                  CF5, CF6, CF7, CF8
} ;

struct RTInfo_t                                         // Structure of Real-time input data
{
  uint16_t   mb_regnr ;                                 // ModBus registernummer
  mb_data_t  mb_data_type ;                             // Soort register
  char       name[MAXJSONNAMESIZE+1] ;                  // Naam van het register
  int16_t    refr_time ;                                // Aantal seconden te wachten bij refreshen
  int16_t    refr_time_cnt ;  	                        // Telt seconden tot aan refr_time
  int32_t    A ;                                        // Ruwe naar fysieke waarde omrekenfactor, deelfactor
  int32_t    value ;                                    // Measured data in fysieke grootheid
} ;


RTInfo_t rtdata[] =                                     // Real-time data from Venus
{
    { 32104,   u16, "SOC",                 10, 10,  1,   52 },     // SOC    - State of charge in procenten, bijv 14
    { 32202,   s32, "AC power bat",         5,  5,  1,    0 },     // ACPW   - AC power into the grid, bijv -198
    { 35100,   u16, "Inverter-state",       5,  5,  1,    0 },     // INVST  - Inverters state 0..5
    { 44000,   u16, "Charge cutoff",       20, 20, 10,   90 },     // CHGCC  - Charging cutoff capacity 80..100%
    { 44001,   u16, "Discharge cutoff",    20, 20, 10,   12 },     // DISCC  - Discharging cutoff capacity 12..30%
    { 44002,   u16, "max charge power",    20, 20,  1, 2500 },     // MAXCHG - Max charge power 200..5000W
    { 44003,   u16, "max discharge power", 20, 20,  1,  800 },     // MAXDIS - Max discharge power 200..5000W
    {     0, undef, "Power in",            -1, -1,  1,    0 },     // PWIN   - Power in (P1) (negatief is power uit)
    { 32102,   s32, "Laadt met",            5,  5,  1,    0 },     // CURBC  - Huidig vermogen opladen (negatief is ontladen)
    {     0, undef, "Setpoint",            -1, -1,  1,    0 },     // SETBC  - Setpoint laden/ontladen batterij
    {     0, undef, "Fout teller",         -1, -1,  1,    0 },     // ERRCNT - Errorcount
    {     0, undef, "Uptime",              -1, -1,  1,    0 },     // UPTIME - Uptime in seconden
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 1
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 2
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 3
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 4
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 5
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 6
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 7
    {     0,   eot, "",                    60, 60,  1,    0 },     // Custom field 8
    {     0,   eot, "End of table",        -1, -1,  1,    0 },     // Absolue einde van de tabel
} ;

// Code voor R/W registers
enum sw_oreg_nr { CM485, FORCE,                         // RS485 mode, force (dis)charge
                  COUT, DOUT } ;                        // Charging power, discharging power

struct controlout_t                                     // Structure of control output data
{
  uint16_t   mb_regnr ;                                 // ModBus registernummer
  char       name[30] ;                                 // Naam van het register
  int16_t    refr_time ;                                // Aantal cycli te wachten bij refreshen
  int16_t    refr_time_cnt ;  	                        // Telt cycli tot aan refr_time
  int16_t    value ;                                    // Output data
} ;

controlout_t ctdata[] =                                 // Structure of control output data
{
  { 42000, "RS485 Control mode",  60,  60, MODBUS_ENABLE }, // CM485  - Enable/ disable RS485 control mode
  { 42010, "Forcible charge",      1,   0,      0        }, // FORCE  - 0 = stop, 1 = charge, 2 = discharge
  { 42020, "Charge power",         1,   0,      0        }, // COUT   - Forcible charge power
  { 42021, "DisCharge power",      1,   0,      0        }, // DOUT   - Forcible discharge power
} ;

#define NUMOREGS ( sizeof(ctdata) / sizeof(ctdata[0]) )

// Data in RTC geheugen.  Blijft bewaard bij soft reset, niet bij reset button of power on.
RTC_NOINIT_ATTR stats0_t statistics ;         // Ruimte voor de statistieken.

//**************************************************************************************************
// End of global data section.                                                                     *
//**************************************************************************************************

// Supported dongles:
#include "P1_Dongle_pro.h"          // smart-stuff.nl model
#include "Homewizard.h"             // Home Wizard model  (UNTESTED)
#include "Shelly_pro_3.h"           // Shelly Pro 3 energy meter
#include "P1_MQTT.h"                // Energie meter via MQTT
#include "P1_simul.h"               // Simulation of a dongle



//**************************************************************************************************
//                                          D B G P R I N T                                        *
//**************************************************************************************************
// Send a line of info to serial output.  Works like vsprintf(), but checks the DEBUG flag.        *
// Debug lines will be added to dbglines, a buffer holding the last debuglines.                    *
// Print only if DEBUG flag is true.                                                               *
//**************************************************************************************************
void dbgprint ( const char* format, ... )
{
  static char sbuf[DEBUG_BUFFER_SIZE] ;                 // For debug lines
  va_list varArgs ;                                     // For variable number of params
  char     tbuf[16] ;                                   // For time stamp
  String   dbgline ;                                    // Resulting line

  if ( xSemaphoreTake ( dbgsem, 20 ) != pdTRUE  )       // Claim resource
  {
    return ;                                            // Not available
  }
  va_start ( varArgs, format ) ;                        // Prepare parameters
  vsnprintf ( sbuf, sizeof(sbuf), format, varArgs ) ;   // Format the message
  va_end ( varArgs ) ;                                  // End of using parameters
  if ( DEBUG )                                          // DEBUG on?
  {
    sprintf ( tbuf, "%02d:%02d:%02d - ",                // Yes, print prefix (time of day)
                    timeinf.tm_hour,
                    timeinf.tm_min,
                    timeinf.tm_sec ) ;
    dbgline = String ( tbuf ) + String ( sbuf ) ;       // Time and info to a String
    SERIAL1.println ( dbgline.c_str() ) ;               // and the info
    if ( dbglines )                                     // Opslagruimte aangemaakt?
    {
      if ( dbglinx < DEBUG_SAVE_LINES )                 // Ja, ruimte beschikbaar dbglines?
      {
        strcpy ( dbglines->logregel[dbglinx++],         // Yes, add to buffer with debug lines
                 dbgline.c_str() ) ;
      }
    }
  }
  xSemaphoreGive ( dbgsem ) ;                           // Release resource
}


//**************************************************************************************************
//                                      S E T P I X E L                                            *
//**************************************************************************************************
// Toon een kleur op de neopixel.                                                                  *
//**************************************************************************************************
void setpixel ( uint32_t color )
{
  neopixel.setPixelColor ( 0, color ) ;                     // Zet kleur van de enige LED
  neopixel.show() ;                                         // Toon die kleur
}


//**************************************************************************************************
//                      T I M E _ S Y N C _ N O T I F I C A T I O N _ C B                          *
//**************************************************************************************************
// Call-back van timeserver.                                                                       *
//**************************************************************************************************
void time_sync_notification_cb ( struct timeval *tv )
{
  dbgprint ( "Tijd sync event ontvangen" ) ;
}


//**************************************************************************************************
//                            I N I T I A L I Z E _ S N T P                                        *
//**************************************************************************************************
// Initialiseer aanroep van een NTP server om de tijd te synchroniseren.                           *
//**************************************************************************************************
void initialize_sntp()
{
  dbgprint ( "Start SNTP" ) ;
  esp_sntp_setoperatingmode ( SNTP_OPMODE_POLL ) ;
  esp_sntp_setservername ( 0, ntpServer ) ;
  esp_sntp_set_time_sync_notification_cb ( time_sync_notification_cb ) ;
  esp_sntp_init() ;
}


//**************************************************************************************************
//                                 O B T A I N _ T I M E                                           *
//**************************************************************************************************
// Haal de tijd van een SNTP server.                                                               *
//**************************************************************************************************
void obtain_time()
{
  time_t             tnow = 0 ;                                 // Wordt de tijd van timeserver
  const char*        timezone ;                                 // Timezone voor nederland 
  int                retry = 30 ;                               // Teller aantal maal proberen
  sntp_sync_status_t stat ;                                     // Sync status

  initialize_sntp() ;                                           // Init ntp
  while ( --retry > 0 )                                         // Tel aantal retries
  {
    stat = sntp_get_sync_status() ;                             // Haal sync status op
    if ( stat == SNTP_SYNC_STATUS_COMPLETED )                   // Tijd gezien?
    {
      break ;                                                   // Ja, okay
    }
    else
    {
      dbgprint ( "Wacht op NTP (%d), stat is %d...",            // Toon dat we bezig zijn
                 retry, stat ) ;
      delay ( 2000 ) ;
    }
  }
  if ( retry )
  {
    timezone = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00" ; // Time zone voor Nederland
    setenv ( "TZ", timezone, 1 ) ;                              // Set timezone voor NL
    tzset() ;                                                   // Switch naar deze zone
    time ( &tnow ) ;                                            // Haal tijd op
    localtime_r ( &tnow, &timeinf ) ;                           // Splits
    dbgprint ( "Verbinding met NTP gelukt" ) ;
  }
  else
  {
    dbgprint ( "Verbinding met NTP mislukt!" ) ;
  }
}


//**************************************************************************************************
//                                  D B G T A S K I N F O                                          *
//**************************************************************************************************
// Show some task information.  Called at start of task.                                           *
//**************************************************************************************************
void dbgTaskInfo()
{
  TaskHandle_t  th ;                                       // Own taskhandle

  th = xTaskGetCurrentTaskHandle() ;                       // Get own task handle
  dbgprint ( "Taak %-12s op CPU %d, "                      // Show activity
             "%d MHz. Vrij geheugen %6d bytes",
             pcTaskGetTaskName ( th ),
             xPortGetCoreID(),
             ESP.getCpuFreqMHz(),
             ESP.getFreeHeap() ) ;
}


//**************************************************************************************************
//                                      C L A I M D A T A                                          *
//**************************************************************************************************
// Claim the data.  Uses FreeRTOS semaphores.                                                      *
// De parameter geeft een aanwijzing voor debuggen.                                                *
//**************************************************************************************************
void claimData ( const char* p )
{
  const        TickType_t ctry = 10 ;                       // Time to wait for semaphore
  uint32_t     count = 0 ;                                  // Wait time in ticks

  while ( xSemaphoreTake ( datasem, ctry ) != pdTRUE  )     // Claim data area
  {
    if ( count++ > 10 )
    {
      dbgprint ( "Data semaphore niet beschikbaar gedurende %d ticks op CPU %d, id %s",
                 count * ctry,
                 xPortGetCoreID(),
                 p ) ;
    }
  }
}


//**************************************************************************************************
//                                   R E L E A S E D A T A                                         *
//**************************************************************************************************
// Free the data resource.  Uses FreeRTOS semaphores.                                              *
//**************************************************************************************************
void releaseData()
{
  xSemaphoreGive ( datasem ) ;                            // Release SPI bus
}


//**************************************************************************************************
//                                      C L A I M M B                                              *
//**************************************************************************************************
// Claim the modbus.  Uses FreeRTOS semaphores.                                                      *
// De parameter geeft een aanwijzing voor debuggen.                                                *
//**************************************************************************************************
void claimMb ( const char* p )
{
  const        TickType_t ctry = 10 ;                       // Time to wait for semaphore
  uint32_t     count = 0 ;                                  // Wait time in ticks

  while ( xSemaphoreTake ( MBsem, ctry ) != pdTRUE  )       // Claim SPI bus
  {
    if ( count++ > 10 )
    {
      dbgprint ( "ModBus semaphore niet beschikbaar gedurende %d ticks op CPU %d, id %s",
                 count * ctry,
                 xPortGetCoreID(),
                 p ) ;
    }
  }
}


//**************************************************************************************************
//                                   R E L E A S E M B                                             *
//**************************************************************************************************
// Free the ModBus resource.  Uses FreeRTOS semaphores.                                            *
//**************************************************************************************************
void releaseMb()
{
  xSemaphoreGive ( MBsem ) ;                                  // Release SPI bus
}


//**************************************************************************************************
//                                        B U I L D J S O N                                        *
//**************************************************************************************************
// Maak een JSON string met de besturingsvariabelen.                                               *
// Het resultaat wordt in de buffer gezet die als paramater wordt meegegeven.                      *
//**************************************************************************************************
void buildJSON ( char* bjbuf, int16_t len )
{
  JsonDocument doc ;                                      // JSON structuur
  int          inx = 0 ;                                  // Index in JSON structuur
  
  claimData ( "buildJSON" ) ;                             // Exclusieve toegang
  rtdata[UPTIME].value = seconds ;                        // Zet uptime in rtdata
  while ( rtdata[inx].mb_data_type != eot )               // Ga alle variabelen langs
  {
    int tmp = rtdata[inx].value ;                         // Get data als integer
    doc[rtdata[inx].name] = String ( tmp ) ;              // Vul een key/value pair
    inx++ ;
  }
  doc["Controller_tekst"] = controltext ;                 // Vul extra key/value pair
  doc["Controller_mode"] = controlmod ;                   // Ook modus (nom,man,stb,off)
  doc["Manual_setpoint"] = String ( man_setpoint ) ;      // En handmatig setpoint
  releaseData() ;
  serializeJsonPretty ( doc, bjbuf, len ) ;               // Maak er een string van
}


//**************************************************************************************************
//                                P 1 _ M Q T T _ I N I T                                          *
//**************************************************************************************************
// Set subscribe topics voor power-delivered en power-returned.                                    *
// De topic namen zijn configureerbaar.                                                            *
//**************************************************************************************************
void p1_mqtt_init()
{
  bool     res = true  ;                                    // Resultaat van subscribe

  if ( ! field_pIn.isEmpty() )
  {
    res && mqttclient.subscribe ( field_pIn.c_str() ) ;     // Subscribe to MQTT, "power delivered" topic
  }
  if ( ! field_pOut.isEmpty() )
  {
    res && mqttclient.subscribe ( field_pOut.c_str() ) ;    // Subscribe to MQTT, "power returned" topic
  }
  if ( !res )
  {
    dbgprint ( "MQTT subscribe fout!" ) ;                   // Failure
  }
}



//**************************************************************************************************
//                                    M Q T T R E C O N N E C T                                    *
//**************************************************************************************************
// Maak een (nieuwe) connectie met de broker.                                                      *
// Indien force == true, dan wordt de re-connectie altijd gedaan.                                   *
//**************************************************************************************************
bool mqttreconnect ( bool force )
{
  static uint32_t retrytime = 0 ;                         // Limiteer reconnect interval
  static uint16_t mqttcount = 0 ;                         // Teller MAXMQTTCONNECTS
  bool            res = false ;                           // Connect resultaat

  if ( ! force && ( ( millis() - retrytime ) < 5000 ) )   // Echt doen?
  {
    return true ;                                         // Nee, return okay status
  }
  retrytime = millis() ;                                  // Set tijdstip laatste poging
  if ( mqttcount > MAXMQTTCONNECTS )                      // Genoeg pogingen?
  {
    mqtt_on = false ;                                     // Ja, uit voor altijd
    mqtt_ok = false ;
    dbgprint ( "MQTT give-up!" ) ;
    strcpy ( controlmod, "stb" ) ;                        // Ga naar stand-by
    return res ;                                          // Doorgaan niet zinvol
  }
  mqttcount++ ;                                           // Count the retries
  mqttclient.setServer ( broker.c_str(), mqttport ) ;     // Specificeer de broker en de poort
  mqttclient.setCallback ( onMqttMessage ) ;              // Set callback on receive
  res = mqttclient.connect ( nodename,                    // Connect to broker
                             mqttuser.c_str(),            // Met deze user
                             mqttpasswd.c_str() ) ;       // Met dit password
  if ( res )
  {
    dbgprint ( "MQTT connectie %d gelukt", mqttcount ) ;
    mqttcount = 0 ;                                       // Okay, reset errorcount
    mqtt_ok = true ;
    p1_mqtt_init() ;                                      // Init P1_MQTT energiemeter
  }
  else
  {
    dbgprint ( "MQTT connectie %d fout, rc=%d!",          // Fout, toon dat
               mqttcount, mqttclient.state() ) ;
  }
  return res ;
}


//**************************************************************************************************
//                                      W I F I E V E N T                                          *
//**************************************************************************************************
// Will be executed on WiFi driver events.                                                         *
//**************************************************************************************************
void WiFiEvent ( WiFiEvent_t event )
{
  switch ( event )                                    // What event?
  {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED :
      dbgprint ( "Station verbonden" ) ;
      break ;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED :
      dbgprint ( "Station verbroken" ) ;
      break ;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP :
      dbgprint ( "IP adres verkregen" ) ;
      break ;
    case ARDUINO_EVENT_WIFI_STA_STOP :
      dbgprint ( "Station stop" ) ;
      break ;
    case ARDUINO_EVENT_WIFI_OFF :
      dbgprint ( "wifi off" ) ;
      break ;
    case ARDUINO_EVENT_WIFI_READY :
      dbgprint ( "wifi ready" ) ;
      break ;
    case ARDUINO_EVENT_WIFI_AP_START :
      dbgprint ( "AP start" ) ;
      break ;
    case ARDUINO_EVENT_WIFI_AP_STOP :
      dbgprint ( "AP stop" ) ;
      break ;
    default :
      dbgprint ( "WiFi event %d", (int)event ) ;      // Unknown event
      break ;
  }
}


//**************************************************************************************************
//                                      S E T W I F I C R E D                                      *
//**************************************************************************************************
// Set the SSID and password from the entry in preferences.                                        *
// Will be called only once by setup().                                                            *
//**************************************************************************************************
void  setWifiCred()
{
  String      buf ;                                       // "SSID/password"
  int         inx ;                                       // Place of "/"
  const char* key = "wifi" ;

  buf = String ( emptyWifiCred ) ;                        // Neem aan: een fantasie netwerk
  if ( nvssearch ( key ) )                                // WiFi credentials in NVS?
  {
    buf = nvsgetstr ( key ) ;                             // Ja, neem over
  }
  else
  {
    nvssetstr ( "wifi", buf ) ;                           // Zorg voor een gevulde key in NVS
  }
  inx = buf.indexOf ( "/" ) ;                             // Zoek scheiding tussen ssid en password
  if ( inx > 0 )                                          // Scheiding gevonden?
  {
    lpw = buf.substring ( inx + 1 ) ;                     // Ja, isoleer password
    lssid = buf.substring ( 0, inx ) ;                    // Is nu SSID
  }
}


//******************************************************************************************
//                               C O N N E C T W I F I                                     *
//******************************************************************************************
// Connect to WiFi.                                                                        *
//******************************************************************************************
bool connectwifi()
{
  bool     localAP = true ;                             // True if only local AP is left
  uint16_t trycount = 0 ;                               // Teller aantal pogingen
  
  dbgprint ( "Probeer WiFi %s", lssid.c_str() ) ;       // Message to show during WiFi connect
  WiFi.begin ( lssid.c_str(), lpw.c_str() ) ;           // Connect to selected SSID
  while ( millis() < 60000  )                           // Geef 1 minuut de tijd voor connect
  {
    if (  WiFi.status() == WL_CONNECTED )               // Is er nu verbinding?
    {
      localAP = false ;                                 // Ja, dus we maken geen AP
      break ;
    }
    if ( ( trycount++ % 10 ) == 0 )                     // Elke 10 seconden een reconnect doen
    {
      WiFi.reconnect() ;                                // Misschien helpt dat
    }
    if ( trycount & 1 )                                 // Bepaal kleur voor knipperen
    {
      setpixel ( blue ) ;                               // 1 seconde blauw
    }
    else
    {
      setpixel ( green ) ;
    }
    delay ( 1000 ) ;                                    // Seconde wachten
  }
  if ( localAP )                                        // Lokaal AP opzetten?
  {
    if ( APpassword.isEmpty() )                         // Exotisch password opgegeven?
    {
      APpassword = nodename ;                           // Nee, gebruik default (=nodename)
    }
    dbgprint ( "WiFi verbinding mislukt!  "             // Toon het probleem
               "Probeer AP op te zetten met"
               " naam %s en password %s.",
               nodename, APpassword.c_str() ) ;
    WiFi.disconnect ( true ) ;                          // After restart the router could
    WiFi.softAPdisconnect ( true ) ;                    // still keep the old connection
    if ( ! WiFi.softAP ( nodename,                      // This ESP will be an AP
                         APpassword.c_str() ) )
    {
      dbgprint ( "AP opzetten mislukt" ) ;              // Setup of AP failed
    }
    ipaddress = String ( "192.168.4.1" ) ;              // Fixed IP address
  }
  else
  {
    dbgprint ( "SSID = %s, sterkte = %d dB",            // Format string with SSID connected to
               WiFi.SSID().c_str(),
               WiFi.RSSI() ) ;
    ipaddress = WiFi.localIP().toString() ;             // Form IP address
    setpixel ( green ) ;
  }
  dbgprint ( "IP = %s", ipaddress.c_str() ) ;
  return ( localAP == false ) ;                         // Return result of connection
}


//**************************************************************************************************
//                                         C H O M P                                               *
//**************************************************************************************************
// Do some filtering on de inputstring:                                                            *
//  - String comment part (starting with "#").                                                     *
//  - Strip trailing CR.                                                                           *
//  - Strip leading spaces.                                                                        *
//  - Strip trailing spaces.                                                                       *
//**************************************************************************************************
void chomp ( String &str )
{
  int   inx ;                                         // Index in de input string

  if ( ( inx = str.indexOf ( "#" ) ) >= 0 )           // Comment line or partial comment?
  {
    str.remove ( inx ) ;                              // Yes, remove
  }
  str.trim() ;                                        // Remove spaces and CR
}

//**************************************************************************************************
//                             J S O N R E P L A C E S P A C E S                                   *
//**************************************************************************************************
// Vervang de spaties in de JSON keys door underlianes.                                            *
//**************************************************************************************************
void  jsonReplaceSpaces()
{
  uint8_t inx = 0 ;                                       // Index in rtdata
  char*   p ;                                             // Will point to next space
  
  while ( rtdata[inx].mb_data_type != eot )
  {
    while ( ( p = strchr ( rtdata[inx].name, ' '  ) ) )   // Zit er een spatie in deze naam?
    {
      *p = '_' ;                                          // Ja, vervang
    }
    inx++ ;
  }
}


//**************************************************************************************************
//                                      M Q T T _ I N I T                                          *
//**************************************************************************************************
// Initialiseer de MQTT broker.                                                                    *
// Wordt gedaan als er in de configuratie een definitie is voor "mqtt = ...".                      *
//**************************************************************************************************
void mqtt_init()
{
  uint16_t inx1 ;                                           // Index in mqtt
  uint16_t inx2 ;                                           // Index in mqtt

  mqtt_on = true ;                                          // Enable MQTT
  if ( ( inx1 = mqtt.indexOf ( ":" ) ) >= 0  )              // Zoek scheiding tussen username en password
  {
    mqttuser = mqtt.substring ( 0, inx1++ ) ;               // Set username, set index op begin password
    //dbgprint ( "user is %s", mqttuser.c_str() ) ;
    if ( ( inx2 = mqtt.indexOf ( "@" ) ) > inx1 )           // Zoek scheiding tussen username en password
    {
      mqttpasswd = mqtt.substring ( inx1, inx2 ) ;        	// Set password
      //dbgprint ( "pw is %s", mqttpasswd.c_str() ) ;
      broker = mqtt.substring ( ++inx2 ) ;                  // Set domein of IP van de broker
      //dbgprint ( "broker is %s", broker.c_str() ) ;
    }
  }
  if ( ! mqttclient.setBufferSize ( 512, 512 ) )            // Vergroot de buffer
  {
    dbgprint ( "MQTT setbuffersize mislukt!" ) ;            // Maar dat lukt niet
  }
  mqttreconnect ( true ) ;                                  // Connect to broker
}


//**************************************************************************************************
//                            G E T _ M O D B U S _ D A T A _ R E G S                              *
//**************************************************************************************************
// Read data registers from modbus.                                                                *
// Registers are 2 bytes, big endian.                                                              *
// Na ontvangst van de reply wachten we even om ervoor te zorgen dat de requests niet te snel      *
// achter elkaar komen.                                                                            *
//**************************************************************************************************
bool get_modbus_data_regs()
{
  bool        fres = true ;                                       // Function result, assume positive
  uint8_t     i = 0 ;                                             // Loop control
  uint16_t    regnr ;                                             // Register te lezen
  mb_data_t   dt ;                                                // Type, bijvoorbeeld "s32"
  uint16_t    n ;                                                 // Aantal register te lezen, 1 = 16 bits woord
  int32_t     value32 ;                                           // Gelezen waarde, 32 bits
  const char* TAG = "get_modbus_data_regs" ;                      // Voor debugging semaphore

  if ( simul )                                                    // Aan het simuleren?
  {
    return true ;                                                 // Ja, gauw klaar
  }
  for ( i = 0 ; ; i++ )
  {
    dt = rtdata[i].mb_data_type ;                                 // Data type
    if ( dt == eot )                                              // Einde van de tabel?
    {
      break ;                                                     // Ja, stop
    }
    regnr = rtdata[i].mb_regnr ;                                  // Register te lezen
    if ( regnr == 0 )                                             // Is het een Modbus register?
    {
      continue ;                                                  // Nee, sla over
    }
    if ( ++rtdata[i].refr_time_cnt < rtdata[i].refr_time )        // Tijd voor actie?
    {
      continue ;                                                  // Nee, sla over
    }
    rtdata[i].refr_time_cnt = 0 ;                                 // Ja, reset counter al vast
    n = 1 ;                                                       // Meestal 1 woord van 16 bits
    if ( ( dt == u32 ) || ( dt == s32 ) )                         // Dubbel word?
    {
      n = 2 ;                                                     // Ja, lees 2 woorden van 16 bits
    }
    //dbgprint ( "Lees %d registers vanaf %d(%s)",
    //           n, regnr, rtdata[i].name ) ;
    claimMb ( TAG ) ;                                             // Access modbus hardware
    if ( modbus->readHRegisters ( regnr, n  ) )                   // Read holdingregister (0x03) from Venus
    {
      switch ( dt )
      {
        case u16 :
          value32 = modbus->getResponse_u16() ;                   // Haal u16 data op
          break ;
        case s16 :
          value32 = modbus->getResponse_s16() ;                   // Haal s16 data op
          break ;
        case s32 :
          value32 = modbus->getResponse_s32() ;                   // Haal s32 data op
          break ;
        case u32 :
          value32 = modbus->getResponse_u32() ;                   // Haal u32 data op
          break ;
      }
      delay ( 5 ) ;                                               // Laat anderen toe
      releaseMb() ;                                               // Geef ModBus weer vrij
      claimData ( "modbus get" ) ;                                // Exclusieve toegang tot data vragen
      rtdata[i].value = value32 / rtdata[i].A ;                   // Scale and store data
      //dbgprint ( "Register inhoud is %d", rtdata[i].value ) ;
      releaseData() ;                                             // Geef data weer vrij
    }
    else
    {
      delay ( 5 ) ;                                               // Laat anderen toe
      releaseMb() ;                                               // Geef resource weer vrij
      dbgprint ( "MB: Fout bij lezen van register %d!",
                 regnr ) ;
      fres = false ;      	                                      // Set negatief function result
      errorcount++ ;                                              // Count errors
      break ;                                                     // Doorgaan is zinloos
    }
  }
  return fres ;                                                   // Return result as a boolean
}


//**************************************************************************************************
//                            P U T _ M O D B U S _ D A T A _ R E G S                              *
//**************************************************************************************************
// Write data registers to modbus.                                                                 *
// Registers are 2 bytes, big endian.                                                              *
// De parameter geeft aan of het een timeout was.                                                  *
//**************************************************************************************************
bool put_modbus_data_regs ( bool was_timeout )
{
  uint8_t     i ;                                                 // Loop control
  uint16_t    regnr ;                                             // Register te lezen
  uint16_t    value ;                                             // Gelezen waarde
  const char* TAG = "put_modbus_data_regs" ;                      // Voor debugging semaphore
  
  if ( simul )                                                    // Simulatie?
  {
    return true ;                                                 // Ja, gauw klaar
  }
  for ( i = 0 ; i < NUMOREGS ; i++ )                              // Ga de outputs langs
  {
    regnr = ctdata[i].mb_regnr ;                                  // Register te lezen
    if ( ++ctdata[i].refr_time_cnt < ctdata[i].refr_time )        // Tijd voor actie?
    {
      continue ;                                                  // Nee, sla over
    }
    ctdata[i].refr_time_cnt = 0 ;                                 // Ja, reset counter al vast
    value = ctdata[i].value ;                                     // Waarde te schrijven
    //dbgprint ( "Schrijf registers %d (%s), inhoud is %d",
    //          regnr, ctdata[i].name, value ) ;
    claimMb ( TAG ) ;                                             // Access modbus hardware
    modbusOkay = modbus->writeHRegister ( regnr, value  ) ;       // Schrijf holdingregister
    delay ( 5 ) ;                                                 // Laat anderen toe
    releaseMb() ;                                                 // Geef resource weer vrij
    if ( ! modbusOkay )                                           // Success?
    {
      dbgprint ( "MB: Fout bij schrijven register %d!", regnr) ;  // Modbus I/O error
      errorcount++ ;                                              // Count errors
      break ;                                                     // Doorgaan is zinloos
    }
  }
  return modbusOkay ;                                             // Return result as a boolean
}


//**************************************************************************************************
//                                           O T A S T A R T                                       *
//**************************************************************************************************
// Update via WiFi has been started by update request.                                             *
//**************************************************************************************************
void otastart()
{
  dbgprint ( "OTA update gestart" ) ;
}


//**************************************************************************************************
//                                D O _ S O F T W A R E _ U P D A T E                              *
//**************************************************************************************************
// Update software or SPIFFS from OTA stream.                                                      *
//**************************************************************************************************
bool do_software_update ( uint32_t clength, int command )
{
  bool res = false ;                                          // Update result
  
  if ( Update.begin ( clength, command  ) )                   // OTA to flash possible?
  {
    dbgprint ( "Start OTA update, lengte is %d bytes",
               clength ) ;
    setpixel ( black ) ;                                      // LED uit
    if ( Update.writeStream ( otaclient ) == clength )        // writeStream is the real download
    {
      dbgprint ( "%d bytes met succes geschreven", clength ) ;
    }
    else
    {
      dbgprint ( "Schrijven mislukt!" ) ;
    }
    if ( Update.end() )                                       // Check for successful flash
    {
      dbgprint( "OTA uitgevoerd" ) ;
      if ( Update.isFinished() )
      {
        dbgprint ( "Update met succes beëindigd" ) ;
        res = true ;                                          // Positive result
      }
      else
      {
        dbgprint ( "Update niet goed beëindigd!" ) ;
      }
    }
    else
    {
      dbgprint ( "Fout opgetreden. Melding is %s",
                 Update.getError() ) ;
      errorcount++ ;                                          // Count errors
    }
  }
  else
  {
    // Not enough space to begin OTA
    dbgprint ( "Te weinig ruimte om OTA te doen!" ) ;
    otaclient.stop() ;
  }
  return res ;
}


//**************************************************************************************************
//                            S C A N _ C O N T E N T _ L E N G T H                                *
//**************************************************************************************************
// If the line contains content-length information: return clength (content length counter).       *
//**************************************************************************************************
uint32_t scan_content_length ( const char* metalinebf )
{
  if ( strstr ( metalinebf, "Content-Length" ) )        // Line contains content length
  {
    return atoi ( metalinebf + 15 ) ;                   // Yes, set clength
  }
  return 0 ;
}


//**************************************************************************************************
//                                        U P D A T E _ S O F T W A R E                            *
//**************************************************************************************************
// Update Venus-control software by download from remote host.                                     *
// Zowel de firmware als de website kan worden ververst.                                           *
//**************************************************************************************************
bool update_software ( const char* lstmodkey,                   // v_firmware or v_webintf
                       const char* updatehost,
                       const char* updatedir,
                       const char* binfile,                     // firmware.bin or spiffs.bin
                       int         command )                    // U_FLASh or U_SPIFFS
{
  uint32_t    timeout = millis() ;                              // To detect time-out
  String      line ;                                            // Input header line
  String      lstmod = "" ;                                     // Last modified timestamp in NVS
  String      newlstmod ;                                       // Last modified from host
  bool        res = false ;                                     // Result of download
  uint32_t    tmpl ;                                            // Temporary content length
  uint32_t    updlen ;                                          // Contentlength
  
  otastart() ;                                                  // Show something on screen
  lstmod = nvsgetstr ( lstmodkey ) ;                            // Get current last modified timestamp
  dbgprint ( "Verbinden met %s voor /%s/%s",
              updatehost, updatedir, binfile ) ;
  if ( !otaclient.connect ( updatehost, 80 ) )                  // Connect to host
  {
    dbgprint ( "Verbinden met updateserver mislukt!" ) ;
    return res ;
  }
  otaclient.printf ( "GET %s/%s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "Cache-Control: no-cache\r\n"
                     "Connection: close\r\n\r\n",
                     updatedir, binfile, updatehost ) ;
  while ( otaclient.available() == 0 )                          // Wait until response appears
  {
    if ( millis() - timeout > 5000 )                            // Moet wel binnen 5 seconden gebeuren
    {
      dbgprint ( "Verbinden met updateserserver time-out!" ) ;
      otaclient.stop() ;
      return res ;
    }
  }
  // Connected, handle response
  while ( otaclient.available() )
  {
    line = otaclient.readStringUntil ( '\n' ) ;                 // Read a line from response
    line.trim() ;                                               // Remove garbage
    if ( !line.length() )                                       // End of headers?
    {
      break ;                                                   // Yes, get the OTA started
    }
    dbgprint ( line.c_str() ) ;                                 // Debug info
    // Check if the HTTP Response is 200.  Any other response is an error.
    if ( line.startsWith ( "HTTP/1.1" ) )                       // 
    {
      if ( line.indexOf ( " 200 " ) < 0 )
      {
        dbgprint ( "Kreeg een niet-200 fout code van de "
                   "server: %s!", line.c_str() ) ;
        return res ;
      }
    }
    if ( ( tmpl = scan_content_length ( line.c_str() ) ) )      // Scan for content_length
    {
      updlen = tmpl ;                                           // Found length
    }
    if ( line.startsWith ( "Last-Modified: " ) )                // Timestamp of binary file
    {
      newlstmod = line.substring ( 15 ) ;                       // Isolate timestamp new version
      dbgprint ( "Huidige versie: %s", lstmod.c_str() ) ;       // Show current version too
    }
  }
  // End of headers reached
  if ( newlstmod == lstmod )                                    // Need for update?
  {
    dbgprint ( "Geen nieuwe versie beschikbaar" ) ;             // No, show reason
    otaclient.stop() ;
    return res ;    
  }
  if ( updlen > 0 )
  {
    if ( do_software_update ( updlen, command ) )               // Flash updated sketch
    {
      nvssetstr ( lstmodkey, newlstmod ) ;                      // Update Last Modified (version) in NVS
      res = true ;
    }
  }
  else
  {
    dbgprint ( "Er was geen inhoud in het antwoord!" ) ;
    otaclient.stop() ;
  }
  return res ;
}


//**************************************************************************************************
//                              R E A D _ P 1 _ D O N G L E _H T T P                               *
//**************************************************************************************************
// Read data from P1 dongle over http.                                                             *
// Er wordt een enkele API call gedaan.  Dit levert in de meeste gevallen een JSON structuur op.   *
// Die wordt opgeslagen in json_doc.                                                               *
//**************************************************************************************************
bool read_p1_dongle_http()
{
  String               line ;                                   // Input header line
  uint32_t             tmpl ;                                   // Mogelijke input lengte
  uint32_t             jlen = 0 ;                               // Input length
  int                  bread ;                                  // Number of bytes read
  int                  trycount = 4 ;                           // Aantal connect pogingen
  DeserializationError jsonerr ;

  if ( ! NetworkFound )                                         // Hebben we toegang tot netwerk?
  {
    return false ;                                              // Nee, dan maar niet
  }
  //dbgprint ( "Verbinden met %s, port %d for %s",
  //          dongle_host.c_str(),
  //          dongle_port,
  //          dongle_api.c_str() ) ;
  while ( ! P1_client.connect ( dongle_host.c_str(),            // connect to dongle
                                dongle_port, 5000 ) )           // Time-out op 5 seconden
  {
    if ( --trycount == 0 )                                      // Genoeg geprobeerd?
    {
      dbgprint ( "Verbinden met P1-dongle mislukt!" ) ;         // Ja, geef op
      return false ;
    }
  }
  P1_client.printf ( "GET %s HTTP/1.1\r\n"                      // Gelukt, gebruik API
                     "Host: %s\r\n"
                     "Cache-Control: no-cache\r\n"
                     "Connection: close\r\n\r\n",
                    dongle_api.c_str(), dongle_host.c_str() ) ;
  uint32_t  timeout = millis() ;                                // To detect time-out
  while ( P1_client.available() == 0 )                          // Wait until response appears
  {
    if ( millis() - timeout > 5000 )
    {
      dbgprint ( "Verbinden met P1-dongle time-out!" ) ;
      P1_client.stop() ;
      return false ;
    }
  }
  // Connected, handle response
  while ( P1_client.available() )
  {
    line = P1_client.readStringUntil ( '\n' ) ;                 // Read a line from response
    line.trim() ;                                               // Remove garbage
    //dbgprint ( "line = %s", line.c_str() ) ;                  // Debug info
    if ( !line.length() )                                       // End of headers?
    {
      break ;                                                   // Yes, get the read data started
    }
    // Check if the HTTP Response is 200.  Any other response is an error.
    if ( line.startsWith ( "HTTP/1.1" ) )                       // 
    {
      if ( line.indexOf ( " 200 " ) < 0 )
      {
        dbgprint ( "req = GET %s Host: %s", 
                    dongle_api.c_str(), dongle_host.c_str() ) ;
        dbgprint ( "line = %s", line.c_str() ) ;                // Debug info
        dbgprint ( "P1, kreeg een non-200 status terug van de api!" ) ;
        P1_client.stop() ;
        return false ;
      }
    }
    if ( ( tmpl = scan_content_length ( line.c_str() ) ) )     // Scan for content_length
    {
      jlen = tmpl ;                                            // Found length
    }
  }
  // End of headers reached
  if ( ( jlen > 0 ) && ( jlen <= ( sizeof(json_buf) -2 ) ) )
  {
    bread = P1_client.readBytes ( (uint8_t*)json_buf,           // Lees de JSON string
                                   sizeof(json_buf) ) ;
    P1_client.stop() ;
    //dbgprint ( "json length is %d", bread ) ;
    if ( bread >= jlen )
    {
      json_buf[bread] = '\0' ;                                  // Zorg voor delimeter
      json_begin = strstr ( json_buf, "{" ) ;                   // Wijs naar begin van de json string
      if ( json_begin == nullptr )
      {
        dbgprint ( "JSON begint niet met een acculade!" ) ;     // Geen begin gevonden
        return false ;
      }
      jsonerr = deserializeJson ( json_doc, json_begin ) ;      // Decodeer json
      if ( jsonerr != DeserializationError::Ok )
      {
        dbgprint ( "P1 JSON codering fout %d, lengte is %d!",
                   jsonerr, strlen ( json_begin ) ) ;
        return false ;
      }
      //dbgprint ( "JSON okay" ) ;
    }
    else
    {
      dbgprint ( "P1 read lengte is %d, verwacht %d!",
                 bread, jlen ) ;
      return false ;
    }
  }
  else
  {
    dbgprint ( "P1 onverwachte inhoud in antwoord!" ) ;
    P1_client.stop() ;
    return false ;
  }
  return true ;
}


//**************************************************************************************************
//                                          T I M E R 1 S E C                                      *
//**************************************************************************************************
// Timing.  Called every second.                                                                   *
// Note that calling timely procedures within this routine or in called functions will             *
// cause a crash!                                                                                  *
//**************************************************************************************************
void IRAM_ATTR timer1sec()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE ;
  uint32_t   bits = readP1taskbit | savetaskbit ;       // Events die elke seconde wordt geset
  
  seconds++  ;                                          // Update seconds counter
  if ( ++timeinf.tm_sec >= 60 )                         // Update number of seconds
  {
    timeinf.tm_sec = 0 ;                                // Wrap after 60 seconds
    if ( ++timeinf.tm_min >= 60 )
    {
      timeinf.tm_min = 0 ;                              // Wrap after 60 minutes
      if ( ++timeinf.tm_hour >= 24 )
      {
        timeinf.tm_hour = 0 ;                           // Wrap after 24 hours
      }
    }
  }
  xEventGroupSetBitsFromISR ( taskEvents,               // Set the event bits
                              bits,
                              &xHigherPriorityTaskWoken ) ;
  // Behandel de watchdog
  if ( wdcount++ > 1800 )                               // No watchdog feed for half an hour?
  {
    resetreq = true ;                                   // Alarm situation, request reset
    if ( wdcount > 1805 )                               // Still interrupts after 5 seconds reset?
    {
      esp_restart() ;                                   // Yes, try this reset function
    }
  }
}


//**************************************************************************************************
//                                          T I M E R 1 0 0                                        *
//**************************************************************************************************
// Called every 100 msec on interrupt level, so must be in IRAM and no lengthy operations          *
// allowed.                                                                                        *
//**************************************************************************************************
void IRAM_ATTR timer100()
{
  static int16_t count = 0 ;                      // Counter for timing 1 second
  if ( ++count == 10  )                           // 1 second passed?
  {
    timer1sec() ;                                 // Yes, call 1 second function
    count = 0 ;                                   // Yes, reset count
  }
}


//**************************************************************************************************
//                                          W D G F E E D                                          *
//**************************************************************************************************
// Feed the watchdog.                                                                              *
//**************************************************************************************************
void wdgfeed()
{
  wdcount = 0 ;                                   // Reset the counter
}


//**************************************************************************************************
//                            B U B B L E S O R T K E Y S                                          *
//**************************************************************************************************
// Bubblesort the nvskeys.                                                                         *
//**************************************************************************************************
void bubbleSortKeys ( uint16_t n )
{
  uint16_t i, j ;                                             // Indexes in nvskeys
  char     tmpstr[32] ;                                       // Temp. storage for a key

  for ( i = 0 ; i < n - 1 ; i++ )                             // Examine all keys
  {
    for ( j = 0 ; j < n - i - 1 ; j++ )                       // Compare to following keys
    {
      if ( strcmp ( nvskeys[j], nvskeys[j + 1] ) > 0 )        // Next key out of order?
      {
        strcpy ( tmpstr, nvskeys[j] ) ;                       // Save current key a while
        strcpy ( nvskeys[j], nvskeys[j + 1] ) ;               // Replace current with next key
        strcpy ( nvskeys[j + 1], tmpstr ) ;                   // Replace next with saved current
      }
    }
  }
}


//**************************************************************************************************
//                                      F I L L K E Y L I S T                                      *
//**************************************************************************************************
// File the list of all relevant keys in NVS.                                                      *
// The keys will be sorted.                                                                        *
//**************************************************************************************************
void fillkeylist()
{
  nvs_iterator_t   it ;                                         // Iterator for NVS
  nvs_entry_info_t info ;                                       // Info in entry
  uint16_t         nvsinx = 0 ;                                 // Index in nvskey table

  nvs_entry_find ( "nvs", NAME, NVS_TYPE_ANY, &it ) ;           // Get first entry
  while ( it )
  {
    nvs_entry_info ( it, &info ) ;                              // Get info on this entry
    //dbgprint ( "%s::%s type=%d",
    //           info.namespace_name, info.key, info.type ) ;
    if ( info.type == NVS_TYPE_STR )                            // Only string are used
    {
      strcpy ( nvskeys[nvsinx], info.key ) ;                    // Save key in table
      if ( ++nvsinx == MAXKEYS )
      {
        nvsinx-- ;                                              // Prevent excessive index
      }
    }
    nvs_entry_next ( &it ) ;
  }
  nvs_release_iterator ( it ) ;                                 // Release resource
  nvskeys[nvsinx][0] = '\0' ;                                   // Empty key at the end
  //dbgprint ( "Lees %d keys van NVS", nvsinx ) ;
  bubbleSortKeys ( nvsinx ) ;                                   // Sort the keys
}


//**************************************************************************************************
//                                  H A N D L E F I L E R E A D                                    *
//**************************************************************************************************
// Transfer file van SPIFFS naar webserver client.                                                 *
//**************************************************************************************************
void handleFileRead ( AsyncWebServerRequest *request )
{
  String       ct ;                                   // Content type
  String       path ;                                 // Filename for SPIFFS
  String       reply ;                                // Reply on not file request
  const char * p ;                                    // Reply from analyzecmd
  
  path = request->url() ;                             // Path for requested filename
  //dbgprint ( "Verwerk file leesactie, file is %s",
  //           path.c_str() ) ;
  if ( path == String ( "/" ) )                       // Default is index.html
  {
    path = String ( "/index.html" ) ;                 // Select index.html
  }
  ct = getContentType ( path ) ;                      // Get content type
  if ( SPIFFS.exists ( path ) )                       // Does it exist in SPIFFS?
  {
    request->send ( SPIFFS, path, ct ) ;              // Send to client
  }
  else
  {
    ct = "text/plain" ;
    p = analyzeCmd ( path.c_str() + 1 ) ;             // No file, try to analyze
    reply = httpheader ( String ( "text/plain" ) +    // Set header for reply
            String ( p ) ) ;
  
    request->send ( 200, ct, p ) ;                    // Send reply
  }
}


//**************************************************************************************************
//                                     S C A N S E R I A L                                         *
//**************************************************************************************************
// Listen to commands on the Serial inputline.                                                     *
//**************************************************************************************************
void scanserial()
{
  static String serialcmd ;                      // Command from Serial input
  char          c ;                              // Input character
  const char*   reply = "" ;                     // Reply string from analyzeCmd
  uint16_t      len ;                            // Length of input string

  while ( SERIAL1.available() )                  // Any input seen?
  {
    c =  (char)SERIAL1.read() ;                  // Yes, read the next input character
    SERIAL1.write ( c ) ;                        // Echo
    len = serialcmd.length() ;                   // Get the length of the current string
    if ( ( c == '\n' ) || ( c == '\r' ) )
    {
      if ( len )
      {
        strncpy ( cmd, serialcmd.c_str(),
                  sizeof(cmd) ) ;
        reply = analyzeCmd ( cmd ) ;             // Analyze command and handle it
        dbgprint ( reply ) ;                     // Result for debugging
        serialcmd = "" ;                         // Prepare for new command
      }
    }
    else if ( ( c >= ' ' ) && ( c <= '~') )      // Only accept useful characters
    {
      serialcmd += c ;                           // Add to the command
    }
    else
    {
      serialcmd = "" ;                           // Bad character, reset
    }
    if ( len >= ( sizeof(cmd) - 2 )  )           // Check for excessive length
    {
      serialcmd = "" ;                           // Too long, reset
    }
  }
}


//**************************************************************************************************
//                                        W R I T E P R E F S                                      *
//**************************************************************************************************
// Update the preferences.  Called from the web interface.                                         *
// Parameter is a string with multiple HTTP key/value pairs.                                       *
//**************************************************************************************************
void writeprefs ( AsyncWebServerRequest *request )
{
  int        numargs ;                              // Number of arguments
  int        i ;                                    // Index in arguments
  String     key ;                                  // Name of parameter i
  String     contents ;                             // Value of parameter i

  nvsclear() ;                                      // Remove all preferences
  numargs = request->params() ;                     // Haal aantal parameters
  for ( i = 0 ; i < numargs ; i++ )                 // Scan de parameters
  {
    key = request->argName ( i ) ;                  // Get name (key)
    contents = request->arg ( i ) ;                 // Get value
    chomp ( key ) ;                                 // Remove leading/trailing spaces
    chomp ( contents ) ;                            // Remove leading/trailing spaces
    if ( key == "version" )                         // Skip de "version" parameter
    {
      continue ;
    }
    dbgprint ( "Verwerk NVS setting %s = %s",
               key.c_str(), contents.c_str() ) ;    // Toon POST parameter
    nvssetstr ( key.c_str(), contents ) ;           // Save new pair
    analyzeCmd ( key.c_str(), contents.c_str() ) ;
  }
  nvs_commit( nvshandle ) ;
  fillkeylist() ;                                   // Update list with keys
}


//**************************************************************************************************
//                                       R E A D P R E F S                                         *
//**************************************************************************************************
// Read the preferences and interpret the commands.                                                *
// If output == true, the key / value pairs are returned to the caller as a String.                *
// Het aantal gevonden preferences wordt in numprefs gezet.                                        *
//**************************************************************************************************
String readprefs ( bool output )
{
  uint16_t    i ;                                           // Loop control
  String      val ;                                         // Contents of preference entry
  String      cmd ;                                         // Command for analyzCmd
  String      outstr = "" ;                                 // Outputstring
  char*       key ;                                         // Point to nvskeys[i]

  i = 0 ;
  while ( *( key = nvskeys[i] ) )                           // Loop trough all available keys
  {
    val = nvsgetstr ( key ) ;                               // Read value of this key
    cmd = String ( key ) +                                  // Yes, form command
          String ( " = " ) +
          val ;
    if ( output )
    {
      outstr += String ( key ) +                            // Add to outstr
                String ( " = " ) +
                val +
                String ( "\n" ) ;                           // Add newline
    }
    else
    {
      chomp ( val ) ;                                       // Do not show comments
      dbgprint ( "Config %-15s = %s",                       // Preferences to logging
                 key, val.c_str() ) ;
      analyzeCmd ( cmd.c_str() ) ;                          // Analyze it
    }
    i++ ;                                                   // Next key
  }
  if ( i == 0 )
  {
    outstr = String ( "No preferences found.\n" ) ;
  }
  numprefs = i ;                                            // Save number of keys found
  return outstr ;
}


//***************************************************************************************************
//                                   S T R T O D A T A T Y P                                        *
//***************************************************************************************************
// Vertaal de string naar een datatype.                                                             *
//***************************************************************************************************
mb_data_t strToDataTyp ( const char* str )
{
  if ( strstr ( str, "s16" ) )                  // Converteer "xxxx" naar mb_data_t
  {
    return s16 ;
  }
  else if ( strstr ( str, "u32" ) )
  {
    return u32 ;
  }
  else if ( strstr ( str, "s32" ) )
  {
    return s32 ;
  }
  return u16 ;                                  // default is u16
}

//***************************************************************************************************
//                             H A N D L E _ M B _ M A N I P                                        *
//***************************************************************************************************
// Handling of the various commands for Modbus registerbit manipulation.                            *
// Will be called by by webinterface, so it may collide with ModBus transaction in de loop().       *
// Therefore a semaphore is used to protect this situation.                                         *
// The command is "mbgew" or "mbset", followed by an underline and the data type.                   *
// The datatype can be "u16", "s16", "u32" or "s32" for mbget or "u16" for mbset.                   *
// Examples:                                                                                        *
//  mbget_s32=32202      - Lees AC power                                                            *
//  mbset_u16=42020,1000 - Set charge power                                                         *
//***************************************************************************************************
String handle_mb_manip ( String argument, String value )
{
  uint16_t    p1 ;                                      // Value of first parameter
  int16_t     p2 ;                                      // Value of second parameter
  int         cpos ;                                    // Position of comma in value
  String      res ;                                     // Result
  mb_data_t   dt ;                                      // Datatype van het register
  int16_t     n ;                                       // Aantal woorden te lezen
  const char* TAG = "handle_mb_manip" ;                 // Tag voor debugging semaphores
  bool        r ;                                       // Resulaat van modbus transactie
  int32_t     value32 ;                                 // Gelezen waarde, 32 bits

  res = String ( "!Error" ) ;                           // Default result
  dbgprint ( "ModBus actie %s[%s]", argument.c_str(),
             value.c_str() ) ;
  if ( argument.length() > 8 )                          // Controleer of het een zinnig commando is
  {
    p1 = value.toInt() ;                                // Get value of first parameter
    dt = strToDataTyp ( argument.c_str() + 6 ) ;        // Bepaal het type register
    n = 1 ;                                             // Meestal 1 woord van 16 bits
    if ( ( dt == u32 ) || ( dt == s32 ) )               // Dubbel word?
    {
      n = 2 ;                                           // Ja, lees 2 woorden van 16 bits
    }
    if ( argument.startsWith ( "mbget_" ) )             // Is it Get register?
    {
      claimMb ( TAG ) ;                                 // Access modbus hardware
      r = modbus->readHRegisters ( p1, n  ) ;           // Read inputregister(s) from Venus
      if ( r )                                          // Resultaat goed?
      {
        switch ( dt )
        {
          case u16 :
            value32 = modbus->getResponse_u16() ;       // Converteer u16 naar int32
            break ;
          case s16 :
            value32 = modbus->getResponse_s16() ;       // Converteer s16 naar int32
            break ;
          case s32 :
            value32 = modbus->getResponse_s32() ;       // Converteer s32 naar int32
            break ;
          case u32 :
            value32 = modbus->getResponse_u32() ;
            break ;
        }
        res = String ( value32 ) ;                      // Converteer naar een string
      }
      releaseMb() ;                                     // Geef ModBus weer vrij
    }
    else if ( argument == "mbset_u16" )                 // Is it set register?
    {
      cpos = value.indexOf ( "," ) ;                    // Yes, get position of comma
      if ( cpos > 0 )                                   // Check position
      {
        value = value.substring ( cpos + 1 ) ;          // Get second parameter
        p2 = value.toInt() ;                            // Get value, value to be stored
        claimMb ( TAG ) ;                               // Access modbus hardware
        r = modbus->writeHRegister ( p1, p2 ) ;         // Schrijf naar ModBus
        if ( r )                                        // Resultaat goed?
        {
          res = String ( "Okay" ) ;                     // Good result
        }
        releaseMb() ;                                   // Geef weer vrij
      }
    }
  }
  return res ;
}


//**************************************************************************************************
//                                   M Q T T E X T R A S                                           *
//**************************************************************************************************
// Zet extra registers in de rtdata tabel.                                                         *
//**************************************************************************************************
void mqttExtras ( int key, String &value )
{
  uint16_t  inx1, inx2 ;                                  // Indexen in value
  uint16_t  regnr ;                                       // Modbus registernummer
  String    rtype ;                                       // Registertype als string
  mb_data_t mbdtype ;                                     // Idem als mb_data_t
  String    naam ;                                        // Naam van de JSON variabele

  inx1 = value.indexOf ( "," ) ;                          // Zoek eerste scheidingsteken
  if ( inx1 >= 0 )                                        // Gevonden?
  {
    regnr = mqtt.substring ( 0, inx1++ ).toInt() ;        // Ja, zet registernummer klaar
    inx2 = value.indexOf ( ",", inx1 ) ;                  // Zoek 2e scheidingsteken
    if ( ( inx2 - inx1 ) == 3 )                           // Gevonden?
    {
      rtype = value.substring ( inx1, inx2++ ) ;          // Ja, haal datatype, bijv. "s16"
      mbdtype = strToDataTyp ( rtype.c_str() ) ;          // Converteer naar mb_data_t
      naam = value.substring ( inx2 ) ;                   // Laatste deel is naam
      if ( ( naam.length() > 0 ) &&                       // Redelijke naam?
           ( naam.length() <= MAXJSONNAMESIZE ) )
      {
        rtdata[key].mb_regnr = regnr ;                    // Zet in rtdata
        rtdata[key].mb_data_type = mbdtype ;
        sprintf ( rtdata[key].name, "%s", naam.c_str() ) ;
      }
    }
  }
}



//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient or SERIAL1.                              *
// Version for handling string with: <parameter>=<value>                                           *
//**************************************************************************************************
const char* analyzeCmd ( const char* str )
{
  char*        value ;                           // Points to value after equalsign in command
  const char*  res ;                             // Result of analyzeCmd

  value = strstr ( str, "=" ) ;                  // See if command contains a "="
  if ( value )
  {
    *value = '\0' ;                              // Separate command from value
    res = analyzeCmd ( str, value + 1 ) ;        // Analyze command and handle it
    *value = '=' ;                               // Restore equal sign
  }
  else
  {
    res = analyzeCmd ( str, "0" ) ;              // No value, assume zero
  }
  return res ;
}


//**************************************************************************************************
//                                     A N A L Y Z E C M D                                         *
//**************************************************************************************************
// Handling of the various commands from remote webclient or serial.                               *
// par holds the parametername and val holds the value.                                            *
// "wifi_n" may appear more than once, like wifi_1, wifi_2, etc.                                   *
// Examples with available parameters:                                                             *
//   dongle                       // Naam (model) van de dongle                                    *
//   dongle_host                  // IP of hostnaam van de P1 dongle                               *
//   dongle_port                  // Poort voor P1 request, default 80                             *
//   dongle_api                   // api van de P1 dongle                                          *
//   dongle_schaal                // 1000 voor kW, 1 voor Watt in de API                           *
//   field_pin                    // Field/topic Power delivered voor JSON/MQTT                    *
//   field_pout                   // Field/topic Power returned voor JSON/MQTT                     *
//   mqtt                         // User, password en broker-IP van MQTT energiemeter             *
//   mqtt_extra<n>                // Voeg extra ModBus registers toe aan MQTT output.              *
//   mode                         // Control mode at startup, default "nom".                       *
//   nospace                      // Vervang spaties in MQTT keys door underlines.                 *
//   test                         // For test purposes                                             *
//   simul                        // 1 for modbus simulation                                       *
//   reset                        // Restart the ESP32                                             *
//   updhost                      // Set host for Update of ESP32 software.                        *
//   update_f                     // Request OTA update firmware from host                         *
//   update_w                     // Request OTA update webinterface (SPIFFS)                      *
//   v_firmware                   // Timestamp last ESP32 firmware                                 *
//   v_webintf                    // Timestamp last ESP32 webinterface in SPIFFS                   *
//   venus_mb_adr                 // ModBus adres van de batterij, default is 1                    *
//   rs485_pin_rx                 // RX pin used for serial communication with RS485 bus.          * 
//   rs485_pin_tx                 // TX pin used for serial communication with RS485 bus.          * 
//   rs485_pin_de                 // DE pin used for serial communication with RS485 bus.          * 
//   mbget_xxx                    // Get Modbus real-time register. Example: mbget_u32=2202        *
//   mbset_xxx                    // Set Modbus R/W register. Example: mbset_u32=41200,0           *
//   mincharge                    // Minimale laad/ontlaad vermogen, default 100W.                 *
//   maxcharge                    // Maximale laad vermogen, default 2500W.                        *
//   appassword                   // Password indie AP wordt aangemaakt, default is nodename.      *
//**************************************************************************************************
const char* analyzeCmd ( const char* par, const char* val )
{
  String             argument ;                       // Argument as string
  String             value ;                          // Value of an argument as a string
  int                ivalue ;                         // Value of argument as an integer
  static char        reply[180] ;                     // Reply to client, will be returned
  String             tmpstr ;                         // Temporary
  
  strcpy ( reply, "Command accepted" ) ;              // Default reply
  argument = String ( par ) ;                         // Get the argument
  chomp ( argument ) ;                                // Remove comment and useless spaces
  if ( argument.length() == 0 )                       // Lege commandline (comment)?
  {
    return reply ;                                    // Ignore
  }
  argument.toLowerCase() ;                            // Force to lower case
  value = String ( val ) ;                            // Get the specified value
  chomp ( value ) ;                                   // Remove comment and extra spaces
  ivalue = value.toInt() ;                            // Also as an integer
  if ( value.length() == 0 )
  {
    dbgprint ( "Commando: %s (zonder parameter)",
               argument.c_str() ) ;
  }
  // Behandle de parameters
  if ( argument.startsWith ( "mb" ) )                 // Modbus manupilation?
  {
    tmpstr = handle_mb_manip ( argument, value ) ;    // Yes, call manupulation function
    strcpy ( reply, tmpstr.c_str() ) ;                // Set reply
  }
  else if ( argument == "dongle" )                    // Type dongle?
  {
    dongle = value ;                                  // Yes set hostname/IP adres
  }
  else if ( argument == "dongle_host" )               // Host specification van P1 dongle?
  {
    dongle_host = value ;                             // Yes set hostname/IP adres
  }
  else if ( argument == "dongle_port" )               // Port specification van P1 dongle?
  {
    dongle_port = ivalue ;                            // Yes set port
  }
  else if ( argument == "dongle_api" )                // api specification van P1 dongle?
  {
    dongle_api = value ;                              // Yes set hostname/IP adres
  }
  else if ( argument == "dongle_schaal" )             // api factor voor omrekenen naar Watt?
  {
    dongle_schaal = ivalue ;                          // Ja, set factor 1000 of 1
  }
  else if ( argument == "mqtt" )                      // Gegevens voor MQTT?
  {
    mqtt = value ;                                    // Ja, zet gegevens klaar
  }
  else if ( argument.startsWith ( "mqtt_extra_" ) )   // Extra ModBus registers voor MQTT output?
  {
    int key = argument.substring(11).toInt() & 7 ;    // Ja, bepaal plaats in tabel 0..7 relatief
    key += (int)CF1 ;                                 // Plaats absoluut
    mqttExtras ( key, value ) ;                       // Zet register in rtdata
  }
  else if ( argument == "field_pin" )                 // JSON item of MQTT topic voor power delivered?
  {
    field_pIn = value ;                               // Ja, zet veldnaam klaar
  }
  else if ( argument == "field_pout" )                // JSON item of MQTT topic voor power returned?
  {
    field_pOut = value ;                              // Ja, zet veldnaam klaar
  }
  else if ( argument == "updhost" )                   // Updatehost specification?
  {
    updhost = value ;                                 // Yes set hostname for updates
  }
  else if ( argument == "update_f" )                  // Update firmware request?
  {
    updatefirmware = true ;                           // Yes, set request flag
  }
  else if ( argument == "update_w" )                  // Update web interface (SPIFFS) firmware request?
  {
    updatewebintf = true ;                            // Yes, set request flag
  }
  else if ( argument.startsWith ( "reset" ) )         // Reset request?
  {
    resetreq = true ;                                 // Reset all
  }
  else if ( argument == "venus_mb_adr" )              // ModBus address of the battery?
  {
    venus_mb_adr = ivalue ;                           // Yes, set ModBus address
  }
  else if ( argument == "rs485_pin_rx" )              // RX pin for communication with RS485 bus?
  {
    RS485_pin_rx = ivalue ;                           // Yes, set pinnumber
  }
  else if ( argument == "rs485_pin_tx" )              // TX pin for communication with RS485 bus?
  {
    RS485_pin_tx = ivalue ;                           // Yes, set pinnumber
  }
  else if ( argument == "rs485_pin_de" )              // DE pin for communication with RS485 bus?
  {
    RS485_pin_de = ivalue ;                           // Yes, set pinnumber
  }
  else if ( argument == "simul" )                     // Modbus simulation?
  {
    simul = ( ivalue == 1 ) ;                         // Yes, set accordingly
  }
  else if ( argument == "mincharge" )                 // Minimaal laad/ontlaad vermogen?
  {
    if ( ivalue > 25 )                                // Ja, niet al te gek?
    {
      minCharge = ivalue ;                            // Redelijke waarde overnemen
    }
  }
  else if ( argument == "maxcharge" )                 // Maximale laad/ontlaad vermogen?
  {
    if ( ( ivalue <= 2500 ) && ( ivalue >= 250 ) )    // Ja, niet al te gek?
    {
      maxCharge = ivalue ;                            // Redelijke waarde overnemen
    }
  }
  else if ( argument == "appassword" )                // Password bij accesspoint mode
  {
    APpassword = value ;                              // Ja, neem over
  }
  else if ( argument == "mode" )                      // Mode bij startup?
  {
    if ( strstr ("stb,nom,off,man", value.c_str()) )  // Zinnige waarde?
    {
      sprintf ( controlmod, "%s", value.c_str() ) ;   // Ja, zet de gewenste control mode
    }
  }
  else if ( argument == "nospaces" )                  // Spaties in keywords JSON ongewenst?
  {
    if ( ivalue != 0 )                                // Ja, optie op 1 gezet?
    {
      jsonReplaceSpaces() ;                           // Ja, vervang de spaties
    }
  }
  else if ( argument == "test" )                      // Test command
  {
    int i = 0 ;                                       // Index in rtdata
    while ( rtdata[i].mb_data_type != eot )
    {
      dbgprint ( "Modbus reg [%-20s] is %d",
                 rtdata[i].name, rtdata[i].value ) ;
      i++ ;
    }
    dbgprint ( "Stack MBtask      is %4d", uxTaskGetStackHighWaterMark ( xMBtask ) ) ;
    dbgprint ( "Stack readP1task  is %4d", uxTaskGetStackHighWaterMark ( xreadP1task ) ) ;
    dbgprint ( "Stack controltask is %4d", uxTaskGetStackHighWaterMark ( xcontroltask ) ) ;
    dbgprint ( "Stack savetask    is %4d", uxTaskGetStackHighWaterMark ( xsavetask ) ) ;
    sprintf ( reply,
              "Vrij geheugen is %d, "
              "Stack hoofdprogramma is %4d, "
              "Foutteller is %d, "
              "Watchdog teller is %d, max 1800, "
              "Modbus communicatie is %s",
              ESP.getFreeHeap(),
              uxTaskGetStackHighWaterMark ( xmaintask ),
              errorcount,
              wdcount,
              modbusOkay ? "okay" : "fout" ) ;
  }
  else
  {
    sprintf ( reply, "%s called with illegal parameter: %s",
              NAME, argument.c_str() ) ;
  }
  return reply ;                                      // Return reply to the caller
}


//**************************************************************************************************
//                                     H T T P H E A D E R                                         *
//**************************************************************************************************
// Set http headers to a string.                                                                   *
//**************************************************************************************************
String httpheader ( String contentstype )
{
  return String ( "HTTP/1.1 200 OK\nContent-type:" ) +
         contentstype +
         String ( "\n"
                  "Server: " NAME "\n"
                  "Cache-Control: " "max-age=3600\n"
                  "Last-Modified: " VERSION "\n\n" ) ;
}


//**************************************************************************************************
//                                     G E T C O N T E N T T Y P E                                 *
//**************************************************************************************************
// Returns the contenttype of a file to send.                                                      *
//**************************************************************************************************
String getContentType ( String filename )
{
  if      ( filename.endsWith ( ".html" ) ) return "text/html" ;
  else if ( filename.endsWith ( ".png"  ) ) return "image/png" ;
  else if ( filename.endsWith ( ".gif"  ) ) return "image/gif" ;
  else if ( filename.endsWith ( ".jpg"  ) ) return "image/jpeg" ;
  else if ( filename.endsWith ( ".ico"  ) ) return "image/x-icon" ;
  else if ( filename.endsWith ( ".css"  ) ) return "text/css" ;
  else if ( filename.endsWith ( ".zip"  ) ) return "application/x-zip" ;
  else if ( filename.endsWith ( ".gz"   ) ) return "application/x-gzip" ;
  else if ( filename.endsWith ( ".mp3"  ) ) return "audio/mpeg" ;
  else if ( filename.endsWith ( ".js"   ) ) return "application/javascript" ;
  else if ( filename.endsWith ( ".pw"   ) ) return "" ;              // Passwords are secret
  return "text/html" ;
}


//**************************************************************************************************
//                                      N V S I N I T                                              *
//**************************************************************************************************
bool nvsinit()
{
  esp_err_t err = nvs_flash_init() ;
  
  if ( err == ESP_ERR_NVS_NO_FREE_PAGES ||
       err == ESP_ERR_NVS_NEW_VERSION_FOUND )
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    dbgprint ( "NVS flash initialisatie is niet in orde!" ) ;
    err = nvs_flash_erase() ;
    if ( err != ESP_OK )
    {
      dbgprint ( "nvs_flash_erase fout!" ) ;
    } 
    err = nvs_flash_init() ;
    if ( err != ESP_OK )
    {
      dbgprint ( "nvs_flash_init fout!" ) ;
    } 
  }
  return ( err == ESP_OK ) ;
}


//**************************************************************************************************
//                                      N V S O P E N                                              *
//**************************************************************************************************
// Open Preferences with my-app namespace. Each application module, library, etc.                  *
// has to use namespace name to prevent key name collisions. We will open storage in               *
// RW-mode (second parameter has to be false).                                                     *
//**************************************************************************************************
void nvsopen()
{
  if ( ! nvshandle )                                         // Opened already?
  {
    nvserr = nvs_open ( NAME, NVS_READWRITE, &nvshandle ) ;  // No, open nvs
    if ( nvserr )
    {
      dbgprint ( "nvs_open fout!" ) ;
    }
  }
}


//**************************************************************************************************
//                                      N V S C L E A R                                            *
//**************************************************************************************************
// Clear all preferences.                                                                          *
//**************************************************************************************************
esp_err_t nvsclear()
{
  nvsopen() ;                                         // Be sure to open nvs
  return nvs_erase_all ( nvshandle ) ;                // Clear all keys
}


//**************************************************************************************************
//                                      N V S G E T S T R                                          *
//**************************************************************************************************
// Read a string from nvs.                                                                         *
//**************************************************************************************************
String nvsgetstr ( const char* key )
{
  static char   nvs_buf[NVSBUFSIZE] ;       // Buffer for contents
  size_t        len = NVSBUFSIZE ;          // Max length of the string, later real length

  nvsopen() ;                               // Be sure to open nvs
  nvs_buf[0] = '\0' ;                       // Return empty string on error
  nvserr = nvs_get_str ( nvshandle, key, nvs_buf, &len ) ;
  if ( nvserr )
  {
    dbgprint ( "nvs_get_str fout %X voor key %s, keylengte is %d, lengte is %d!",
               nvserr, key, strlen ( key), len ) ;
    dbgprint ( "Inhoud: %s", nvs_buf ) ;
  }
  return String ( nvs_buf ) ;
}


//**************************************************************************************************
//                                      N V S S E T S T R                                          *
//**************************************************************************************************
// Put a key/value pair in nvs.  Length is limited to allow easy read-back.                        *
// No writing if no change.                                                                        *
//**************************************************************************************************
esp_err_t nvssetstr ( const char* key, String val )
{
  String curcont ;                                         // Current contents
  bool   wflag = true  ;                                   // Assume update or new key

  //dbgprint ( "Setstring voor %s: %s", key, val.c_str() ) ;
  if ( val.length() >= NVSBUFSIZE )                        // Limit length of string to store
  {
    dbgprint ( "nvssetstr lengte te groot!" ) ;
    return ESP_ERR_NVS_NOT_ENOUGH_SPACE ;
  }
  if ( nvssearch ( key ) )                                 // Already in nvs?
  {
    curcont = nvsgetstr ( key ) ;                          // Read current value
    wflag = ( curcont != val ) ;                           // Value change?
  }
  if ( wflag )                                             // Update or new?
  {
    nvserr = nvs_set_str ( nvshandle, key, val.c_str() ) ; // Store key and value
    if ( nvserr )                                          // Check error
    {
      dbgprint ( "nvssetstr fout!" ) ;
    }
  }
  return nvserr ;
}


//**************************************************************************************************
//                                      N V S S E A R C H                                          *
//**************************************************************************************************
// Check if key exists in nvs.                                                                     *
//**************************************************************************************************
bool nvssearch ( const char* key )
{
  size_t        len = NVSBUFSIZE ;                      // Length of the string

  nvsopen() ;                                           // Be sure to open nvs
  nvserr = nvs_get_str ( nvshandle, key, NULL, &len ) ; // Get length of contents
  return ( nvserr == ESP_OK ) ;                         // Return true if found
}


//**************************************************************************************************
//                                    I N I T P R E F S                                            *
//**************************************************************************************************
// Initialiseer de NVS met defaults voor enkele parameters.                                        *
// Dit gebeurt alleen bij de eerste start, indien de NVS nog helemaal leeg is.                     *
// Bij het flashen kan er in de "coredump" partitie al een key voor de wifi credentials aanwezig   *
// zijn.  Die nemen we dan over, de credentials staan vanaf positie 16.                            *
//**************************************************************************************************
void initprefs()
{
  char        coredumpbuf[128] ;                                // Ruimte voor data uit coredump partition
  String      buf = emptyWifiCred ;                             // "SSID/password"

  nvsclear() ;                                                  // Erase all keys
  nvssetstr ( "v_firmware",   ""                  ) ;
  nvssetstr ( "v_webintf",    ""                  ) ;
  nvssetstr ( "dongle",       "P1_Dongle_Pro"     ) ;           // Als voorbeeld een P1_Dongle_pro
  nvssetstr ( "dongle_host",  "192.168.1.54"      ) ;           // Met een voorbeeld IP adres
  if ( coredump != nullptr )                                    // Is er een coredump partitie?
  {
    if ( esp_partition_read ( coredump, 0, coredumpbuf,         // Ja, lees een deel van die partitie
                              sizeof(coredumpbuf) ) == ESP_OK )
    {
      if ( strcmp ( coredumpbuf, CDSECRET ) == 0 )              // Staan er credentials in coredump (van Esptool)?
      {
        buf = String ( coredumpbuf + 16 ) ;                     // Ja, probeer die
        dbgprint ( "WiFi credentials %s gevonden",              // Toon in logging
                   coredumpbuf + 16 ) ;
      }
    }
  }
  nvssetstr ( "wifi", buf ) ;
}


//**************************************************************************************************
//                                  H A N D L E _ G E T S T A T U S                                *
//**************************************************************************************************
// Called from index page to display proces data.                                                  *
// De string met JSON wordt in een buffer gezet die exclusief voor deze functie is.                *
//**************************************************************************************************
void handle_getStatus ( AsyncWebServerRequest *request )
{
  static char bjstat[512] ;                                 // Buffer met JSON string 

  //dbgprint ( "HTTP get status" ) ;                        // Show request
  buildJSON ( bjstat, sizeof(bjstat ) ) ;                   // Format de JSON
  request->send ( 200, "text/json", bjstat ) ;              // Stuur als reply
}


//**************************************************************************************************
//                                  H A N D L E _ G E T J S O N                                    *
//**************************************************************************************************
// Called from index page to display last json from dongle.                                        *
//**************************************************************************************************
void handle_getjson ( AsyncWebServerRequest *request )
{
  //dbgprint ( "HTTP get json" ) ;                          // Show request
  request->send ( 200, "text/json", json_begin ) ;          // Send the reply
}


//**************************************************************************************************
//                                  H A N D L E _ G E T G R A P H                                  *
//**************************************************************************************************
// Called from statistics page to display statistic data.                                          *
//**************************************************************************************************
void handle_getgraph ( AsyncWebServerRequest *request )
{
  AsyncWebServerResponse *response ;                        // Response on request
  uint16_t               statslen ;                         // Lengte van de te versturen data
  String                 encoded ;                          // Result base64 encode
  base64                 b64c ;                             // Base64 object
  String                 graphName ;                        // Naam van de gewenste grafiek
  const uint8_t*         graph ;                            // Wijs naar 1 van de 3 grafieken
  int                    inx ;                              // Index in één van de 3 grafieken
  int                    tinx = SEC10 ;                     // SEC10, MIN1, MIN10 of UUR1
  uint64_t               t ;                                // Tijd van laatste meting

  graphName =request->arg ( "t" ) ;                         // Welke grafiek is gewenst?
  //dbgprint ( "HTTP get graph %s", graphName.c_str() ) ;   // Show request
  if ( graphName == "10sec" )                               // 100 x 10 seconden?
  {
    tinx = SEC10 ;                                          // Het gaat om deze grafiek
  }
  else if ( graphName == "1min" )                           // 100 x 1 minuut?
  {
    tinx = MIN1 ;                                           // Het gaat om deze grafiek
  }
  else if ( graphName == "10min" )                          // 100 x 10 minuten?
  {
    tinx = MIN10 ;                                          // Het gaat om deze grafiek
  }
  else if ( graphName == "1uur" )                           // 100 x 1 uur?
  {
    tinx = UUR1 ;                                           // Het gaat om deze grafiek
  }
  t = (uint64_t) statistics.lasttime[tinx] ;                // Pak tijdstip laatste meting
  graph = (const uint8_t*)statistics.soort[tinx].buf ;      // Pointer naar de juiste grafiek
  inx = statistics.soort[tinx].inx ;                        // Pak bijbehorende index
  statslen = sizeof(stats2_t) * STATSIZ ;                   // Lengte van de te coderen data
  encoded = b64c.encode ( graph,                            // Encode de data
                          statslen ) ;
  response = request->beginResponse (                       // Definieer de response
                        200,                                // Status
                        "text/plain",
                        encoded ) ;
  response->addHeader ( "Index", inx ) ;                    // Current index in array
  response->addHeader ( "tijd", String ( t ) ) ;            // Voeg tijdstip laaste meteing toe
  request->send ( response ) ;                              // Stuur de response
}


//**************************************************************************************************
//                                    H A N D L E _ G E T P R E F S                                *
//**************************************************************************************************
// Called from config page to display configuration data.                                          *
//**************************************************************************************************
void handle_getprefs ( AsyncWebServerRequest *request )
{
  String prefs ;
  
  dbgprint ( "HTTP lees configuratie" ) ;              // Show request
  prefs = readprefs ( true ) ;                         // Read preference values
  request->send ( 200, "text/plain", prefs ) ;         // Send the reply
}


//**************************************************************************************************
//                                    H A N D L E _ R E S E T                                      *
//**************************************************************************************************
// Called from config page to reset the ESP32.                                                     *
//**************************************************************************************************
void handle_reset ( AsyncWebServerRequest *request )
{
  String reply = "Reset request accepted" ;            // Default reply

  dbgprint ( "HTTP reset" ) ;                          // Show request
  resetreq = true ;                                    // Set reset request flag
  request->send ( 200, "text/plain", reply ) ;         // Send the reply
}


//**************************************************************************************************
//                                    H A N D L E _ S A V E P R E F S                              *
//**************************************************************************************************
// Called from config page to save configuration data.                                             *
//**************************************************************************************************
void handle_saveprefs ( AsyncWebServerRequest *request )
{
  String reply = "Preferences saved" ;                 // Default reply
  
  dbgprint ( "HTTP bewaar configuratie" ) ;            // Show request
  writeprefs ( request ) ;                             // Write to NVS
  request->send ( 200, "text/plain", reply ) ;         // Send the reply
}


//**************************************************************************************************
//                                    H A N D L E _ S A V E M O D E                                *
//**************************************************************************************************
// Aangeroepen vanuit de "Handmatig" pagina om de instellingen te bewaren.                         *
//**************************************************************************************************
void handle_savemode ( AsyncWebServerRequest *request )
{
   const char* reply = "Instelling opgeslagen" ;              // Default reply

   dbgprint ( "HTTP mode setting" ) ;                         // Show request
   if ( request->params() != 3 )                              // Parameters aanwezig?
   {
     reply = "Error" ;                                        // Nee, dus fout
   }
   else
   {
     sprintf ( controlmod, "%s", request->arg(1).c_str() ) ;  // Bewaar mode
     man_setpoint = request->arg(2).toInt() ;                 // En setpoint
   }
   request->send ( 200, "text/plain", reply ) ;               // Send the reply
}


//**************************************************************************************************
//                                    H A N D L E _ U P D A T E                                    *
//**************************************************************************************************
// Called from config page to update the software or the webinterface (SPIFFS).                    *
//**************************************************************************************************
void handle_update ( AsyncWebServerRequest *request )
{
  const char*        reply = "Update request accepted" ;  // Default reply
  const int          parnr = 0 ;                          // Eerste parameter is interessant
  
  if ( request->params() > 0 )                            // Parameter aanwezig?
  {
    if ( request->arg ( parnr ) == "w" )                  // Webinterface update?
    {
      updatewebintf = true ;                              // Yes, set request
      dbgprint ( "HTTP update web-interface" ) ;          // Show request
    }
    else
    {
      updatefirmware = true ;                             // No, set firmware update request
      dbgprint ( "HTTP update firmware" ) ;               // Show request
    }
  }
  request->send ( 200, "text/plain", reply ) ;            // Send the reply
}


//**************************************************************************************************
//                                        C B  _ L O G G I N G                                     *
//**************************************************************************************************
// Callback function for handle_logging, will be called for every chunk to send to client.         *
// If no more data is availble, this function will return 0.                                       *
//**************************************************************************************************
size_t cb_logging ( uint8_t *buffer, size_t maxLen, size_t index )
{
  static int   i ;                                   // Index in dbglines
  static int   nrl ;                                 // Mumber of lines in dbglines
  static char  linebuf[DEBUG_BUFFER_SIZE + 20] ;     // Holds one debug line
  static char* p_in ;                                // Pointer in linebuf
  char*        p_out = (char*)buffer ;               // Fill pointer for output buffer
  const char*  s ;                                   // Single line from dbglines
  size_t       len = 0 ;                             // Number of bytes filled in buffer
  
  if ( index == 0 )                                 // First call for this page?
  {
    i = 0 ;                                         // Yes, set index
    nrl = dbglinx ;                                 // Number of lines in dbglines
    p_in = linebuf ;                                // Set linebuf to empty
    *p_in = '\0' ;
  }
  while ( maxLen-- > 0 )                            // Space for another character?
  {
    if ( *p_in == '\0' )                            // Input buffer end?
    {
      if ( i == nrl )                               // Yes, is there another line?
      {
        break ;                                     // No, end of text
      }
      s = dbglines->logregel[i++] ;                 // Yes, get next line from array
      strcpy ( linebuf, s ) ;                       // Fill linebuf
      strcat ( linebuf, "\n" ) ;                    // Add a break
      p_in = linebuf ;                              // Pointer to start of line
    }
    *p_out++ = *p_in++ ;                            // Copy next character
    len++ ;                                         // Increase result length
  }
  // We come here if output buffer is completely full or if end of dbglines is reached
  return len ;                                      // Return filled length of buffer
}


//**************************************************************************************************
//                                    H A N D L E _ L O G G I N G                                  *
//**************************************************************************************************
// Called from logging page to list the logging in dbglines.                                       *
// It will handle the chunks for the client.  The buffer is filled by the callback routine.        *
// If called with parameter "func=c", de buffer with debugline is cleared.                         *
//**************************************************************************************************
void handle_logging ( AsyncWebServerRequest *request )
{
  AsyncWebServerResponse *response ;
  const char*            reply = "No logging available" ;
  const int              parnr = 0 ;                // Eerste parameter is interessant

  //dbgprint ( "HTTP toon logging verzoek" ) ;
  if ( ! dbglines )                                 // Hebben we een buffer met logregels?
  {
    request->send ( 200, "text/plain", reply ) ;    // Nee, stuur error reply
    return ;
  }
  if ( request->params() == 1 )                     // Haal aantal parameters
  {
    String value = request->arg ( parnr ) ;         // Haal parameter op
    if ( value == "c" )                             // Parameter is een "C" (voor wissen)?
    {
      dbglinx = 0 ;                                 // Ja, begin overnieuw
      dbgprint ( "Logging is leeggemaakt" ) ;
    }
  }
  response = request->beginChunkedResponse ( "text/plain", cb_logging ) ;
  response->addHeader ( "Server", NAME ) ;
  request->send ( response ) ;
}


//**************************************************************************************************
//                               C B _ P 1 P D I S C O N N E C T                                     *
//**************************************************************************************************
// Event callback on disconnect                                                                   *
//**************************************************************************************************
void cb_P1Disconnect ( void* arg, AsyncClient* client )
{
  dbgprint ( "Verbinding mat P1-poort verbroken" ) ;
  P1_host_connected = false ;
}




//**************************************************************************************************
//                               C B _ P 1 E R R O R                                               *
//**************************************************************************************************
// Event callback on error.                                                                        *
//**************************************************************************************************
void cb_P1Error ( void* arg, AsyncClient* client, int error )
{
  dbgprint ( "TCP verbinding met P1-poort mislukt!" ) ;
  P1_host_error = true ;
  errorcount++ ;                                    // Count errors
}


//**************************************************************************************************
// Functions for Comm task.                                                                        *
//**************************************************************************************************

//**************************************************************************************************
//                            M B P R E T R A N S M I S S I O N                                    *
//**************************************************************************************************
// Call-back routine to set 485_EN before sending data to ModBus.                                  *
//**************************************************************************************************
void MBpreTransmission()
{
  digitalWrite ( RS485_pin_de, HIGH ) ;
  delayMicroseconds ( 10 ) ;   
  while ( RS485serial->available() )                    // Flush all input, normally some null chars
  {
    //int c =
    RS485serial->read ( ) ;
    //dbgprint ( "MPpre flushed 0x%02X", c ) ;
  }
}


//**************************************************************************************************
//                               M B P O S T T R A N S M I S S I O N                               *
//**************************************************************************************************
// Call-back routine to clear 485_EN after sending data to ModBus.                                 *
// By switching the enable pin, stray input characters may be seen on the serial input.            *
// We will flush these (usually 2)characters.                                                      *
//**************************************************************************************************
void MBpostTransmission()
{
  //delayMicroseconds ( 10 ) ;                                // Make sure transmission finished
  digitalWrite ( RS485_pin_de, LOW ) ;
  delay ( 2 ) ;                                               // Make sure R/T is quiet
  while ( RS485serial->available() )                          // Flush stray input
  {
    RS485serial->read() ;                                     // Usually read 0x00 character
  }
}


//**************************************************************************************************
//                                     M B T A S K                                                 *
//**************************************************************************************************
// Proces dat doorlopend de ModBus data-registers leest en schrijft.                               *
// De timing wordt doorgaans gedaan via een event vanuit de timer, elke seconde.                   *
// Als er door de controltask een niuw setpoint is bepaald, dan komt er ook een event, waardoor er *
// versneld kan worden gereageerd.                                                                 *
//**************************************************************************************************
void MBtask ( void * parameter )
{
  EventBits_t b ;                                         // Result of xEventGroupWaitBits()

  dbgTaskInfo() ;                                         // Toon info
  RS485serial = new HardwareSerial ( 2 ) ;                // Use 2nd serial for RS485 comm.
  RS485serial->begin ( RS485BR, RS485MODE,                // Initialize serial port
                       RS485_pin_rx, RS485_pin_tx ) ;     // RX and TX (cofigurable)
  pinMode ( RS485_pin_de, OUTPUT ) ;                      // Voor sturing lees/schrijf modbus
  MBpostTransmission() ;                                  // Maak ModBus inactief
  modbus = new Vmodbus() ;                                // Create modbus object
  modbus->begin ( *RS485serial ) ;                        // Set modbus serial to use
  modbus->preTransmission ( MBpreTransmission ) ;         // Set callbacks for 485_EN signal
  modbus->postTransmission ( MBpostTransmission ) ;
  while ( true )
  {
    b = xEventGroupWaitBits ( taskEvents, MBtaskbit,      // Wacht op trigger
                              pdTRUE, pdFALSE,  5000 ) ;  // Clear bits on exit, 5 seconds time-out
    setpixel ( black ) ;                                  // LED even uit
    if ( put_modbus_data_regs ( b == MBtaskbit ) )        // Schrijf de registers
    {
      setpixel ( green ) ;                                // LED weer aan
    }
    else
    {
      setpixel ( red ) ;
      delay ( 2000 ) ;                                    // Ging fout, vertraag ietwat
    }
    if ( get_modbus_data_regs() )                         // Lees de registers
    {
      setpixel ( green ) ;                                // LED weer aan
      claimData ( "readMBtask" ) ;
      rtdata[ERRCNT].value = errorcount ;                 // Neem ook error count over in de data
      releaseData() ;
    }
    else
    {
      setpixel ( red ) ;
      delay ( 2000 ) ;                                    // Ging fout, vertraag ietwat
    }
    wdgfeed() ;                                           // Voeden van de waakhond
  }
}


//**************************************************************************************************
//                                     R E A D P 1 T A S K                                         *
//**************************************************************************************************
// Proces dat de variabelen uit de dongle leest.                                                   *
// De timing wordt gedaan via een event vanuit de timer, elke seconde.                             *
//**************************************************************************************************
void readP1task ( void * parameter )
{
  uint16_t timing = INT16_MAX ;                                 // Timer voor 5 seconden
  uint16_t err_row = 0 ;                                        // Telt aantal fouten in een rij
  bool     p1_res ;                                             // Laatste resultaat uitlezen dongle
  uint16_t disconncount = 0 ;                                   // Disconnect count

  dbgTaskInfo() ;                                               // Toon info
  while ( true )
  {
    if ( xEventGroupWaitBits ( taskEvents, readP1taskbit,       // Is er een trigger?
                               pdTRUE, pdFALSE, 10 ) )          // Clear bits on exit
    {
      if ( ++timing < 5 )                                       // Tijd voor actie?
      {
        continue ;                                              // Nee, uitstellen
      }
      timing = 0 ;                                              // Ja, reset timer
      if ( simul || dongle.equalsIgnoreCase ( "P1_Simul" ) )    // Gebruik maken van een gesimuleerde dongle?
      {
        p1_res = p1_simul_handle() ;                            // Ja, genereer gesimuleerde vermogens
      }
      else if ( dongle.equalsIgnoreCase ( "P1_Dongle_Pro" ) )   // Nee, is het een P1_Dongle_Pro?
      {
        p1_res = p1_dongle_pro_handle() ;                       // Ja, haal vermogens
      }
      else if ( dongle.equalsIgnoreCase ( "Homewizard" ) )      // Nee, is het een Homewizard?
      {
        p1_res = homewizard_handle() ;                          // Ja, haal vermogens
      }
      else if ( dongle.equalsIgnoreCase ( "Shellypro3" ) )      // Nee, is het een Shelly Pro 3?
      {
        p1_res = shellypro3_handle() ;                          // Ja, haal vermogens
      }
      else if ( dongle.equalsIgnoreCase ( "P1_MQTT" ) )         // Nee, is het een energiemeter via MQTT?
      {
        p1_res = p1_mqtt_handle() ;                             // Ja, haal vermogens
      }
      else
      {
        p1_res = false ;
        dbgprint ( "Dongle %s is onbekend", dongle.c_str() ) ;
        delay ( 30000 ) ;                                       // Misschien over 30 seconden?
      }
      if ( p1_res )                                             // Dongle uitlezen gelukt?
      {
        err_row = 0 ;                                           // Ja, reset aantal fouten in een rij
        xEventGroupSetBits ( taskEvents, controltaskbit ) ;     // Start controltask
      }
      else
      {
        if ( ++err_row >= 5 )                                   // Nee, gebeurt dat vaak?
        {
          dbgprint ( "Lange storing bij lezen P1 dongle!" ) ;   // Ja, log dat
          errorcount++ ;                                        // Tel aantal fouten
        }
      }
      //dbgprint ( "Netto vermogen is %d", rtdata[PWIN].value ) ;
      wdgfeed() ;                                               // Voeden van de waakhond
      claimData ( "loop" ) ;
      rtdata[ERRCNT].value = errorcount ;                       // Neem ook error count over in de data
      releaseData() ; 
    }
    // We komen hier na een trigger, maar ook elke 10 msec
    if ( mqtt_on )
    {
      mqttclient.loop() ;                                       // Onderhouden van MQTT connectie
      if ( mqttclient.connected() )                             // Test of er een verbinding is
      {
        disconncount = 0 ;                                      // Jawel, reset counter
      }
      else
      {
        if ( ++disconncount >= 1000 )                           // Lang zonder connectie?
        {
          mqttreconnect ( false ) ;                             // Nee, reconnect
          disconncount = 0 ;                                    // Jawel, reset counter
        }
      }
    }
  }
  // Komt hier nooit
}


//**************************************************************************************************
//                                     S A V E T A S K                                             *
//**************************************************************************************************
// Proces dat de data bewaart voor de satistieken.                                                 *
// Ook wordt er data gestuurd naar MQTT indien geconfigureerd.                                     *
// De timing wordt gedaan via een event vanuit de timer, elke seconde.                             *
//**************************************************************************************************
void savetask ( void * parameter )
{
  int             tinx ;                                    // Grafiek nummer
  const  uint16_t cycletime = 10 ;                          // Cycle time in seconden
  const  int16_t  seconds[] = { 10, 60, 600, 3600 } ;       // Interval per grafiek
  uint32_t        myseconds = 0 ;                           // Interne klok
  int16_t         pIn, pOut, soc ;                          // Te registreren vermogens
  char            mqjbuf[512] ;                             // Ruimte voor geformatte JSON string

  dbgTaskInfo() ;                                           // Toon info
  while ( true )
  {
    xEventGroupWaitBits ( taskEvents, savetaskbit,          // Wacht op trigger
                          pdTRUE, pdFALSE, 10000 ) ;        // Clear bits on exit, 10 seconds time-out
    if ( ++myseconds % cycletime )                          // Tijd voor actie (10, 20, 30, enz.)?
    {
      continue ;                                            // Nee, uitstellen
    }
    if ( mqtt_on && mqtt_ok )
    {
      buildJSON ( mqjbuf, sizeof(mqjbuf) ) ;                // Format de JSON string
      if ( ! mqttclient.publish ( nodename, mqjbuf ) )      // Publiceer algemene data
      {
        dbgprint ( "MQTT publish mislukt, lengte is %d!",
                   strlen ( mqjbuf ) ) ;
        errorcount++ ;                                      // Tel de fouten
      }
      // else
      // {
      //   dbgprint ( "MQTT publish, node %s gelukt", nodename ) ;
      // }
    }
    claimData ( "savetask" ) ;
    pIn  = rtdata[PWIN].value ;                             // Gemeten (P1) netto vermogen
    pOut = ctdata[DOUT].value - ctdata[COUT].value  ;       // Uitgestuurde vermogen (één van de twee is nul)
    soc  = rtdata[SOC].value ;                              // Batterij lading in procent
    for ( tinx = SEC10 ; tinx <= UUR1 ; tinx++ )            // Ga de grafieken af
    {
      stats1_t* g = &statistics.soort[tinx] ;               // Het gaat om deze
      g->sum_pIn  += pIn ;                                  // Sommeer voor gemiddelde pIn
      g->sum_pOut += pOut ;                                 // Sommeer voor gemiddelde pOut
      g->sum_soc  += soc ;                                  // Sommeer voor gemiddelde soc
      g->sumpoints++ ;                                      // Tel aantal punten
      if ( ( myseconds % seconds[tinx] ) == 0 )             // Tijd om gemiddelde te berekenen?
      {
        g->buf[g->inx].pIn  = g->sum_pIn  / g->sumpoints ;  // Bereken gemiddelde pIn en sla op
        g->buf[g->inx].pOut = g->sum_pOut / g->sumpoints ;  // Bereken gemiddelde pOut en sla op
        g->buf[g->inx].soc  = g->sum_soc  / g->sumpoints ;  // Bereken gemiddelde soc en sla op
        g->sum_pIn  = 0 ;                                   // Reset sommen van middeling
        g->sum_pOut = 0 ;
        g->sum_soc  = 0 ;
        g->sumpoints = 0 ;                                  // Reset ook de teller
        statistics.lasttime[tinx] = time(nullptr) ;         // Zet tijdstip van de meting                      
        if ( ++g->inx >= STATSIZ )                          // Verhoog pointer
        {
          g->inx = 0 ;                                      // Round robin
        }
      }
    }
    releaseData() ;                                         // Geef datagebied weer vrij
  }
}


//**************************************************************************************************
//                                     C O N T R O L T A S K                                       *
//**************************************************************************************************
// Proces dat doorlopend de sturing voor de batterij berekent en uitstuurt.                        *
// De timing wordt gedaan via een event vanuit de P1 lees taak, ongeveer elke 5 seconden.          *
// Als dat niet gebeurt binnen 20 seconden, dan wordt stand-by geactiveerd.                        *
//**************************************************************************************************
void controltask ( void * parameter )
{
  int16_t        pIn ;                                      // Huidig grid input in Watt
  int16_t        pOut_old ;                                 // Huidig uitgestuurde vermogen
  int16_t        pOut ;                                     // Berekende uitsturing, + is discharge
  bool           stop ;                                     // True als batterij te vol of te leeg is
  EventBits_t    res ;                                      // Resutaat xEventGroupWaitBits
  bool           tasktimeout ;                              // True indien er geen startevent was

  dbgTaskInfo() ;                                           // Toon info
  while ( true )
  {
    res = xEventGroupWaitBits ( taskEvents, controltaskbit, // Wacht op trigger
                                pdTRUE, pdFALSE,  20000 ) ; // Clear bits on exit, 20 seconds time-out
    tasktimeout = ( res & controltaskbit ) == 0 ;           // Was er sprake van time-out?
    stop = false ;                                          // Neem aan dat er geen reden om te stoppen is
    claimData ( "controltask" ) ;                           // Bevries data gebied
    pOut_old = rtdata[SETBC].value ;                        // Huidige setting
    pIn = rtdata[PWIN].value ;                              // Haal huidige input van grid
    pOut = pIn + pOut_old ;                                 // Benodigde ontlaad capaciteit (indien positief)
    ctdata[CM485].value = MODBUS_ENABLE ;                   // Neem aan dat RS485 sturing mag
    // Kijk of er handmatige setting is.
    if ( strstr ( controlmod, "stb" ) )                     // Manual stand-by?
    {
      pOut = 0 ;                                            // Ja, overrule met 0 Watt
    }
    else if ( strstr ( controlmod, "man" ) )                // Manual setting?
    {
      pOut = man_setpoint ;                                 // Ja, gebruik die setting
    }
    else if ( strstr ( controlmod, "off" ) )                // Modbus mode uit?
    {
      pOut = 0 ;                                            // Ja, niet meer sturen
      ctdata[CM485].value = MODBUS_DISABLE ;                // En RS485 control uitzetten
    }
    // Indien we aan het ontladen zijn en de batterij is al bijna leeg, dan stoppen we ermee.
    // Ook als we aan het laden zijn en de batterij is al bijna vol.
    if ( ( pOut > 0 ) &&                                    // Ontladen
         ( rtdata[SOC].value <= rtdata[DISCC].value ) )     // en leeg?
    {
      sprintf ( controltext,
                "Te weinig lading in de batterij (%d%%) "
                "voor energie bijdrage (%dW)",
                rtdata[SOC].value, pOut ) ;
      stop = true ;                                         // Ja, dus stoppen
    }
    if ( ( ! stop ) && ( tasktimeout ) )                    // Time-out bij startevent?
    {
      sprintf ( controltext,
                "Stand-by ten gevolge van storing P1 dongle" ) ;
      stop = true ;                                         // Dus stoopen
    }
    if ( ( pOut < 0 ) &&                                    // Laden
         ( rtdata[SOC].value >= rtdata[CHGCC].value ) )     // en vol?
    {
      sprintf ( controltext,
                "Te veel lading in de batterij (%d%%) "
                "voor de overtollige energie (%dW)",
                rtdata[SOC].value, pOut ) ;
      stop = true ;                                         // Ja, dus stoppen
    }
    if ( ! stop )
    {
      if ( pOut > 0 )                                       // Gaan we ontladen?
      {
        if ( pOut > rtdata[MAXDIS].value )                  // Ja, meer ontladen dan limiet?
        {
          pOut = rtdata[MAXDIS].value ;                     // Ja, beperk ontladen
        }
      }
      else                                                  // We gaan laden
      {
        if ( abs ( pOut ) > rtdata[MAXCHG].value )          // Groter dan limiet?
        {
          pOut = - rtdata[MAXCHG].value ;                   // Ja, beperk laden
        }
        if ( abs ( pOut ) > maxCharge )                     // Groter dan geconfigureerde limiet?
        {
          pOut = - maxCharge ;                              // Ja, beperk laden
        }
      }
      if ( abs ( pOut ) < minCharge  )                      // Minder dan minimum laad/ontlaad vermogen?
      {
        sprintf ( controltext,
                  "Laden of ontladen met minder dan %dW "
                  "(%dW) wordt niet gedaan",
                  minCharge, pOut ) ;
        stop = true ;                                       // Dus stoppen
      }
    }
    pOut = ( pOut_old + pOut * 3 ) / 4 ;                    // Beetje filteren
    if ( stop )                                             // Stoppen met sturen?
    {
      pOut = 0 ;                                            // Ja, dus niets doen
      ctdata[DOUT].value = 0 ;                              // Niet ontladen
      ctdata[COUT].value = 0 ;                              // Niet opladen
      ctdata[FORCE].value = 0 ;                             // Schakel naar stop.
    }
    else if ( pOut <  0 )                                   // Opladen?
    {
      ctdata[COUT].value = abs ( pOut ) ;                   // Ja, set charge power
      ctdata[DOUT].value = 0 ;                              // Niet ontladen
      ctdata[FORCE].value = 1 ;                             // Schakel naar laden
      sprintf ( controltext,
                "Aan het opladen met %dW",
                abs ( pOut ) ) ;
    }
    else if ( pOut > 0 )                                    // Ontladen?
    {
      ctdata[DOUT].value = pOut ;                           // Ja, set discharge power
      ctdata[COUT].value = 0 ;                              // Niet opladen
      ctdata[FORCE].value = 2 ;                             // Schakel naar ontladen
      sprintf ( controltext,
                "Aan het ontladen met %dW", pOut ) ;
    }
    rtdata[SETBC].value = pOut ;                            // Bewaar setpoint
    releaseData() ;                                         // Geef datagebied weer vrij
    if ( pOut != pOut_old )                                 // Verschil t.o.v. vorige setting?
    {
      xEventGroupSetBits ( taskEvents, MBtaskbit ) ;        // Ja, versnel MBtask
      if ( ( pOut & 0x8000 ) ^ ( pOut_old & 0x8000 ) )      // Geschakeld van ontladen naar laden of andersom?
      {
        dbgprint ( controltext ) ;                          // Ja, meld dat in de logging
      }
    }
  }
}


//**************************************************************************************************
//                                           S E T U P                                             *
//**************************************************************************************************
// Setup for the program.                                                                          *
//**************************************************************************************************
void setup()
{
  const esp_partition_t*    spiffs = nullptr ;              // Pointer to SPIFFS partition struct
  char                      macstr[17+1] ;                  // Mac address as string
  esp_partition_iterator_t  pi ;                            // Iterator for partition_find()
  const esp_partition_t*    ps ;                            // Pointer to partition struct
  uint32_t                  psb = 0 ;                       // Wordt hoeveelheid psram

  setCpuFrequencyMhz ( 160 ) ;                              // Laat CPU niet te warm worden
  SERIAL1.begin () ;                                        // For application debug (USB)
  SERIAL1.setDebugOutput ( true ) ;                         // ESP debug to same port
  //WRITE_PERI_REG ( RTC_CNTL_BROWN_OUT_REG, 0 ) ;          // Disable brownout detector
  dbgsem = xSemaphoreCreateMutex() ;                        // Semaphore for exclusive use of dbgprint
  datasem = xSemaphoreCreateMutex() ;                       // Semaphore for exclusive use rtdata
  MBsem = xSemaphoreCreateMutex() ;                         // Semaphore for Modbus
  taskEvents = xEventGroupCreate() ;                        // Creeer eventflags voor triggering tasks
  neopixel.begin () ;                                       // Init neopixel
  for ( uint8_t i = 0 ; i < 20 ; i++ )                      // Geef tijd voor aansluiten USB kabel
  {
    setpixel ( white ) ;                                    // LED wit
    delay ( 150 ) ;
    setpixel ( green ) ;                                    // LED uit
    delay ( 150 ) ;
    setpixel ( black ) ;                                    // LED uit
    delay ( 200 ) ;
  }
  SERIAL1.println() ;
  nvsinit() ;                                               // Init nvs
  if ( ( have_psram = psramInit() ) )                       // Hebben we Psram?
  {
    psb = ESP.getPsramSize() ;                              // Haal hoeveelheid psram op (ex logregels)
    dbglines = (logregel_t*)ps_malloc(sizeof(logregel_t)) ; // Maak ruimte voor logregels in psram
  }
  dbgprint ( "Start " NAME ", version %s",                  // Show start-up info
             VERSION ) ;
  dbgprint ( "Gecompileerd: %s %s local time",              // Show compile time
             __DATE__, __TIME__ ) ;
  dbgprint ( "Beschikbare hoeveelheid Psram is %d", psb ) ; // Toon hoeveelheid Psram
  if ( statistics.valid != VALIDDATA )                      // Statistieken in RTC geheugen okay?
  {
    dbgprint ("Statistieken gewist" ) ;
    memset ( &statistics, 0, sizeof(statistics) ) ;         // Nee, maak statistieken leeg
    statistics.valid = VALIDDATA ;                          // Maak valide
  }
  xmaintask = xTaskGetCurrentTaskHandle() ;                 // My taskhandle
  if ( !SPIFFS.begin ( FSIF ) )                             // Mount and test SPIFFS
  {
    dbgprint ( "SPIFFS Mount Error!" ) ;                    // A pity...
  }
  else
  {
    dbgprint ( "SPIFFS is goed, ruimte %d, gebruikt %d",    // Show available SPIFFS space
               SPIFFS.totalBytes(),
               SPIFFS.usedBytes() ) ;
  }
  pi = esp_partition_find ( ESP_PARTITION_TYPE_ANY,         // Get partition iterator for
                            ESP_PARTITION_SUBTYPE_ANY,      // All partitions
                            nullptr ) ;
  while ( pi )
  {
    ps = esp_partition_get ( pi ) ;                       // Get partition struct
    dbgprint ( "Partitie '%-8.8s' "                       // Show partition
               "gevonden op 0x%06X, "
               "%8d bytes",
               ps->label, ps->address, ps->size ) ;
    if ( strcmp ( ps->label, "nvs" ) == 0 )               // Is this the NVS partition?
    {
      nvs = ps ;                                          // Yes, remember NVS partition
    }
    else if ( strcmp ( ps->label, "spiffs" ) == 0 )       // Is this the SPIFFS partition?
    {
      spiffs = ps ;                                       // Yes, remember SPIFFS partition
    }
    else if ( strcmp ( ps->label, "coredump" ) == 0 )     // Is this the coredump partition?
    {
      coredump = ps ;                                     // Yes, remember coredump partition
    }
    pi = esp_partition_next ( pi ) ;                      // Find next
  }
  if ( ( nvs == nullptr ) || ( spiffs == nullptr ) )      // Check essentiële partities
  {
    dbgprint ( "Partitie probleem! " ) ;                  // Zeer onwaarschijnlijk...
    while ( true ) ;                                      // Niet doorgaan
  }
  fillkeylist() ;                                         // Fill keynames with all keys
  readprefs ( false ) ;                                   // Read preferences
  delay ( 200 ) ;
  if ( numprefs == 0 )                                    // Waren er preferences?
  {
    dbgprint ( "Initialiseer configuratie" ) ;
    initprefs() ;                                         // Nee, vul met defaults
    readprefs ( false ) ;                                 // Read back
    if ( numprefs == 0 )                                  // Filled now?
    {
      dbgprint ( "Fout bij initialisatie!" ) ;            // No, report
      delay ( 10000 ) ;                                   // STOP further actions
      esp_restart() ;                                     // Probeer nog eens
    }
  }
  setWifiCred() ;                                         // Set WiFi credentials.  only 2.4 GHz accesspoints!
  WiFi.disconnect() ;                                     // After restart router could still
  delay ( 500 ) ;                                         //   keep old connection
  WiFi.mode ( WIFI_STA ) ;                                // This ESP is a station
  delay ( 500 ) ;                                         // ??
  WiFi.persistent ( false ) ;                             // Do not save SSID and password
  WiFi.onEvent ( WiFiEvent ) ;                            // Set actions on ETH events
  setpixel ( blue ) ;                                     // Show pixel blue
  esp_wifi_get_mac ( WIFI_IF_STA, mac ) ;                 // MAC address if in STA mode
  sprintf ( macstr, MACSTR,                               // Convert to a string (lower case)
            MAC2STR ( mac ) ) ;
  sprintf ( nodename, "Venus_%02X%02X",                   // Create AP node name, like "Venus_67AB"
            mac[5],
            ( mac[4] + mac[0] ) & 0xFF) ;
  dbgprint ( "Node %s, mac address is %s",                // Show nodename en mac address
             nodename, macstr ) ;
  NetworkFound = connectwifi() ;                          // Connect to WiFi network
  if ( NetworkFound )                                     // Is er een netwerk?
  {
    setpixel ( green ) ;                                  // Show pixel green
    obtain_time() ;                                       // Ja, synchroniseer de klok
  }
  int mdnsTry = 5 ;                                       // Initialize mDNS
  while ( !MDNS.begin ( nodename ) && mdnsTry-- > 0 )
  {
    dbgprint ( "Start start mdns %d", mdnsTry ) ;         // No success yet
    delay ( 1000 ) ;
  }
  dbgprint ( "Start webserver" ) ;
  cmdserver.on ( "/getprefs",     handle_getprefs ) ;     // Handle get preferences
  cmdserver.on ( "/saveprefs",    handle_saveprefs ) ;    // Handle save preferences
  cmdserver.on ( "/savemode",     handle_savemode ) ;     // Handle save manual settings
  cmdserver.on ( "/getstatus",    handle_getStatus ) ;    // Handle get status
  cmdserver.on ( "/getgraph",     handle_getgraph ) ;     // Handle get statistic graphs
  cmdserver.on ( "/logging",      handle_logging ) ;      // Handle logging
  cmdserver.on ( "/reset",        handle_reset ) ;        // Handle reset button
  cmdserver.on ( "/update",       handle_update ) ;       // Handle update software button
  cmdserver.on ( "/getjson",      handle_getjson ) ;      // Voor teste json
  cmdserver.onNotFound ( handleFileRead ) ;               // Just handle a simple page/file
  cmdserver.begin() ;                                     // Start http server
  MDNS.addService ( "http", "tcp", 80 ) ;                 // Add webinterface service
  timer = timerBegin ( 1000000 ) ;                        // Timer on 1 MHz
  timerAttachInterrupt ( timer, &timer100 ) ;             // Call timer100() on timer alarm
  timerAlarm ( timer, 100000, true, 0 ) ;                 // 100 msec, auto-reload
  if ( NetworkFound )                                     // Is er een verbinding met een Wifi netwerk?
  {                                                       // Ja, start de processen
    if ( ! mqtt.isEmpty() )                               // MQTT in gebruik?
    {
      mqtt_init() ;                                       // Ja, maak connectie met broker
    }
    xTaskCreatePinnedToCore (
      MBtask,                                             // Task to read ModBus registers.
      "MBtask",                                           // Name of task.
      3000,                                               // Stack size of task
      NULL,                                               // parameter of the task
      2,                                                  // priority of the task
      &xMBtask,                                           // Task handle to keep track of created task
      0 ) ;                                               // Run on CPU 0
    xTaskCreatePinnedToCore (
      readP1task,                                         // Task to read P1 dongle.
      "readP1task",                                       // Name of task.
      3000,                                               // Stack size of task
      NULL,                                               // parameter of the task
      2,                                                  // priority of the task
      &xreadP1task,                                       // Task handle to keep track of created task
      1 ) ;                                               // Run on CPU 1
    xTaskCreatePinnedToCore (
      controltask,                                        // Task to control the battery.
      "controltask",                                      // Name of task.
      3000,                                               // Stack size of task
      NULL,                                               // parameter of the task
      2,                                                  // priority of the task
      &xcontroltask,                                      // Task handle to keep track of created task
      0 ) ;                                               // Run on CPU 0
    xTaskCreatePinnedToCore (
      savetask,                                           // Task to save statistcal data
      "savetask",                                         // Name of task.
      3400,                                               // Stack size of task
      NULL,                                               // parameter of the task
      2,                                                  // priority of the task
      &xsavetask,                                         // Task handle to keep track of created task
      0 ) ;                                               // Run on CPU 0
  }
  dbgTaskInfo() ;                                         // Show info about main task
  delay ( 1000 ) ;                                        // 
  dbgprint ( "Einde setup") ;
}


//**************************************************************************************************
//                                           L O O P                                               *
//**************************************************************************************************
// Main loop of the program.                                                                       *
//**************************************************************************************************
void loop()
{
  if ( updatefirmware )                                   // Firmware update requested?
  {
    updatefirmware = false ;                              // Yes, clear update flag
    if ( update_software ( "v_firmware",                  // Update sketch from remote file
                           updhost.c_str(),
                           UPDATEDIR, FIRMWAREBIN,
                           U_FLASH ) )                    // Resultaat naar program partition
    {
      resetreq = true ;                                   // And reset
    }
  }
  if ( updatewebintf )                                    // Firmware update requested?
  {
    updatewebintf = false ;                               // Yes, clear update flag
    update_software ( "v_webintf",                        // Update SPIFFS from remote file
                      updhost.c_str(),
                      UPDATEDIR, WEBINTFBIN,
                      U_SPIFFS ) ;                        // Resultaat naar SPIFFS partition
  }
  if ( resetreq )                                         // Reset requested?
  {
    dbgprint ( "Reset gevraagd..." ) ;                    // Yes, show
    P1_client.stop() ;                                    // Disconnect from P1-port
    vTaskDelete ( xcontroltask ) ;                        // Stop controltask
    ctdata[FORCE].value = 0 ;                             // Schakel naar stop
    xEventGroupSetBits ( taskEvents, MBtaskbit ) ;        // Vrsnel MBtask
    delay ( 500 ) ;                                       // Wait some time
    ESP.restart() ;                                       // Reboot
  }
  scanserial() ;                                          // Handle serial input
  if ( errorcount >= 1000 )                               // Teveel fouten?
  {
    dbgprint ( "Foutteller te hoog!  Reset.." ) ;
    resetreq = true ;                                     // Ja, forceer reset
  }
  //delay ( 10 ) ;
}
