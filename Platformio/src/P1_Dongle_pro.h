// P1_Dongle_pro.h
// Module om data uit de JSON van deze dongle te halen.
// De dongle wordt geleverd door "smart-stuff.nl".
// Software is te downloaden via https://install.smart-stuff.nl/p1u/
// De api is bereikbaar via http://<ip>/api/v2/sm/actual
// Connect gaat soms fout als er ook andere apparaten via WiFi contact maken met de dongle.
// Het levert een JSON op die er zo uitziet:
// {"timestamp":{"value":"250207122727W"},
//  "energy_delivered_tariff1":{"value":6071.137,"unit":"kWh"},
//  "energy_delivered_tariff2":{"value":5551.723,"unit":"kWh"},
//  "energy_returned_tariff1":{"value":1735.003,"unit":"kWh"},
//  "energy_returned_tariff2":{"value":3571.852,"unit":"kWh"},
//  "electricity_tariff":{"value":"0002"},
//  "power_delivered":{"value":2.729,"unit":"kW"},
//  "power_returned":{"value":0,"unit":"kW"},
//  "voltage_l1":{"value":234,"unit":"V"},
//  "current_l1":{"value":11,"unit":"A"},
//  "power_delivered_l1":{"value":2.729,"unit":"kW"},
//  "power_returned_l1":{"value":0,"unit":"kW"},
//  "gas_delivered":{"value":6194.451,"unit":"m3"},
//  "gas_delivered_timestamp":{"value":"250207122500W"}
// }

// De naam van de dongle is "P1_Dongle_Pro".  Vermeld die in de config-pagina
// van de web-interface als je deze wilt gebruiken:
// dongle = P1_Dongle_Pro
//
// De functie wordt elke 5 seconden aangeroepen.
// We zijn geïnteresseerd in "power_delivered" en "power_returned".  Deze waarden
// worden als een floating point getal in een string doorgegeven.  De eenheid is kW.
#include "ArduinoJson.h"
//
//**************************************************************************************************
//                           P 1 _ D O N G L E _ P R O _ I N I T                                   *
//**************************************************************************************************
// Set JSON keys voor power- in en uit.                                                            *
//**************************************************************************************************
void p1_dongle_pro_init()
{
  if ( field_pIn.isEmpty() )                                // Field pIn al ingevuld?
  {
    field_pIn = "power_delivered" ;                         // Nee, gebruik default
  }
  if ( field_pOut.isEmpty() )                               // Field pOut al ingevuld?
  {
    field_pOut = "power_returned" ;                         // Nee, gebruik default
  }
  if ( dongle_api.isEmpty() )                               // API bekend?
  {
    dongle_api = "/api/v2/sm/actual" ;                      // Nee, set default
  }
}


//**************************************************************************************************
//                                   D O N G L E _ H A N D L E                                     *
//**************************************************************************************************
// Lees de JSON structuur van de P1 meter via http.                                                *
// Haal de geleverde en de teruggeleverde vermogen uit de JSON string en zet die in common data.   *
//**************************************************************************************************
bool p1_dongle_pro_handle()
{
  float        p1, p2 ;                                         // Power gebruik en opwek
  static bool  once = true ;                                    // Om init één keer uit te voeren

  if ( once )                                                   // Nog initialiseren?
  {
    p1_dongle_pro_init() ;                                      // Ja, doe dat dan
    once = false ;                                              // En daarna nooit meer
  }
  if ( read_p1_dongle_http() )                                  // Lees JSON van P1 via http
  {
    claimData ( "P1" ) ;                                        // Claim data gebied
    p1 = json_doc[field_pIn.c_str()]["value"] ;                 // Bepaal netto vermogen (grid naar huis)
    p2 = json_doc[field_pOut.c_str()]["value"]  ;               // Beide zijn in kW
    rtdata[PWIN].value = ( p1 - p2 ) * dongle_schaal ;          // Bewaar netto vermogen in Watt
    releaseData() ;
    return true ;                                               // Geef data gebied weer vrij
  }
  return false ;
}
