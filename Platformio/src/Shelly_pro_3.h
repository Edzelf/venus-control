// Shelly_pro_3.h
// Module om data uit de JSON van de Shelly Pro 3 EM dongle te halen.
// De api is bereikbaar via http://<IP>/rpc/EM.GetStatus?id=0.
// Het levert een JSON op die er bijvoorbeeld zo uitziet:
// {
// 	"id": 0,
// 	"a_current": 0.041,
// 	"a_voltage": 0.1,
// 	"a_act_power": -0.0,
// 	"a_aprt_power": 0.0,
// 	"a_pf": 1.00,
// 	"b_current": 0.027,
// 	"b_voltage": 0.1,
// 	"b_act_power": 0.0,
// 	"b_aprt_power": 0.0,
// 	"b_pf": 1.00,
// 	"c_current": 0.026,
// 	"c_voltage": 233.5,
// 	"c_act_power": 0.1,
// 	"c_aprt_power": 6.1,
// 	"c_pf": 1.00,
// 	"n_current": null,
// 	"total_current": 0.094,
// 	"total_act_power": 0.067,
// 	"total_aprt_power": 6.140,
// 	"user_calibrated_phase": [],
// 	"errors": ["no_load"]
// }

// De naam van de dongle is "Shellypro3".  Vermeld die in de config-pagina
// van de web-interface als je deze wilt gebruiken:
// dongle = Shellypro3
//
// De functie wordt elke 5 seconden aangeroepen.
// We zijn geïnteresseerd in "total_act_power".  Deze waarden
// worden als een getal in een string doorgegeven.  De eenheid is kWatt.
#include "ArduinoJson.h"
//
//**************************************************************************************************
//                                   D O N G L E _ H A N D L E                                     *
//**************************************************************************************************
// Lees de JSON structuur van de P1 meter via http.                                                *
// Haal de geleverde en de teruggeleverde vermogen uit de JSON string en zet die in common data.   *
//**************************************************************************************************
bool shellypro3_handle()
{
  float p1 ;                                                    // Power gebruik (negatief is opwek)

  if ( dongle_api.isEmpty() )                                   // API bekend?
  {
    dongle_api = "/api/v1/data" ;                               // Nee, set default
  }
  if ( read_p1_dongle_http() )                                  // Lees JSON van P1 via http
  {
    claimData ( "P1" ) ;                                        // Claim data gebied
    p1 = json_doc["total_act_power"] ;                          // Bepaal netto vermogen (grid naar huis)
    rtdata[PWIN].value = p1 * 1000.0 ;                          // Bewaar netto vermogen in Watt
    releaseData() ;
    return true ;                                               // Geef data gebied weer vrij
  }
  return false ;
}
