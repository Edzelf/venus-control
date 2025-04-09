// Homewizard.h
// Module om data uit de JSON van de HomeWizard P1 dongle te halen.
// De api is bereikbaar via http://<IP>/api/v1/data.  De V1 API moet eerst geactiveerd worden.
// Connect gaat soms fout als er ook andere apparaten via WiFi contact maken met de dongle.
// Het levert een JSON op die er bijvoorbeeld zo uitziet:
// {
//   "wifi_ssid": "ThelosenWifi",
//   "wifi_strength": 58,
//   "smr_version": 50,
//   "meter_model": "CTA5ZIV-METER",
//   "unique_id": "4530303639303030373131353636353230",
//   "active_tariff": 1,
//   "total_power_import_kwh": 12353.876,
//   "total_power_import_t1_kwh": 6624.944,
//   "total_power_import_t2_kwh": 5728.932,
//   "total_power_export_kwh": 18250.33,
//   "total_power_export_t1_kwh": 5659.792,
//   "total_power_export_t2_kwh": 12590.538,
//   "active_power_w": -1,
//   "active_power_l1_w": 6,
//   "active_power_l2_w": -133,
//   "active_power_l3_w": 125,
//   "active_voltage_l1_v": 224,
//   "active_voltage_l2_v": 229,
//   "active_voltage_l3_v": 229,
//   "active_current_a": 1.153,
//   "active_current_l1_a": 0.027,
//   "active_current_l2_a": -0.581,
//   "active_current_l3_a": 0.546,
//   "voltage_sag_l1_count": 28,
//   "voltage_sag_l2_count": 13,
//   "voltage_sag_l3_count": 7,
//   "voltage_swell_l1_count": 12,
//   "voltage_swell_l2_count": 24,
//   "voltage_swell_l3_count": 23,
//   "any_power_fail_count": 43,
//   "long_power_fail_count": 127,
//   "total_gas_m3": 2388.552,
//   "gas_timestamp": 250325221000,
//   "gas_unique_id": "4730303732303034303531323736313230",
//   "external": [
//     {
//       "unique_id": "4730303732303034303531323736313230",
//       "type": "gas_meter",
//       "timestamp": 250325221000,
//       "value": 2388.552,
//       "unit": "m3"
//     }
//   ]
// }

 
// De naam van de dongle is "Homewizard".  Vermeld die in de config-pagina
// van de web-interface als je deze wilt gebruiken:
// dongle = Homewizard
//
// De functie wordt elke 5 seconden aangeroepen.
// We zijn geïnteresseerd in "activePower_w".  Deze waarden
// worden als een getal in een string doorgegeven.  De eenheid is Watt.
#include "ArduinoJson.h"
//
//**************************************************************************************************
//                           H O M E W I Z A R D _ I N I T                                         *
//**************************************************************************************************
// Set JSON keys voor power- in en uit.                                                            *
//**************************************************************************************************
void homewizard_init()
{
  if ( field_pIn.isEmpty() )                                // Field pIn al ingevuld?
  {
    field_pIn = "active_power_w" ;                          // Nee, gebruik default
  }
  if ( dongle_api.isEmpty() )                               // API bekend?
  {
    dongle_api = "/api/v1/data" ;                           // Nee, set default
  }
}


//**************************************************************************************************
//                                   D O N G L E _ H A N D L E                                     *
//**************************************************************************************************
// Lees de JSON structuur van de P1 meter via http.                                                *
// Haal de geleverde en de teruggeleverde vermogen uit de JSON string en zet die in common data.   *
//**************************************************************************************************
bool homewizard_handle()
{
  int32_t p1 ;                                                  // Power gebruik (negatief is opwek)
  static bool  once = true ;                                    // Om init één keer uit te voeren

  if ( once )                                                   // Nog initialiseren?
  {
    homewizard_init() ;                                         // Ja, doe dat dan
    once = false ;                                              // En daarna nooit meer
  }
  if ( read_p1_dongle_http() )                                  // Lees JSON van P1 via http
  {
    claimData ( "P1" ) ;                                        // Claim data gebied
    p1 = json_doc[field_pIn.c_str()] ;                          // Bepaal netto vermogen (grid naar huis)
    rtdata[PWIN].value = p1 ;                                   // Bewaar netto vermogen in Watt
    releaseData() ;
    return true ;                                               // Geef data gebied weer vrij
  }
  return false ;
}
