// P1_MQTT.h
// Module om data uit de MQTT data van een dongle te halen.
//
// De naam van de dongle is "P1_MQTT".  Vermeld die in de config-pagina
// van de web-interface als je deze wilt gebruiken:
// dongle = P1_MQTT
//
// De functie wordt elke 5 seconden aangeroepen.
// We zijn geïnteresseerd in "power_delivered" en "power_returned".  Deze waarden
// worden als een floating point getal in een string doorgegeven.  De eenheid is kW.
#include "ArduinoJson.h"
//

bool  data_ok = false ;                                       // Data is okay of niet
float p1, p2 ;                                                // Power gebruik en opwek in kW


//**************************************************************************************************
//                                    O N M Q T T M E S S A G E                                    *
//**************************************************************************************************
// Executed when a subscribed message is received.                                                 *
// Note that message is not delimited by a '\0'.                                                   *
// Note that cmd buffer is shared with serial input.                                               *
//**************************************************************************************************
void onMqttMessage ( char* topic, byte* payload, unsigned int len )
{
  char  value[10] ;                                   // kW als een string
  float f ;                                           // Gelezen waarde in kW

  data_ok = false ;
  if ( len < ( sizeof(value) - 1 ) )                  // Message may not be too long
  {
    strncpy ( value, (char*)payload, len ) ;          // Maak een kopie van de waarde-string
    value[len] = '\0' ;                               // Zorg voor een delimeter
    dbgprint ( "MQTT message ontvangen [%s], length = %d, %s", topic, len, value ) ;
    f = atof ( value ) ;                              // Haal aantal Watts
    if ( field_pIn.equalsIgnoreCase ( topic ) )       // Gaat het om "delivered"?
    {
      p1 = f ;                                        // Ja, bewaar
    }
    else if ( field_pOut.equalsIgnoreCase ( topic ) ) // Gaat het om "returned"?
    {
      p2 = f ;                                        // Ja, bewaar
    }
    data_ok = true ;                                  // Data is okay
  }
}


//**************************************************************************************************
//                                P 1 _ M Q T T _ I N I T                                          *
//**************************************************************************************************
// Set subscribe topics voor power-delivered en power-returned.                                    *
//**************************************************************************************************
void p1_mqtt_init()
{
  bool     res = true  ;                                    // Resultaat van subscribe

  if ( ! field_pIn.isEmpty() )
  {
    res |= mqttclient.subscribe ( field_pIn.c_str() ) ;     // Subscribe to MQTT, "power delivered" topic
  }
  if ( ! field_pOut.isEmpty() )
  {
    res |= mqttclient.subscribe ( field_pOut.c_str() ) ;    // Subscribe to MQTT, "power returned" topic
  }
  if ( !res )
  {
    dbgprint ( "MQTT subscribe fout!" ) ;                   // Failure
  }
}



//**************************************************************************************************
//                                   D O N G L E _ H A N D L E                                     *
//**************************************************************************************************
// Lees het vermogen van de P1 meter via MQTT.                                                     *
//**************************************************************************************************
bool p1_mqtt_handle()
{
  static bool  once = true ;                                  // Om init één keer uit te voeren

  if ( once )                                                 // Nog initialiseren?
  {
    p1_mqtt_init() ;                                          // Ja, subscribe to topics
    once = false ;                                            // En daarna nooit meer
  }
  claimData ( "P1" ) ;                                        // Claim data gebied
  if ( data_ok )
  {
    rtdata[PWIN].value = ( p1 - p2 ) * dongle_schaal ;        // Bewaar netto vermogen in Watt
  }
  releaseData() ;                                             // Geef data gebied weer vrij
  return data_ok ;                                            // Geef resultaat terug
}


