// P1_MQTT.h
// Module om data uit de MQTT data van een dongle te halen.
//
// De naam van de dongle is "P1_MQTT".  Vermeld die in de config-pagina
// van de web-interface als je deze wilt gebruiken:
// dongle = P1_MQTT
//
// De functie wordt elke 5 seconden aangeroepen.
// We zijn geïnteresseerd in "power_delivered" en "power_returned".  Deze waarden
// worden als een floating point getal in een string doorgegeven.  De eenheid is kW of Watt.
#include "ArduinoJson.h"
//

bool  data_ok = false ;                                       // Data is okay of niet
float p1 = 0.0 ;                                              // Power gebruik en opwek in W/kW
float p2 = 0.0 ;


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
    //dbgprint ( "MQTT topic ontvangen [%s], %s",
    //           topic, value ) ;
    f = atof ( value ) ;                              // Haal aantal Watts
    if ( field_pIn.equalsIgnoreCase ( topic ) )       // Gaat het om "delivered"?
    {
      p1 = f ;                                        // Ja, bewaar
      if ( p1 != 0.0 )                                // Echte waarde?
      {
        p2 = 0.0 ;                                    // Ja, dan is p2 nul
      }
    }
    else if ( field_pOut.equalsIgnoreCase ( topic ) ) // Gaat het om "returned"?
    {
      p2 = f ;                                        // Ja, bewaar
      if ( p2 != 0.0 )                                // Echte waarde?
      {
        p1 = 0.0 ;                                    // Ja, dan is p1 nul
      }
    }
    data_ok = true ;                                  // Data is okay
  }
}


//**************************************************************************************************
//                                   D O N G L E _ H A N D L E                                     *
//**************************************************************************************************
// Lees het vermogen van de P1 meter vanuit de MQTT data.                                          *
//**************************************************************************************************
bool p1_mqtt_handle()
{
  claimData ( "P1" ) ;                                        // Claim data gebied
  if ( data_ok )
  {
    rtdata[PWIN].value = ( p1 - p2 ) * dongle_schaal ;        // Bewaar netto vermogen in Watt
  }
  releaseData() ;                                             // Geef data gebied weer vrij
  return data_ok ;                                            // Geef resultaat terug
}


