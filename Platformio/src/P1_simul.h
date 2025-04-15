// P1_simul.h
// Module om P1 data te simuleren.
// De functie wordt elke 5 seconden aangeroepen.
// We genereren "power_delivered" en "power_returned".
// Elke stap duurt 30 seconden.
//
// De naam van de dongle is "P1_Simul".  Vermeld die in de config-pagina
// van de web-interface als je deze wilt gebruiken:
// dongle = P1_Simul
//
// Als de flag "simul" is  geset, dan wordt deze dongle ook gebruikt.


// Bruto vermogens, negatief is opwek
int16_t simPower[] = { 1000, 800, -2000, 400, INT16_MAX } ;


//**************************************************************************************************
//                                   D O N G L E _ H A N D L E                                     *
//**************************************************************************************************
// Genereer geleverde en opgewekt vermogen.                                                        *
//**************************************************************************************************
bool p1_simul_handle()
{
  static int8_t simIndex = 0 ;                              // Index in simPower
  static int8_t simTimer = 0 ;                              // Timer voor 40 seconden

  claimData ( "P1" ) ;                                      // Claim data gebied
  rtdata[PWIN].value = simPower[simIndex] -                 // Bepaal levering of opwek
                       rtdata[SETBC].value ;                // Minus de huidige laad-energie
  releaseData() ;
  if ( ++simTimer == 8 )                                    // 40 (8 * 5) seconden voorbij?
  {
    simTimer = 0 ;                                          // Ja, reset timer
    if ( simPower[++simIndex] == INT16_MAX )                // Volgende stap, einde van tabel?
    {
      simIndex = 0 ;                                        // Ja, begin weer vooraan
    }
  }
  return true ;                                             // Geef data gebied weer vrij
}
