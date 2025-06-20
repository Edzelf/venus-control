
![logo](https://github.com/user-attachments/assets/343db925-dc42-4a91-88b5-51ac631e3bb0)

Deze repositry beschrijft de ontwikkelingen m.b.t. de Marstek Venus-E.  De bedoeling is om te komen tot een stabiele en verbeterde regeling van de batterij.

De huidige versie maakt gebruik van een Marstek CT003 P1-dongle.  Deze lijkt niet erg stabiel.  Bovendien lijkt de interne regeling v.w.b. “nul op de meter” (NOM) niet overtuigend te werken.  De software van de batterij, momenteel versie 147 is niet open-source, waardoor je afhankelijk bent van de leverancier.

Plannen voor de verbeterde versie zijn:
* Open source software (PlatformIO, ESP32, Arduino).
* Onafhankelijk maken van externe servers.
* Goede documentatie.
* Sturing via ModBus.
* Vrije keus van P1-dongle.
* Meer dan één model voor de regeling van laden/ontladen.
* Data beschikbaar stellen via MQTT.
* Beschikbaar stellen van een Web-interface.
* Zelfbouw mogelijk maken.
* Print ontwerpen.

Ik weet dat er mogelijkheden zijn om Home Automation (HA) te gebruiken voor de sturing van de batterij, maar dit project is gericht op een zelfstandige plug-en-play oplossing.

Ontwikkkeling:
* 12-02-2025 - Eerste versie van de documentatie, ModBus getest met een USB converter onder Linux, programmeren gestart.
* 12-02-2025 - Uitlezen P1 dongle (WiFi) werkt inmiddels.  Dongle iP1_Dongle_Pro van smart-stuff.nl
* 13-02-2025 - Simulatie dongle gemaakt.  Dient om later een besturingsmodel te kunnen testen.
* 14-02-2025 - Print prototype gemaakt.  ModBus communicatie met deze print getest, werkt!
* 17-02-2025 - Debugging ModBus communicatie.
* 18-02-2025 - Webinterface gemaakt. Tonen van procesgegevens in primitieve vorm.
* 26-02-2025 - Eerste versie werkt nu.  De besturing probeert de inkomende/teruggeleverde energie op nul te houden.
* 04-03-2025 - Uitgebreid getest.  Print ontworpen en besteld bij PCBWay.
* 13-03-2025 - Print gereed. Duurtest gestart.
* 20-03-2025 - Uitbreiding met handmatige instelling.
* 25-03-2025 - Betere ModBus driver. Ondersteuning voor Homewizard P1 dongle.
* 08-04-2025 - Energie meting via MQTT.  Instelling voor kW of Watt meting.
* 10-04-2025 - Eenvoudig flashen van de software via webbrowser inclusief preset van WiFi credentials.
* 20-06-2025 - Maximale laadvermogen van de batterij is nu configurerbaar.

Printontwerp:
De print is ontworpen en febrabriceerd in samenwerking met PCBWay.  PCBWay is bekend om zijn uitstekende kwaliteit en zeer snelle levering.  De gefabriceerde print ziet er zo uit:

![image](https://github.com/user-attachments/assets/1456eb45-89a0-4ce3-8583-d6aa8df0ffdf)
Na montage van de onderdelen ziet het er zo uit:

![image](https://github.com/user-attachments/assets/35b91f00-dbfa-4109-9661-561494a5d8fc)

Aangesloten op de Marstek kabel ziet het er zo uit:

![image](https://github.com/user-attachments/assets/3c637cd9-2cd4-4d71-962b-d980cc887dc8)

