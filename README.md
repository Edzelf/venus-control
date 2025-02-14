
![logo](https://github.com/user-attachments/assets/343db925-dc42-4a91-88b5-51ac631e3bb0)

Deze repositry beschrijft de ontwikkelingen m.b.t. de Marstek Venus-E.  De bedoeling is om te komen tot een stabiele en verbeterde regeling van de batterij.

De huidige versie maakt gebruik van een Marstek CT003 P1-dongle.  Deze lijkt niet erg stabiel.  Bovendien lijkt de interne regeling v.w.b. “nul op de meter” (NOM) niet overtuigend te werken.  De software van de batterij, momenteel versie 147 is niet open-source, waardoor je afhankelijk bent van de leverancier.

Plannen voor de verbeterde versie zijn:
* Open source software (PlatformIO, ESP32, Arduino).
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
* 14-02-2025 - Print prototype gemaakt.  ModBus communicatie met deze pring getest, werkt!
