# Android_Localization_Dataset

Android app per la creazione di un Dataset in ambito di localizzazione usando i sensori di uno smartphone android.

La seguente app è stata creata nell'ambito di un progetto universitario presso l'Università degli studi di Verona.

## ToDo

- Test su versioni android più recenti
- Passare a material design
- Creare APK

## Specifiche

App testata su android sdk 15 l'app puo' essere utilizzata su qualsiasi dispositivo android con versione sdk 15 o superiore

## Descrizione

La seguente app permette la creazione di un dataset basato sui seguenti sensori:

- Sensore GPS 
- Accelerometro
- Magnetometro 
- Giroscopio

inoltre permette la creazione dei seguenti dato ad alto livello:

- Tempo ricezioni dati
- creazione di un percorso Ground Truth
- distanza in linea d'aria (metri) tra coordinate GPS consecutive

Alla fine della raccolta dati il dataset creato puo' essere inviato via mail all'indirizzo specificato

# Utilizzo
L'app deve essere usata con il GPS attivo sul dispositivo e si racomanda di utilizzarla in un ambiente dove arrivi il segnale GPS.

Per prima cosa si deve decidere il percorso da fare e settarlo al interno della classe MainActivity.java alla riga 54 del codice,
way = {Cpercorso che si voglie fare};

A questo punto si carica l'app sul dispositivo mobile.

Per prendere le coordinate iniziali si preme il pulsante "INIZIO", inoltre si registra i dati dei sensori( Accelerometro,
Magnetometro, Giroscopio) in un array e il tempo(come cronometro) con una precisione fino ai msec che si impiega a fare 
il step del percorso.

Sul schermo apparira l'indicazione del percorso specificato, indicato come "Percorso: ", il tempo e i dati dei sensori. 

Arrivati alla fine del step si preme "FINE", dove vendono prese le coordinati del punto in cui si è arrivato per calcolare la 
distanza percorsa in metri, che vere mostrato anche sullo schermo come "Distanza: ". 

Tutti i dati vengono registrati in una Stringa report, al fine del percorso vera inviato all'indirizzo mail specificato il 
report con tutti i dati acquisiti durante il test.

Per ricominciare basta premetre INIZIO.








