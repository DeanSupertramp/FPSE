## Esercizio 1
Implementare, in STM32Cube, un programma che fa uso dell'ADC per leggere i valori di
un potenziometro. Acquisire questi valori con una risoluzione a 8 bit (ottenendo valori tra 0 e
255) e usare questi valori per gestire la luminosità di un LED, con un segnale PWM.
Fare in modo che il duty cycle vari in maniera proporzionale al valore letto dal potenzio-
metro. Si presti attenzione al fatto che, evidentemente, il duty cycle varia tra 0 e 100 e non tra
0 e 255, come invece succede per i valori letti da potenziometro.
Implementare l'applicazione in STM32Cube, facendo uso del driver HAL per la gestione
dell'ADC che legge dal potenziometro.

## Svolgimento
Si decide di sviluppare due tipologie di codice, una adottando l'acquisizione ADC in
polling e l'altra tramite Interrupt.
In entrambi i casi, la configurazione dell'ADC e dei timer per generare il segnale PWM è
stata fatta tramite STM32CubeMX. Inoltre, si sfrutta una semplice libreria creata nei precedenti
esercizi per agevolare l'acquisizione tramite ADC.
