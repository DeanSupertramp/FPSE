## Esercizio 1

Utilizzare un potenziometro per gestire il lampeggiamento di un LED.

Utilizzando i valori letti dal potenziometro, il flashing rate del LED deve variare tra 0 secondi e un 1 secondo, nel caso di valore pari a 0.0 il LED deve spegnersi. Il sistema viene gestito dalla pressione di un pulsante (USER_BUTTON o pulsante esterno) che abilita/disabilita l'utilizzo del valore letto dal potenziometro per gestire il lampeggiare del LED: dopo l'avvio del programma, premendo il pulsante si disabilita la lettura dal potenziometro ed il LED continua a lampeggiare con un rate pari all'ultimo valore letto in ingresso; ri-premendo il pulsante si ripristina il funzionamento iniziale, con il potenziometro si può nuovamente gestire il lampeggiare del LED.

Implementare l'applicazione in STM32Cube, facendo uno del driver HAL per la gestione dell'ADC che legge dal potenziometro.
