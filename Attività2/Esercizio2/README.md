## Esercizio 2
Con la stessa logica utilizzata nell'esempio dell'esercitazione, a partire da quell'esempio abilitare altri due pin di ingresso su due GPIO della porta B, connettendoli ad altri due pulsanti, ed altri due pin di uscita su GPIO della porta A, connettendo altri due LED. Collegare logicamente ogni pulsante (ingresso) ad un LED (in uscita) e fare in modo che alla pressione di ogni pulsante si accenda/spenga un LED diverso. Testare il funzionamento dell'intero programma valutando anche le variazioni dei registri tramite interfaccia di debug. Si faccia attenzione ad utilizzare le opportune resistenze per la connessione di LED e pulsanti, come indicato.

## Svolgimento
Si avvia la modalità di debug, confermata dal fatto che la scheda lampeggia. Il debugger si ferma alla funzione HAL Init(). I registri assumono lo stato riportato in Fig. 1.


La prima parte del codice contenuto nel main() è relativo alla configurazione dei registri necessari alla gestione dei GPIO. Si abilita il clock delle porte GPIOA e GPIOB:
