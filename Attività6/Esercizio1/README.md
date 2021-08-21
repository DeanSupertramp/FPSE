# Esercizio 1

Scrivere un programma che acquisisca i dati da un sensore e li invii, tramite comunicazione **UART**, al pc. Il programma deve stare anche in attesa di ricevere, sempre tramite comunicazione UART, un comando di stop (ad esempio un singolo carattere: 'S') che interrompe l'invio dei dati ma non la loro acquisizione ed un segnale di start (ad esempio un singolo carattere: 'P') per riprendere il normale funzionamento.
Se si dispone della IMU già discussa, si può utilizzare come sensore esterno, altrimenti fare uso di un sensore di temperatura, un potenziometro o qualsiasi altro sensore analogico si abbia a disposizione.
Se possibile, collezionare le misure acquisite in un array di almeno 10 elementi e inviare al pc la media di tali misure. Si può usare una semplice media aritmetica o in alternativa una media pesata, che pesi maggiormente gli ultimi valori inseriti nell'array. Si noti che, per un corretto funzionamento di questo procedimento, l'acquisizione dei dati deve avvenire con una frequenza almeno 10 volte maggiore rispetto alla frequenza di invio dei dati.
Infine, utilizzare un LED RGB di segnalazione come descritto:
- LED di colore verde: fase di acquisizione;
- LED di colore blu: fase di invio dei dati;
- LED di colore rosso: comando di stop ricevuto;
- LED di colore bianco: fase di acquisizione dopo aver ricevuto lo stop.
Se possibile implementare una "libreria" dedicata alla gestione del LED RGB. E, se si dispone di uno Shift Register, lo si può usare per gestire il LED RGB come mostrato nelle esercitazioni.

# Svolgimento
Per lo svolgimento dell'esercizio, si decide di usare la ricezione UART con interrupt, implementando una libreria sviluppata durante gli esercizi precedenti per semplicifare le acquisizioni, uno **shift register** per controllare il led RGB. Il sensore scelto è un sensore di temperatura analogico **LM35**. Si implementa una media aritmetica tramite le librerie **CMSIS-DSP**, ottimizzate per il calcolo e l'elaborazione dei segnali. Si allega l'intero progetto.
