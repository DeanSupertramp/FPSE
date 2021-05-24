## Domande
1. Se per un DAC a 7 bit la tensione di riferimento è 2.56V, qual è la sua risoluzione in termini di voltaggio?
2. Descrivere brevemente il funzionamento di un ADC ad approssimazioni successive (SAR).
3. Come funziona l'Analog Watchdog e in quale registro si trovano i campi di configurazione? Come ad esempio il campo analog watchdog channel selection.

## Risposte
1. La risoluzione in termini di voltaggio di un DAC a 7 bit con tensione di riferimento di 2.56V è pari a:

![equation](https://latex.codecogs.com/png.image?\dpi{110}&space;\frac{V_{ref}}{2^n}&space;=&space;\frac{2.56}{2^7}&space;=&space;\frac{2.56}{128}&space;\cong{0.02}&space;V)

2. Esisto tre principali architetture ADC:
   a. Successive-approximation (SAR);
   b. Sigma-delta;
   c. Pipeline.
 Il SAR è  basato  principalmente  su  tre  blocchi:   un  circuito  di Sampling  and  Hold(SH), un comparatore ed un circuito logico (SAR Control Logic).
 Il segnale analogicoin ingresso viene campionato dal circuito SH e comparato con il segnale generato e digi-talizzato (tramite DAC) dal circuito logico.
 L’uscita del comparatore sarà il risultato del confronto tra i due ingressi, fornito in ingresso al controllo che, per approssimazioni successive appunto, arriverà ad inseguire il valore analogico di ingresso: aumenterà se l’uscita del comparatore è positiva,  diminuirà viceversa.
 Di estrema importanza è il numero di cicli impiegati per ottenere il risultato. A tal fine, è necessario attendere un periodo di sampling time, configurabile via software e un periodo di conversion time, pari al numero di bit (risoluzione) per il periodo di clock del ADC.
 
3. L’Analog Watchdog (AWD), se abilitato e configurato correttamente, permette di generare un interrupt se il segnale analogico in ingresso supera due soglie, superiore (HTR) e inferiore (LTR), programmabili nei rispettivi registri a 16 bit ADCHTR e ADCLTR. Il controllo avviene via hardware, non compromettendo l’esecuzione del software. I campi di configurazione si trovano nel ADC control register 1 (ADCCR1), nel quale è possibile configurare:
   a. AWDEN/JAWDEN: enable on regular/injected channels;
   b. AWDCH: analog watchdog channel selection;
   c. AWDSGL: enable watchdog on single channel in scan mode;
   d. AWDIE: enable interrupt on analog watchdog.
