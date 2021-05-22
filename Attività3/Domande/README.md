## Domande
1. Quale registro viene utilizzato per la gestione delle priorità per le interrupts? In quanti (e quali) campi viene suddiviso tale registro per la confiurazione delle priorità?
2. Quali registri entrano in gioco nella configurazione del System Timer (SysTick)?
3. È possibile, usando il clock HSI con PLL, generare un SYSCLK a 100.5 Mhz? Motivare la risposta esplicitando la soluzione, eventualmente mostrare anche uno screen della schermata di Clock Configuration di STM32Cube MX.

## Risposte
1. Il registro utilizzato per la gestione delle priorità per le interrupts è l'Interrupt Priority Register (IP). Tale registro è diviso in tre campi:
   1. Il Preempt Priority Number (default 2 bit), che definisce la priorità di preemption (prelazione). In particolare, determina se un interrupt può anticipare un interrupt già in esecuzione.
   2. Il Sub-Priority Number (default 2 bit), che determina quale interrupt viene eseguito per primo, nel caso in cui abbiano lo stesso Preempt Priority Number.
   3. il terzo campo non è implementato (4 bit).
Può essere gestito tramite l'NVIC tramite il comando:
```
NVIC_SetPriority();
```
Di default, sono dedicati 2 bit per il Preempt Priority Number e 2 bit per il Sub-Priority Number. È possibile ridimensionare i bit associati ai due campi tramite il comando:
```
NVIC_SetPriorityGrouping(n)
```

2. registri che entrano in gioco nella configurazione del System Timer (SysTick) sono 4, ovvero:
   1. SysTick Control and Status Register (SysTick CTRL). Contiene i seguenti campi:
      i. COUNTFLAG: restituisce 1 se il timer ha contato a 0 dall'ultima volta che è stato letto;
      ii. CLKSOURCE: seleziona la sorgente del clock tra AHB/8 (valore 0) e AHB (valore 1);
      iii. TICKINT: abilita il SysTick Interrupt;
      iv. ENABLE: abilita il contatore.
   2. SysTick Reload Value Register (SysTick LOAD). Campo formato da 24 bit, con un valore massimo pari a 0x00FF.FFFF (16,777,215). Contiene il valore dal quale parte il conteggio. Scrivendo in RELOAD il valore 0 disabilita SysTick, indipendentemente da TICKINT.
   3. SysTick Current Value Register (SysTick VAL). Registro contenente il valore corrente.
   4. SysTick Calibration Register (SysTick CALIB). Di sola lettura, contiene alcune proprietà relative alla calibrazione del SysTick.

3. STM32 implementa diversi segnali di clock, ovvero HSI (High Speed Internal), HSE (High Speed External), LSI (Low Speed Internal) e LSE (Low Speed External). In particolare,
HSI viene generata internamente da un circuito RC, con frequenza fissata a 16MHz. Il SYSCLK può essere determinato a partire da uno dei segnali precedentemente elencati,
selezionandolo tramite un multiplexer. Il segnale di clock viene poi ripartito in diversi bus:
  i. AHB (Advanced High-performance Bus) per la CPU, la memoria, il DMA e i GPIO.
  ii. APB1 (Advanced Peripheral Bus 1 ) per alcune periferiche;
  iii. APB2 per altre periferiche.
  
I segnali di clock, generati internamente o esternamente al sistema, possono essere riadattati in frequenza attraverso un circuito chiamato PLL (Phase Locked Loop), con la
funzionalità di moltiplicatore/divisore di frequenza, tramite parametri N, P, Q (divisione) e N (moltiplicazione). Questi valori possono essere settati nel registro RCC PLL
configuration register (RCC PLLCFGR).
È possibile configurare tramite interfaccia grafica i vari campi per la gestione del clock nel Clock Configuration di STM32Cube MX. La configurazione di default è quella
mostrata in Fig.1.

<p align="center">
<img src="Img/clock0.JPG" width="600">
</p>

Obiettivo è quello di generare un SYSCLK a 100.5 Mhz usando il clock HSI con PLL. Il valore di default è 84MHz. Una soluzione è quella di aumentarne il valore tramite il
fattore moltiplicativo N, portandolo da 336 a 402. Ciò permette di ottenere un SYSCLK pari a 100.5MHz, ma come mostrato dal STM32Cube MX e riportato in Fig. 2 ci sono
alcune problematiche sui valori di clock forniti ad alcune periferiche.

<p align="center">
<img src="Img/clock1.JPG" width="600">
</p>

Infatti, per il segnale PCLK1 (APB1 Peripheral clocks) si ha un valore limite massimo pari a 45 MHz, mentre per il segnale PCLK2 (APB2 Peripheral clocks) si ha un valore
limite massimo pari a 90MHz, entrambi superati con il precedente settaggio del PLL. Per risolvere, una soluzione può essere quella di impostare dei valori di prescaler per APB1 e
APB2 tali da rientrare nei range. Ad esempio, scegliendo un fattore di prescaler APB1 pari a /4 e un fattore di prescaler APB2 pari a /2 si rientra nei valori limite, come visibile
in Fig. 3.

<p align="center">
<img src="Img/clock2.JPG" width="600">
</p>
