

## Domande
1. Spiegare in maniera sintetica il funzionamento della capture mode con cui possono operare i timers.
2. Per un basic timer in un sistema con clock a 42Mhz, come possono essere scelti i di prescalar e period per ottenere un update event di 300ms?
3. Quali registri entrano in gioco per la generazione di un segnale PWM con un general purpose timer? Ed in particolare, a quale registro è direttamente associato il valore del
duty cycle?
4. Quale è la differenza tra le modalità up-counting e center-aligned per la generazione di un segnale PWM? Pensa ci possa essere un vantaggio ad usare una delle due o l'utilizzo
dipende esclusivamente dal tipo di segnale che si vuole realizzare?

## Risposte
1. Un timer può operare in diversi modi, ovvero:

    * Compare Mode
    * Capture Mode
    * PWM mode. In capture mode viene generata una sequenza di impulsi da una sorgente esterna. In questa fase puà essere impostato opzionalmente il prescaler per dividere la frequenza del segnale generato dall'evento esterno. Quando avviene un evento, si alza il capture enable, A questo punto, il capture register acquisisce il valore istantaneo contenuto nel timer register nel momento in cui avviene l'evento. A questo punto, è possibile generare un interrupt per effettuare qualche operazione.
  
  Una espressione dell'update event in funzione del valore di prescaler e period puà essere:
 
 <p align="center">
<img src="https://latex.codecogs.com/svg.image?UpdateEvent&space;=&space;\frac{Timer_{clk}}&space;{(Prescaler&plus;1)*(Period&plus;1)}" title="UpdateEvent = \frac{Timer_{clk}} {(Prescaler+1)*(Period+1)}" />
</p>

Per un clock a 42Mhz, è possibile individuare come valore di prescaler 4199 e come period 2999, in modo da ottenere 3.33Hz come update event, ovvero 0.3ms come periodo.

2. In STM32F4 i _general purpose timer_ sono dal **TIM2** al **TIM5** e dal **TIM9** al **TIM14**. Per la generazione di un segnale PWM bisogna impostare il pin desiderato tramite l'_Alternate Function_. Il segnale viene ottenuto confrontando i valori presenti nel registro di conteggio **TIMx_CNT** e quello da impostare **TIMx_CCRy**. Il periodo del segnale viene definito nel registro **TIMx_ARR**, mentre il valore di duty cycle viene definito nel registro **TIMx_CCRy**. Successivamente si setta a 1 il bit **CCxE** per configurare il canale come uscita e i bit **OCxM** per determinare il comportamento se **CNT < CCRy**.
Infine, si setta il bit **CCxP** per determinare la logica (alta o bassa) e il bit **CCxE** per abilitare l'uscita.

3. In generale (a prescindere che si lavori in modalità PWM), un timer può lavorare in 2 modalità:
	* **edge-aligned mode**, differenziandosi in: _up-counting_ oppure _down-counting_,
	* **center-aligned mode**.

In modalità up-counting, il contatore parte da 0 fino al valore impostato nell’auto-reloadregister, genera un evento di **overflow** del contatore e riparte il conteggio da 0.  Il valore di duty cycle dipende da quanto impostato nel registro **TIMxCCRy**.  Il periodo tra unevento e il successivo (quindi del segnale PWM) è pari a:

<p align="center">
<img align="center" src="https://latex.codecogs.com/svg.image?(1&space;&plus;&space;ARR)&space;*&space;Clock&space;Period" title="(1 + ARR) * Clock Period" />  
</p>

In modalità _down-counting_, il contatore parte dal valore impostato nell'_auto-reload register_ fno a 0, genera un evento di **underflow** del contatore e riparte il conteggio dal valore dell'_auto-reload register_. Il valore di duty cycle dipende da quanto impostato nel registro **TIMx_CCRy**. Il periodo tra un evento e il successivo (quindi del segnale PWM) è pari a:

<p align="center">
<img align="center" src="https://latex.codecogs.com/svg.image?(1&space;&plus;&space;ARR)&space;*&space;Clock&space;Period" title="(1 + ARR) * Clock Period" />  
</p>

In modalità _center-aligned_, il contatore eseguirà in successione un up-counting e un down-counting, generando un evendo rispettivamente di over
ow e underflow. Come nei casi precedenti, il valore di duty cycle può essere impostato nel registro **TIMx_CCRy**. Il periodo tra un evento e il successivo (quindi del segnale PWM) sarà il doppio rispetto ai casi precedenti, ovvero:

<p align="center">
<img src="https://latex.codecogs.com/svg.image?2&space;*&space;ARR&space;*&space;Clock&space;Period" title="2 * ARR * Clock Period" />
</p>

La scelta di utilizzare una modalità piuttosto che un'altra dipende dall'applicazione. In genere, si usa la modalità up-counting quando si contano degli intervalli. Il down-counting è usato quando si eseguono i conti alla rovescia prima che si verifichi un evento. Infine, si utilizza il center-aligned per la generazione di segnali PWM. In quest'ultima modalità, per fornire un segnale della stessa frequenza dei precedenti, dovrà avere una velocità di clock doppia, come discusso in precedenza. Ciò è positivo in termini di analisi in frequenza perchè, come riportato nell'application report: **SPRA278 Symmetric PWM Outputs Generation with the TMS320C14 DSP**:

> "It has been shown that symmetric PWM signals generate fewer harmonics in the output currents and voltages."

Si riporta inoltre quanto definito nell'**Application note AN4013**:

>For a given PWM switching frequency, this mode reduces the acoustic noise by doubling the effective current ripple frequency, thus providing the optimum
tradeoff between the power stage's switching losses and noise.

Quindi, per tutte alcune applicazioni critiche sotto questo aspetto è consigliabile l'uso della modalità center-aligned.: più alta è la frequenza PWM rispetto alla nostra del segnale, più lontane sono le armoniche nello spettro che portano a un segnale più pulito, o comunque più facilmente saranno filtrabili.

D'altra parte, la frequenza PWM massima utilizzabile è limitata dal nostro microcontrollore. Una interessante analisi in termini di frequenza è riportata al seguente [link](https://marcelmg.github.io/pwm_left_vs_centered/) Quindi, se l'applicazione è, ad esempio, la creazione di un segnale audio, ciò è positivo perchè ci sono meno armoniche nella gamma udibile. Ma ci sono altre applicazioni, ad esempio nell'elettronica di potenza, in cui ciò non è vantaggioso perchè le armoniche vengono spostate solo a destra nello spettro, ma non diminuite. Bisognerà trovare un giusto trade-off tra le considerazioni discusse.
