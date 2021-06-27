# Domande
1. Nel contesto della configurazione del baud rate per una periferica UART, si suppone che il clock sia 16MHz ed il sistema sia sovracampionato a 16x (OVER8 = 0). Quant'è dunque il valore di **USARTDIV** se si desidera un baud rate di 9600? Infine, in quali registri si metteranno i valori di mantissa ed esponente per realizzare tale configurazione?
2. Per una comunicazione **I²C**, come vengono generalmente usati i segnali SDA e SCL per segnalare una Stop Condition?
3. Descrivere brevemente il funzionamento della modalità slave per una comunicazione I2C.
4. In quale registro per la configurazione della comunicazione I²C si trova l'eventuale indirizzo secondario usato in modalità dual addressing?
5. Per una comunicazione SPI cosa specifica il valore Clock Polarity? Che valore assume nella modalità 0?
6. Per una comunicazione SPI, come si può configurare il data frame format? In quale registro si trova tale parametro di configurazione?
7. Fornire un breve confronto tra comunicazione SPI e I2C.

# Risposte
1. Con un clock a 16MHz e un valore di OVER8 = 0, affinchè si abba un baud rate di 9600 si ricava il valore per USARTDIV dall'espressione:

<p align="center">
<img src="https://latex.codecogs.com/svg.image?baud&space;=&space;\frac&space;{f_{clk}}&space;{8*(2-OVER8)*USARTDIV}" title="baud = \frac {f_{clk}} {8*(2-OVER8)*USARTDIV}" />
</p>

da cui:

<p align="center">
<img src="https://latex.codecogs.com/svg.image?USARTDIV&space;=&space;\frac&space;{f_{clk}}&space;{8*(2-OVER8)*baud}&space;=&space;104.1667" title="USARTDIV = \frac {f_{clk}} {8*(2-OVER8)*baud} = 104.1667" />
</p>

che viene mappato come definito dalla tabella consultabile sul **Reference manual RM0390 "STM32F446xx advanced Arm®-based 32-bit MCUs"** a pag. 811, ovvero pari a 104.1875.
Tale valore viene rappresentato come:
  * Fraction : 16*0.1875 = 3 (4-bit)
  * Mantissa : 104 (12-bit)

che verranno opportunamente shiftati e usati nel registro BRR della USARTx in uso.

2. Per segnalare una Stop Condition, si ha una transizione 0-1 del segnale SDA quando SCL è alto. Tale condizione è sempre segnalata dal Master.

3. Di default il dispositivo I2C lavora in slave mode. Una volta rilevato lo start condition, confronta l'indirizzo ricevuto con il suo; se non combaciano, scarta e va avanti, altrimenti risponde con un ACK. Infine, invia i dati dal registro DR sulla linea SDA tramite shift register, se in modalità trasmissione, altrimenti riceve dalla linea SDA sullo stesso registro se in modalità ricezione.

4. L'indirizzo secondario usato in modalità Dual addressing dell'I2C si trova nel registro OAR2.

5. Il clock della comunicazione SPI può essere configurato tramite alcuni parametri tra cui Clock Phase (CPHA) e il Clock Polarity (CPOL); quest'ultimo determina lo stato logico iniziale del clock. Dalla combinazione di questi due parametri, è possibile ricavare 4 modalità per il clock. In particolare, nella modalità 0 entrambi i parametri sono pari a 0.

6. Per la comunicazione SPI, è possibile configurare il data frame format in due modi, ovvero a 8 o a 16 bit. La configurazione avviene tramite il bit DFF (data frame format) all'interno del Control Register 1 (SPI CR1).

7. SPI è un protocollo di comunicazione sincrona con configurazione full-duplex. In genere, vengono usati quattro segnali: Chip Select (CS), Clock (SCK), Master Out / Slave In (MOSI) e Master In / Slave Out (MISO) per le comunicazioni tra un master e uno slave.
All'aumentare degli slaves, aumenta il numero di segnali di Chip Select. Non vi sono limiti predefiniti in velocità di un bus SPI. Non è implementata la gestione di conferma ricezione, tramite ack o simili. I2C invece richiede solo due linee: Serial Data (SDA) e Serial Clock (SCLK). Per la sua logica di funzionamento, ha dei limiti ben noti in termine di velocità e lunghezza dei cavi. I dispositivi vengono identificati da un indirizzo univoco predefinito. Tutti i dispositivi sono connessi ad un bus seriale condiviso.
