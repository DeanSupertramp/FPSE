## Domande
1. Cosa rappresenta la velocità di un GPIO configurato come output? In quale registro viene conservata questa informazione e quale è il valore di configurazione di default?
2. Cosa cambia, dal punto di vista pratico, nella configurazione di un GPIO di output come push-pull o open-drain? Con quali registri vengono configurate tali modalità?

## Risposte
1. La velocità di un GPIO rappresenta, in termini qualitativi, un indice di quanto tempo impiega a completare il fronte di salita/discesa durante una commutazione dell'uscita. Sono
disponibili quattro velocità, ovvero: Low, Medium, Fast, High. Dalla teoria sulla compatibilità elettromagnetica si evidenzia che minore è il tempo di salita/discesa di un onda
quadra, maggiore è la radiazione elettromagnetica associa, ovvero l'interferenza irradiata. Inoltre, una maggiore velocità di commutazione richiede più potenza. Bisogna dunque valutare caso per caso il tradeoff migliore in funzione del contesto applicativo. Ad esempio, è sufficiente una velocità bassa per effettuare commutazioni sui led, in quanto una velocità maggiore non sarebbe apprezzabile ad occhio umano.
Viceversa, per implementare protocolli di comunicazione come SPI è richiesto una velocità maggiore. Il registro associato alla velocità delle porte GPIO si chiama OSPEEDR (GPIO Port Output Speed Register).
Tale registro ha dimensione pari a 32 bit e per gestire 16 pins sono richiesti 2 bit per pin. Il valore di default è pari a 00 per ogni pin, ovvero una velocità bassa.

2. Configurare un GPIO di output come push-pull o open-drain determina il comportamento dell'uscita per ogni tipo di stato logico. In modalità push-pull, l'uscita è sempre ben
definita, in quanto forzata ad un livello logico alto tramite una rete di pull-up oppure ad un livello logico basso tramite una rete di pull-down. In modalità open-drain è possibile
forzare l'uscita ad un livello logico basso, ma non è ben definito il livello logico alto in quanto, mantenendo dunque floating l'uscita. In questo caso l'uscita non è determinata e ciò può provocare problemi. Dunque, per tale configurazione si rende necessario utilizzare
una resistenza esterna di pull-up, per garantire uno stato logico sempre ben definito. Tali modalità vengono configurate tramite il registro OTYPE, con dimensione pari a 32 bit,
di cui 16 riservati e 16 usati per ognuno dei pin della porta interessata.
