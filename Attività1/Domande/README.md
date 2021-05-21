## Domande
1. Illustrare brevemente funzionamento e differenze tra le funzioni MOV, MVN e MOVT.
2. Di quanti registri vi è bisogno per conservare il risultato di un operazione come UMLAL?

## Risposte
1. Le funzioni MOV, MVN e MOVT fanno parte dell'Instruction Sets della famiglia di microcontrollori ARM Cortex-M. In base alla famiglia di processori,è possibile implementare un set di istruzioni piu o meno esteso. Ad esempio, le istruzioni MOV, MVN sono
implementate in dal Cortex M0/M1, mentre MOVT è fornito a partire dai Cortex-M3.
  1. MOV: copia il valore di un registro o una costante in un secondo registro.
  2. MVN: copia il valore negato di un registro o una costante in un secondo registro.
  3. MOVT: move top, copia i primi 16 bit nella parte alta del registro, trascurando quelli in basso. 

2. UMLAL (unsigned long multiply-subtract), è una istruzione di Long Multiplication. Si tratta in particolare di una Unsigned multiply with accumulate, ovvero restituisce il risultato senza segno della seguente operazione: RdHi,RdLo <- unsigned(RdHi,RdLo + Rn ×
Rm). In particolare, il risultato avrà una grandezza pari a 64 bit, memorizzati dunque su due registri in quanto un singolo registro nell'architettura ARM Cortex-M ha una capienza
di 32 bits (ovvero una word).
