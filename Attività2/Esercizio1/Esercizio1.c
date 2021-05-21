/*Scrivere del codice C che, operando su un valore a 8 bit (unsigned char), configuri i bit 4,5 e 6 rispettivamente ai valori 1, 0 e 1; lasciando tutti i restanti a 0. 
Se l’operazione pu`o essereeseguita con diversi operatori, riportare nel codice tutte le soluzioni che vengono in mente.*/

#include <stdio.h>
#include <stdlib.h>

unsigned char val = 0x0F;   // 0x0F -   00001111    -   15
const unsigned char mask = 0x28;  // 0x28 -   00101000    -   40

unsigned char setBit(unsigned char C){
    C = mask;   // Set bit[4] & bit[6] and clear other bits
    return C;
}

unsigned char setBit2(unsigned char C){
    C &= mask;  // Clear all bits except bit[4] & bit[6]
    C |= mask;  // Set bit[4] & bit[6] and keep others unchanged
    return C;
}

int main()
{
    printf("valore iniziale = %d\n", val);
    printf("valore funzione = %d\n", setBit(val));
    printf("valore funzione 2 = %d\n", setBit2(val));
    return  0;
}