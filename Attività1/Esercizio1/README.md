# Esercizio 1
## Tradurre in codice Assembly la seguente funzione C:

```c
void c_strcpy(const char *src, char *dst){
	int i = 0;
	char c = src[i];

	while(c != '\0'){
		dst[i] = c;
		i += 1;
		c = src[i];
	}
	dst[i] = '\0';
}

```

## Svolgimento:

```assembly
    AREA main, CODE
    EXPORT __main
    ENTRY
__main PROC
    DCB "Sono una stringa",0    ; inizializzo una stringa con termine "\0"
    BL c_strlen                 ; aggiorna lr, salta all'indirizzo indicato. Il risultato viene restituito in r0!
    ENDP
    END

c_strlen
    MOV r1, #0                  ; inizializzo la variabile i
loop LDRB r2, [r0]              ; carico il char puntato a r0 in r1
    ADDS r1, #1                 ; incremento i
    CMP r2, #0                  ; confronto e aggiorno il Program Status Register (PSR)
    BNE loop                    ; se non è zero, torna a loop
    BX lr                       ; salta all'indirizzo indicato
```
