; Archivo: main_p01.s
; Dispositivo: PIC16F887
; Autor: Jonathan Pu
; Compilador: pic-as (v2.30), MPLABX V5.45
; Programa: configuracion de 3 semaforos para una sola via
; Hardware: traffic_lights, 8 display 7seg, 3leds para indicar el modo de 
; de funcionamiento, 3 push para modo, inc y dec
; Creado: 15 de marzo de 2021
; Ultima modificacion: ____________________________________

PROCESSOR 16F887
#include <xc.inc>

;===============================================================================
;			PALABRAS DE CONFIGURACION
;===============================================================================
; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)
  
  
;===============================================================================
;			    MACROS
;===============================================================================
  
reiniciar MACRO
    MOVLW	254     ;para los 10ms con el clock 250kHz
    MOVWF	TMR0	;mover este valor inicial al timer0
    BCF		T0IF	;
ENDM
  
;===============================================================================
;			CONFIGURACION DE RESET
;===============================================================================

PSECT resVect, class=code, abs, delta=2
ORG 00h				;posicion 0000h para el reset
    resetVec:
	PAGESEL main
	GOTO	main
	
;===============================================================================
;			DECLARACION DE VARIABLES
;===============================================================================
PSECT udata_bank0
	estado:		    DS 1
	transistores:	    DS 1
	display:	    DS 8
	tiempo1:	    DS 1
	tiempo2:	    DS 1
	tiempo3:	    DS 1
	decenas_t1:	    DS 1
	unidades_t1:	    DS 1
	dividendo_t1:	    DS 1
	decenas_t2:	    DS 1
	unidades_t2:	    DS 1
	dividendo_t2:	    DS 1
	unidades_t3:	    DS 1
	decenas_t3:	    DS 1
	dividendo_t3:	    DS 1
    
PSECT udata_shr
	W_TEMP:		    DS 1
	STATUS_TEMP:	    DS 1

;===============================================================================
;			VECTOR DE INTERRUPCION
;===============================================================================

PSECT intVect, class=code, abs, delta=2
ORG 04h			;posicion0 0004h para la interrupcion

PUSH:
    MOVWF   W_TEMP
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP
    
ISR: ;revision de los botones del puerto B
    BTFSS   RBIF		    ;hubo botonazo en el puerto B
    CALL    modos_prueba	    ;por si no hubo interrupcion
    BCF	    RBIF
    BTFSC   T0IF	    ;revisa si hubo overflow en el timer0
    CALL    int_t0	    ;subrutina timer0
    
POP:
    SWAPF   STATUS_TEMP, W
    MOVWF   STATUS
    SWAPF   W_TEMP, F
    SWAPF   W_TEMP,W
    RETFIE	       ;stack is popped in the PC
    

    
;===============================================================================
;				TABLA
;===============================================================================
    
//<editor-fold defaultstate="collapsed" desc="tabla">
    PSECT code, delta=2, abs
    ORG 100h

tabla_disp:		    ;tabla para el display de 7seg
    CLRF	PCLATH
    BSF		PCLATH, 0   ;PCLATH = 01 PCL =02
    ANDLW	0x0f
    ADDWF	PCL	    ;PCL=PCLATH +PCL+W
    RETLW	00111111B   ;0
    RETLW	00000110B   ;1
    RETLW	01011011B   ;2
    RETLW	01001111B   ;3
    RETLW	01100110B   ;4
    RETLW	01101101B   ;5
    RETLW	01111101B   ;6
    RETLW	00000111B   ;7
    RETLW	01111111B   ;8
    RETLW	01101111B   ;9
    RETLW	01110111B   ;A
    RETLW	01111100B   ;B
    RETLW	00111001B   ;C
    RETLW	01011110B   ;D
    RETLW	01111001B   ;E
    RETLW	01110001B   ;F
    
    //</editor-fold>

;===============================================================================
;		       CONFIGURACION DE PUERTOS
;===============================================================================

//<editor-fold defaultstate="collapsed" desc="main">
main:
    ;configuracion del puerto B (botones y semaforo via 3)
    BANKSEL	ANSEL
    CLRF	ANSEL
    CLRF	ANSELH
    BANKSEL	TRISB
    BSF		TRISB, 0    ;boton1
    BSF		TRISB, 1    ;boton2
    BSF		TRISB, 2    ;boton3
    BCF		TRISB, 5    ;semaforo3_rojo
    BCF		TRISB, 6    ;semaforo3_amarillo
    BCF		TRISB, 7    ;semaforo3_verde
    
    ;configuracion de los demas puertos como salida
    CLRF	TRISA	    ;leds de los semaforos
    CLRF	TRISC	    ;display 7seg
    CLRF	TRISD	    ;transistores
    CLRF	TRISE	    ;leds indicadores
    
    ;Usar los pull ups internos para estos pines
    BANKSEL	OPTION_REG		
    BCF		OPTION_REG, 7	    ;PORTB pull-ups are enabled by individual PORT latch values
    BANKSEL	WPUB
    MOVLW	00000111B	    ;hay que configurar todos de manera individual
    MOVWF	WPUB
    
    ;configuracion del reloj interno
    BANKSEL	OSCCON
    BCF		IRCF0		    ;el reloj interno 250kHz
    BSF		IRCF1	
    BCF		IRCF2
    BSF		SCS		    ;internal oscillator is used for system clock
    
    ;configurar interrupcion del puerto B
    BANKSEL IOCB
    MOVLW   00000111B	    ;habilita el interrupt on change para los pines RB0 y RB1
    MOVWF   IOCB    
    BANKSEL INTCON	
    BCF	    RBIF	    ;no hay ningun cambio de estado aun - como limpiar puertos
    
    ;configurar bits para interrupciones en general
    BSF	    GIE		    ;habilita las interrupciones globales
    BSF	    RBIE	    ;habilita la interrupcion del puertoB
    BSF	    T0IE	    ;habilita la interrupcion del timer0
    BCF	    T0IF	    ;limpiar puerto para el timer0
    
    ;configurar TMR0
    BANKSEL OPTION_REG
    BCF	    T0CS	    ;oscilador interno
    BCF	    PSA		    ;prescaler asignado al timer0
    BSF	    PS0		    ;prescaler tenga un valor 1:256
    BSF	    PS1
    BSF	    PS2
    
    ;configurar TMR1
    BANKSEL T1CON
    BSF	    T1CKPS1	    ;prescaler de 1:8 para el timer1
    BSF	    T1CKPS0
    BCF	    TMR1CS	    ;internal clock FOSC/4
    BSF	    TMR1ON	    ;enables timer1
     
    ;configurar TMR2
    BANKSEL T2CON
    MOVLW   1001110B	    ;primeros 4bits para postscaler, timer2 is on,
			    ;prescaler is 16
    MOVWF   T2CON	    ;muevo los valores al registro
    
;    ;habilitar interrupciones para TMR1 y TMR2
    BANKSEL PIE1
    BSF	    TMR2IE	    ;enables the timer2 to pr2 match interrupt
    BSF	    TMR1IE	    ;enables the timer1 overflow interrupt
    BANKSEL PIR1
    BCF	    TMR2IF	    ;limpiar banderas para timer2
    BCF	    TMR1IF	    ;limpiar bandera de interrupcio para timer1
    
    ;limpiar los puertos
    BANKSEL	PORTA
    CLRF	PORTA
    CLRF	PORTB
    CLRF	PORTC
    CLRF	PORTD
    CLRF	PORTE
    CLRF	estado
    //</editor-fold>

;===============================================================================
;			    LOOP PRINCIPAL
;===============================================================================
    
loop:
    CALL	normal		;comienza en el modo normal
    ;CALL	multiplex	;para que esten multiplexeando siempre los disp
    CALL	seleccion

    
    GOTO	loop
;===============================================================================
;			    SUBRUTINAS
;===============================================================================
//<editor-fold defaultstate="collapsed" desc="rutinas de division para los tiempos">
division_decenas_t1:
    MOVF	tiempo1, 0
    MOVWF	dividendo_t1	    ;para que me divida en centenas mi tiempo1		    
    CLRF	decenas_t1	            ;para asgurar que se inicia en cero el proceso
    MOVLW	10		    ;le resto una vez 100
    SUBWF	dividendo_t1, 0	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	decenas_t1		    ;cuantas centenas caben en el numero
    BTFSC	STATUS, 0	    ;
    MOVWF	dividendo_t1	    ;el resultado de la resta estaba en W ahora en dividendo
    BTFSC	STATUS, 0	    ;un tercer BTFSC para ver hasta cuando repito la operacion o si sigue a decenas
    GOTO	$-7		    ;se repite si cabe otra centena
    CALL	division_unidades_t1
    RETURN
division_unidades_t1:
    CLRF	unidades_t1	    ;para asgurar que se inicia en cero el proceso
    MOVLW	1		    ;le resto una vez 100
    SUBWF	dividendo_t1, F	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	unidades_t1	    ;cuantas centenas caben en el numero
    BTFSS	STATUS, 0	    ;es cuando ya se completo el numero     
    RETURN
    GOTO	$-6		    ;se repite si cabe otra centena
    
division_decenas_t2:
    MOVF	tiempo2, 0
    MOVWF	dividendo_t2	    ;para que me divida en centenas mi tiempo1		    
    CLRF	decenas_t2	            ;para asgurar que se inicia en cero el proceso
    MOVLW	10		    ;le resto una vez 100
    SUBWF	dividendo_t2, 0	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	decenas_t2		    ;cuantas centenas caben en el numero
    BTFSC	STATUS, 0	    ;
    MOVWF	dividendo_t2	    ;el resultado de la resta estaba en W ahora en dividendo
    BTFSC	STATUS, 0	    ;un tercer BTFSC para ver hasta cuando repito la operacion o si sigue a decenas
    GOTO	$-7		    ;se repite si cabe otra centena
    CALL	division_unidades_t2
    RETURN
division_unidades_t2:
    CLRF	unidades_t2	    ;para asgurar que se inicia en cero el proceso
    MOVLW	1		    ;le resto una vez 100
    SUBWF	dividendo_t2, F	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	unidades_t2	    ;cuantas centenas caben en el numero
    BTFSS	STATUS, 0	    ;es cuando ya se completo el numero     
    RETURN
    GOTO	$-6		    ;se repite si cabe otra centena
    
    
division_decenas_t3:
    MOVF	tiempo3, 0
    MOVWF	dividendo_t3	    ;para que me divida en centenas mi tiempo1		    
    CLRF	decenas_t3	            ;para asgurar que se inicia en cero el proceso
    MOVLW	10		    ;le resto una vez 100
    SUBWF	dividendo_t3, 0	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	decenas_t3		    ;cuantas centenas caben en el numero
    BTFSC	STATUS, 0	    ;
    MOVWF	dividendo_t3	    ;el resultado de la resta estaba en W ahora en dividendo
    BTFSC	STATUS, 0	    ;un tercer BTFSC para ver hasta cuando repito la operacion o si sigue a decenas
    GOTO	$-7		    ;se repite si cabe otra centena
    CALL	division_unidades_t3
    RETURN
division_unidades_t3:
    CLRF	unidades_t3	    ;para asgurar que se inicia en cero el proceso
    MOVLW	1		    ;le resto una vez 100
    SUBWF	dividendo_t3, F	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	unidades_t3	    ;cuantas centenas caben en el numero
    BTFSS	STATUS, 0	    ;es cuando ya se completo el numero     
    RETURN
    GOTO	$-6		    ;se repite si cabe otra centena
    //</editor-fold>

normal:
    BSF		PORTE, 0
    BCF		PORTE, 1
    BCF		PORTE, 2
    RETURN


//<editor-fold defaultstate="collapsed" desc="rutinas para inc y dec variables de tiempo">
incrementar1:
    INCF    tiempo1		;le suma 1 al tiempo1
    MOVLW   20
    SUBWF   tiempo1
    BTFSS   STATUS, 2	;mira si ya llego a 20
    BTFSS   PORTB, 1	;si ya llego a 20 y se presiona una vez mas regresa a 10
    MOVLW   10		;mueve 10 a tiempo1 (valor decimal)
    MOVWF   tiempo1
    CALL    division_decenas_t1
    MOVF    decenas_t1
    CALL    tabla_disp
    MOVWF   display+0
    MOVF    unidades_t1
    CALL    tabla_disp  
    MOVWF   display+1
    RETURN
    
decrementar1:
    DECF    tiempo1		;le resta 1 al tiempo1
    MOVLW   10
    SUBWF   tiempo1
    BTFSS   STATUS, 2	;mira si ya llego a 10
    BTFSS   PORTB, 2	;si ya llego a 10 y se presiona una vez mas regresa a 20
    MOVLW   20		;mueve el valor de 20 a tiempo1 (valor decimal)
    MOVWF   tiempo1
    CALL    division_decenas_t1
    MOVF    decenas_t1
    CALL    tabla_disp
    MOVWF   display+0
    MOVF    unidades_t1
    CALL    tabla_disp  
    MOVWF   display+1
    RETURN
    
incrementar2:
    INCF    tiempo2	;le suma 1 al tiempo1
    MOVLW   20
    SUBWF   tiempo2
    BTFSS   STATUS, 2	;mira si ya llego a 20
    BTFSS   PORTB, 1	;si ya llego a 20 y se presiona una vez mas regresa a 10
    MOVLW   10		;mueve 10 a tiempo1 (valor decimal)
    MOVWF   tiempo2
    CALL    division_decenas_t2
    MOVF    decenas_t2
    CALL    tabla_disp
    MOVWF   display+0
    MOVF    unidades_t2
    CALL    tabla_disp  
    MOVWF   display+1
    RETURN
    
decrementar2:
    DECF    tiempo2	;le resta 1 al tiempo1
    MOVLW   10
    SUBWF   tiempo2
    BTFSS   STATUS, 2	;mira si ya llego a 10
    BTFSS   PORTB, 2	;si ya llego a 10 y se presiona una vez mas regresa a 20
    MOVLW   20		;mueve el valor de 20 a tiempo1 (valor decimal)
    MOVWF   tiempo2
    CALL    division_decenas_t2
    MOVF    decenas_t2
    CALL    tabla_disp
    MOVWF   display+0
    MOVF    unidades_t2
    CALL    tabla_disp  
    MOVWF   display+1
    RETURN
    
incrementar3:
    INCF    tiempo3	;le suma 1 al tiempo1
    MOVLW   20
    SUBWF   tiempo3
    BTFSS   STATUS, 2	;mira si ya llego a 20
    BTFSS   PORTB, 1	;si ya llego a 20 y se presiona una vez mas regresa a 10
    MOVLW   10		;mueve 10 a tiempo1 (valor decimal)
    MOVWF   tiempo3
    CALL    division_decenas_t3
    MOVF    decenas_t3
    CALL    tabla_disp
    MOVWF   display+0
    MOVF    unidades_t3
    CALL    tabla_disp  
    MOVWF   display+1
    RETURN
    
decrementar3:
    DECF    tiempo3	;le resta 1 al tiempo1
    MOVLW   10
    SUBWF   tiempo3
    BTFSS   STATUS, 2	;mira si ya llego a 10
    BTFSS   PORTB, 2	;si ya llego a 10 y se presiona una vez mas regresa a 20
    MOVLW   20		;mueve el valor de 20 a tiempo1 (valor decimal)
    MOVWF   tiempo3
    CALL    division_decenas_t3
    MOVF    decenas_t3
    CALL    tabla_disp
    MOVWF   display+0
    MOVF    unidades_t3
    CALL    tabla_disp  
    MOVWF   display+1
    RETURN
    //</editor-fold>

seleccion:
    MOVLW	2
    SUBWF	estado, 0
    BTFSS	STATUS, 2
    CALL	config_v1
    RETURN
    
;===============================================================================
;			    SUBRUTINAS DE INTERRUPCION				
;===============================================================================
    
reiniciar:
   MOVLW	254     ;para los 10ms con el clock 250kHz
   MOVWF	TMR0	;mover este valor inicial al timer0
   BCF		T0IF	;
   RETURN

modos_prueba:
    BTFSS   PORTB, 0
    INCF    estado    
;    MOVLW   6
;    SUBWF   estado, 0
;    BTFSS   STATUS, 2
;    CLRF    estado
    RETURN

    
config_v1:
    BCF		PORTE, 0
    BSF		PORTE, 1
    BCF		PORTE, 2
;    MOVLW	10		;mueve el valor decimal al tiempo1
;    MOVWF	tiempo1		;
;    BTFSS	PORTB, 1	;se revisa si se quiere incrementar el tiempo1
;    GOTO	incrementar1	;
;    BTFSS	PORTB, 2
;    GOTO	decrementar1	;sino se va a decrementar el tiempo 1
;    ;BTFSS	PORTB, 0
;    ;BSF		estado, 1
;    ;BCF		RBIF
    RETURN
    			
    
config_v2:
    BSF		PORTE, 0
    BSF		PORTE, 1
    BCF		PORTE, 2
;    MOVLW	10
;    MOVWF	tiempo2
;    BTFSS	PORTB, 1
;    GOTO  	incrementar2
;    BTFSS	PORTB, 2
;    GOTO   	decrementar2
;    ;BTFSS	PORTB, 0
;    ;BSF		estado, 2
;    ;BCF		RBIF
    RETURN
    
config_v3:
    BCF		PORTE, 0
    BCF		PORTE, 1
    BSF		PORTE, 2
;    MOVLW	10
;    MOVWF	tiempo3
;    BTFSS	PORTB, 1
;    GOTO	incrementar3
;    BTFSS	PORTB, 2
;    GOTO	decrementar3
    ;BTFSS	PORTB, 0
    ;BSF		estado, 3
    ;BCF		RBIF
    RETURN
    

aceptar_rechazar:
    BSF		PORTE, 0
    BCF		PORTE, 1
    BSF		PORTE, 2
;    BTFSS	PORTB, 0
;    CALL	normal
    GOTO	POP
    

    
//<editor-fold defaultstate="collapsed" desc="desplegar valores en display">
int_t0:
    MOVLW	255     ;para los 10ms con el clock 250kHz
    MOVWF	TMR0	;mover este valor inicial al timer0
    BCF		T0IF	;
    CLRF	PORTD			    ;puerto con transistores
    
    BTFSC	transistores, 0
    GOTO	display2
    
    BTFSC	transistores, 1
    GOTO	display3
    
    BTFSC	transistores, 2
    GOTO	display4
    
    BTFSC	transistores, 3
    GOTO	display5
    
    BTFSC	transistores, 4
    GOTO	display6
    
    BTFSC	transistores, 5
    GOTO	display7
    
    BTFSC	transistores, 6
    GOTO	display8
    
display1:   ;parte decenas via1
    MOVF	display+0
    MOVWF	PORTC
    BSF		PORTD, 0 
    GOTO	next1
display2:   ;parte unidades via1
    MOVF	display+1
    MOVWF	PORTC
    BSF		PORTD, 1
    GOTO	next2
display3:
    MOVF	display+2
    MOVWF	PORTC
    BSF		PORTD, 2
    GOTO	next3
display4:
    MOVF	display+3
    MOVWF	PORTC
    BSF		PORTD, 3
    GOTO	next4
display5:
    MOVF	display+4
    MOVWF	PORTC
    BSF		PORTD, 4
    GOTO	next5
display6:
    MOVF	display+5
    MOVWF	PORTC
    BSF		PORTD, 5
    GOTO	next6
display7:
    MOVF	display+6
    MOVWF	PORTC
    BSF		PORTD, 6
    GOTO	next7
display8:
    MOVF	display+7
    MOVWF	PORTC
    BSF		PORTD, 7
    GOTO	next8
    
    
next1:
    MOVLW	00000001B
    XORWF	transistores, 1
    RETURN
next2:
    MOVLW	00000011B
    XORWF	transistores, 1
    RETURN
next3:
    MOVLW	00000110B
    XORWF	transistores, 1
    RETURN
next4:
    MOVLW	00001100B
    XORWF	transistores, 1
    RETURN
next5:
    MOVLW	00011000B
    XORWF	transistores, 1
    RETURN
next6:
    MOVLW	00110000B
    XORWF	transistores, 1
    RETURN
next7:
    MOVLW	01100000B
    XORWF	transistores, 1
    RETURN
next8:
    CLRF    transistores	;para que vaya al primer display
    RETURN
    //</editor-fold>

END