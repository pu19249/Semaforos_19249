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
  
//<editor-fold defaultstate="collapsed" desc="macros">
reiniciar MACRO
    MOVLW	255     ;para los 10ms con el clock 250kHz
    MOVWF	TMR0	;mover este valor inicial al timer0
    BCF		T0IF	;
ENDM

delay	MACRO
    MOVLW	200
    MOVWF	delay_small
    DECFSZ	delay_small, 1
    GOTO	$-1
ENDM	//</editor-fold>

;===============================================================================
;			CONFIGURACION DE RESET
;===============================================================================

//<editor-fold defaultstate="collapsed" desc="reset">
PSECT resVect, class=code, abs, delta=2
ORG 00h				;posicion 0000h para el reset
resetVec:
    PAGESEL main
    GOTO	main//</editor-fold>
	
;===============================================================================
;			DECLARACION DE VARIABLES
;===============================================================================
	
//<editor-fold defaultstate="collapsed" desc="variables">
PSECT udata_bank0
    estado:		    DS 1
    transistores:	    DS 1
    display:		    DS 8
    tiempo1:		    DS 1
    tiempo2:		    DS 1
    tiempo3:		    DS 1
    temporal:		    DS 1
    decenas_t1:		    DS 1
    unidades_t1:	    DS 1
    dividendo_t1:	    DS 1
    decenas_t2:		    DS 1
    unidades_t2:	    DS 1
    dividendo_t2:	    DS 1
    unidades_t3:	    DS 1
    decenas_t3:		    DS 1
    dividendo_t3:	    DS 1
    decenas_temp:	    DS 1
    unidades_temp:	    DS 1
    dividendo_temp:	    DS 1
    verde_normal:	    DS 1
    verde_titilante:	    DS 1
    amarillo:		    DS 1
    contador:		    DS 1
    display_seven:	    DS 2
    resta_t1:		    DS 1
    tiempo_rojo_t1:	    DS 2
    tiempo_rojo_t2:	    DS 2
    tiempo_rojo_t3:	    DS 2
    cambio_modos:	    DS 1
    normal_1:		    DS 1
    normal_2:		    DS 1
    normal_3:		    DS 1
    tiempo_general:	    DS 1
    decenas_tiempo_general: DS 1
    unidades_tiempo_general:DS 1
    tiempo_1_temporal:	    DS 1
    tiempo_2_temporal:	    DS 1
    tiempo_3_temporal:	    DS 1
	
    cambio_colores:	    DS 1
    cambio_colores_2:	    DS 1
    verde_t1:		    DS 1
    verde_titilante_t1:	    DS 1
    amarillo_t1:	    DS 1
    rojo_t1:		    DS 1
    verde_t2:		    DS 1
    verde_titilante_t2:	    DS 1
    amarillo_t2:	    DS 1
    rojo_t2:		    DS 1
    verde_t3:		    DS 1
    verde_titilante_t3:	    DS 1
    amarillo_t3:	    DS 1
    rojo_t3:		    DS 1
	
    funcionar:		    DS 1
    delay_small:	    DS 1
    ver1:		    DS 1
    
    resta:		    DS 1
    verdec:		    DS 1
    verdet:		    DS 1

PSECT udata_shr
    W_TEMP:		    DS 1
    STATUS_TEMP:	    DS 1

    
    GLOBAL estado, transistores, display, tiempo1, tiempo2, tiempo3, decenas_t1
    GLOBAL unidades_t1, decenas_t2, unidades_t2, decenas_t3, unidades_t3//</editor-fold>

;===============================================================================
;			VECTOR DE INTERRUPCION
;===============================================================================

//<editor-fold defaultstate="collapsed" desc="interrupciones">
PSECT intVect, class=code, abs, delta=2
ORG 04h			;posicion0 0004h para la interrupcion

PUSH:
    MOVWF   W_TEMP
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP
    
ISR: ;revision de los botones del puerto B
    BTFSC   RBIF		    ;hubo botonazo en el puerto B
    CALL    modos	    ;por si no hubo interrupcion
    BCF	    RBIF
    BTFSC   T0IF	    ;revisa si hubo overflow en el timer0
    CALL    int_t0	    ;subrutina timer0
    BTFSC   TMR1IF
    CALL    reiniciar_tmr1
    BCF	    TMR1IF
    
POP:
    SWAPF   STATUS_TEMP, W
    MOVWF   STATUS
    SWAPF   W_TEMP, F
    SWAPF   W_TEMP,W
    RETFIE	       ;stack is popped in the PC//</editor-fold>

;===============================================================================
;				TABLA
;===============================================================================
    
//<editor-fold defaultstate="collapsed" desc="tabla">
    PSECT code, delta=2, abs
    ORG 100h

tabla_disp:		    ;tabla para el display de 7seg
    CLRF	PCLATH
    BSF		PCLATH, 0   ;PCLATH = 01 PCL =02
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
    MOVLW	10
    MOVWF	tiempo1
    MOVWF	normal_1
    MOVLW	10
    MOVWF	tiempo2
    MOVWF	normal_2
    MOVLW	10
    MOVWF	tiempo3
    MOVWF	normal_3
    
    MOVLW	10		;mueve el valor decimal al tiempo1
    MOVWF	tiempo_general	;
    
    BSF		funcionar, 0
    ;CLRF	cambio_colores
    ;CLRF	cambio_colores_2
    
    //</editor-fold>

;===============================================================================
;			    LOOP PRINCIPAL
;===============================================================================
    
//<editor-fold defaultstate="collapsed" desc="loop">
loop:
   
    BTFSC    funcionar, 0
    CALL     colores
   ;revisar a que modo me voy
    MOVLW    1
    SUBWF    cambio_modos, 0	    ;para que no se altere el valor en la variable
    BTFSC    STATUS, 2
    CALL     config_v1
    MOVLW    2
    SUBWF    cambio_modos, 0	    ;para que no se altere el valor en la variable
    BTFSC    STATUS, 2
    CALL     config_v2
    MOVLW    3
    SUBWF    cambio_modos, 0	    ;para que no se altere el valor en la variable
    BTFSC    STATUS, 2
    CALL     config_v3
    MOVLW    4
    SUBWF    cambio_modos, 0	    ;para que no se altere el valor en la variable
    BTFSC    STATUS, 2
    CALL     aceptar_rechazar
    MOVLW    5
    SUBWF    cambio_modos, 0
    BTFSC    STATUS, 2
    CLRF     cambio_modos
   
    CALL     division_decenas_t1
    CALL     division_decenas_t2
    CALL     division_decenas_t3
    CALL     division_decenas_temp
    CALL     mostrar_display

    GOTO	loop//</editor-fold>

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
    
division_decenas_temp:
    MOVF	tiempo_general, 0
    MOVWF	dividendo_temp	    ;para que me divida en centenas mi tiempo1		    
    CLRF	decenas_tiempo_general	            ;para asgurar que se inicia en cero el proceso
    MOVLW	10		    ;le resto una vez 100
    SUBWF	dividendo_temp, 0	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	decenas_tiempo_general		    ;cuantas centenas caben en el numero
    BTFSC	STATUS, 0	    ;
    MOVWF	dividendo_temp	    ;el resultado de la resta estaba en W ahora en dividendo
    BTFSC	STATUS, 0	    ;un tercer BTFSC para ver hasta cuando repito la operacion o si sigue a decenas
    GOTO	$-7		    ;se repite si cabe otra centena
    CALL	division_unidades_temp
    RETURN
division_unidades_temp:
    CLRF	unidades_tiempo_general	    ;para asgurar que se inicia en cero el proceso
    MOVLW	1		    ;le resto una vez 100
    SUBWF	dividendo_temp, F	    ;lo guardo en W
    BTFSC	STATUS, 0	    ;Skip if clear, porque cuando haya un resultado valido se activara
    INCF	unidades_tiempo_general	    ;cuantas centenas caben en el numero
    BTFSS	STATUS, 0	    ;es cuando ya se completo el numero     
    RETURN
    GOTO	$-6		    ;se repite si cabe otra centena
    
    //</editor-fold>

//<editor-fold defaultstate="collapsed" desc="cambio de modos-inc/dec tiempo general">
modos:
    BTFSS   PORTB, 0
    INCF    cambio_modos	;con esto reviso mi cambio de modos con restas
    
    
    ;revisar si quiero incrementar mi tiempo
    BTFSS   PORTB, 1
    INCF    tiempo_general	;le suma 1 al tiempo1
    MOVLW   21
    SUBWF   tiempo_general, 0
    BTFSS   STATUS, 2	;mira si ya llego a 20
    GOTO    $+3
    MOVLW   10		;mueve 10 a tiempo1 (valor decimal)
    MOVWF   tiempo_general
    
    ;revisar si quiero decrementar mi tiempo
    BTFSS   PORTB, 2
    DECF    tiempo_general
    MOVLW   9
    SUBWF   tiempo_general, 0
    BTFSS   STATUS, 2
    GOTO    $+3
    MOVLW   20
    MOVWF   tiempo_general
    RETURN    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="decrementar tiempos">
reiniciar_tmr1:
    BTFSC	TMR1IF
    BANKSEL	TMR1L
    MOVLW	0x7C
    MOVWF	TMR1L
    MOVLW	0xE1	
    MOVWF	TMR1H
    ;DECF	tiempo1
    ;DECF	tiempo2
    ;DECF	tiempo3
    INCF	resta_t1	    ;para el cambio de los colores
    BTFSC	estado, 0
    GOTO	dec_tiempo2
    
    BTFSC	estado, 1
    GOTO	dec_tiempo3
    
dec_tiempo1:
    DECF	tiempo1
    BTFSS	STATUS, 2
    GOTO	regresar
    MOVF	normal_1, 0
    MOVWF	tiempo1
    BSF		estado, 0
    RETURN
    
dec_tiempo2:
    DECF	tiempo2
    BTFSS	STATUS, 2
    GOTO	regresar
    MOVF	normal_2, 0
    MOVWF	tiempo2
    BCF		estado, 0
    BSF		estado, 1
    RETURN
    
dec_tiempo3:
    DECF	tiempo3
    BTFSS	STATUS, 2
    GOTO	regresar
    MOVF	normal_3, 0
    MOVWF	tiempo3
    CLRF	estado
    RETURN
    
regresar:
    RETURN//</editor-fold>
       
//<editor-fold defaultstate="collapsed" desc="cambio de colores semaforos">
colores:
    
    btfsc   cambio_colores, 0
    goto    sema02
    
    btfsc   cambio_colores, 1
    goto    sema03
    
    btfsc   cambio_colores, 2
    goto    sema04 

    btfsc   cambio_colores, 3
    goto    sema05
    
    btfsc   cambio_colores, 4
    goto    sema06
    
    btfsc   cambio_colores, 5
    goto    sema07 
    
    btfsc   cambio_colores, 6
    goto    sema08 
    
    btfsc   cambio_colores, 7
    goto    sema09 
    
    btfsc   cambio_colores_2, 0
    goto    reseteo
    
sema01:
    bcf	    STATUS, 2
    bcf	    PORTA, 0	; Rojo s1
    bcf	    PORTA, 1	; Amarillo s1
    bsf	    PORTA, 2	; Verde s1
    bsf	    PORTA, 3	; Rojo s2
    bcf	    PORTA, 4	; Amarillo s2
    bcf	    PORTA, 5	; Verde s2
    bsf	    PORTB, 5	; Rojo s3
    bcf	    PORTB, 6	; Amarillo s3
    bcf	    PORTB, 7	; Verde s3
    
    movf    tiempo1, w
    movwf   verdec
    movlw   6
    subwf   verdec, 1
    movf    verdec, w
    movwf   resta
    movf    resta_t1, w
    subwf   verdec, 1
    btfss   STATUS, 2
    goto    $+3
    bcf	    PORTA, 2
    bsf	    cambio_colores, 0
    return
sema02:  
    bcf	    STATUS, 2
    bsf	    PORTA, 2
    delay
    bcf	    PORTA, 2
    movlw   3
    addwf   resta, w
    movwf   verdet
    movf    resta_t1, w
    subwf   verdet, 1
    btfss   STATUS, 2
    goto    $+3
    bcf	    cambio_colores, 0
    bsf	    cambio_colores, 1
    return
sema03:
    bcf	    STATUS, 2
    bsf	    PORTA, 1
    movlw   6
    addwf   resta, w
    movwf   amarillo
    movf    resta_t1, w
    subwf   amarillo, 1
    btfss   STATUS, 2
    goto    $+5
    bcf	    cambio_colores, 1
    bsf	    cambio_colores, 2
    bcf	    PORTA, 1
    CLRF    resta_t1
    return
sema04:
    bcf	    STATUS, 2
    bcf	    PORTA, 3
    bsf	    PORTA, 0
    bsf	    PORTA, 5
    movf    tiempo2, w
    movwf   verdec 
    movlw   6
    subwf   verdec, 1
    movf    verdec, w
    movwf   resta
    movf    resta_t1, w
    subwf   verdec, 1
    btfss   STATUS, 2
    goto    $+4
    bcf	    PORTA, 5
    bcf	    cambio_colores, 2
    bsf	    cambio_colores, 3    
    return
sema05:
    bcf	    STATUS, 2
    bsf	    PORTA, 5
    delay
    bcf	    PORTA, 5
    movlw   3
    addwf   resta, w
    movwf   verdet
    movf    resta_t1, w
    subwf   verdet, 1
    btfss   STATUS, 2
    goto    $+3
    bcf	    cambio_colores, 3
    bsf	    cambio_colores, 4
    return
sema06:
    bcf	    STATUS, 2
    bsf	    PORTA, 4
    movlw   6
    addwf   resta, w
    movwf   amarillo
    movf    resta_t1, w
    subwf   amarillo, 1
    btfss   STATUS, 2
    goto    $+5
    bcf	    cambio_colores, 4
    bsf	    cambio_colores, 5
    bcf	    PORTA, 4
    CLRF    resta_t1
    return
sema07:
    bcf	    STATUS, 2
    bcf	    PORTB, 5	    ;ROJO S3
    bsf	    PORTA, 3
    bsf	    PORTB, 7	    ;VERDE S3
    movf    tiempo2, w
    movwf   verdec 
    movlw   5
    subwf   verdec, 1
    movf    verdec, w
    movwf   resta
    movf    resta_t1, w
    subwf   verdec, 1
    btfss   STATUS, 2
    goto    $+4
    bcf	    PORTB, 7
    bcf	    cambio_colores, 5
    bsf	    cambio_colores, 6    
    return    
sema08:
    bcf	    STATUS, 2
    bsf	    PORTB, 7
    delay
    bcf	    PORTB, 7
    movlw   3
    addwf   resta, w
    movwf   verdet
    movf    resta_t1, w
    subwf   verdet, 1
    btfss   STATUS, 2
    goto    $+3
    bcf	    cambio_colores, 6
    bsf	    cambio_colores, 7
    return   
sema09:
    bcf	    STATUS, 2
    bsf	    PORTB, 6
    movlw   6
    addwf   resta, w
    movwf   amarillo
    movf    resta_t1, w
    subwf   amarillo, 1
    btfss   STATUS, 2
    goto    $+6
    bcf	    cambio_colores, 7
    bsf	    cambio_colores_2, 0
    bcf	    PORTB, 6
    bsf	    PORTB, 5
    CLRF    resta_t1
    return
reseteo:
    clrf    verdec
    clrf    verdet
    clrf    amarillo
    clrf    resta
    clrf    cambio_colores
    bcf     cambio_colores_2, 0
    ;clrf    STATUS
    return
    //</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="mostrar display">
mostrar_display:
    MOVF	decenas_t1, 0
    CALL	tabla_disp
    MOVWF	display+0
    MOVF	unidades_t1, 0
    CALL	tabla_disp
    MOVWF	display+1
    MOVF	decenas_t2, 0
    CALL	tabla_disp
    MOVWF	display+2
    MOVF	unidades_t2, 0
    CALL	tabla_disp
    MOVWF	display+3
    MOVF	decenas_t3, 0
    CALL	tabla_disp
    MOVWF	display+4
    MOVF	unidades_t3, 0
    CALL	tabla_disp
    MOVWF	display+5
    MOVF	decenas_tiempo_general, 0
    CALL	tabla_disp
    MOVWF	display+6
    MOVF	unidades_tiempo_general, 0
    CALL	tabla_disp
    MOVWF	display+7
    RETURN
	//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="modos de configuracion">
config_v1:
    BSF		funcionar, 1
    BSF		funcionar, 2
    BSF		funcionar, 3
    BCF		PORTE, 0
    BSF		PORTE, 1
    BCF		PORTE, 2
    MOVF	tiempo_general, 0
    MOVWF	tiempo_1_temporal
    RETURN
    
config_v2:
    BSF		funcionar, 1
    BSF		funcionar, 2
    BSF		funcionar, 3
    BSF		PORTE, 0
    BSF		PORTE, 1
    BCF		PORTE, 2
    MOVF	tiempo_general, 0
    MOVWF	tiempo_2_temporal
    RETURN
    
config_v3:
    BSF		funcionar, 1
    BSF		funcionar, 2
    BSF		funcionar, 3
    BCF		PORTE, 0
    BCF		PORTE, 1
    BSF		PORTE, 2
    MOVF	tiempo_general, 0
    MOVWF	tiempo_3_temporal
    
    RETURN
    
aceptar_rechazar:
    BCF		funcionar, 1
    BCF		funcionar, 2
    BCF		funcionar, 3
    BSF		PORTE, 0
    BCF		PORTE, 1
    BSF		PORTE, 2
    ;enviar los tiempos temporales a las variables del modo normal
    BTFSS	PORTB, 1
    CALL    	aceptar
    BTFSS	PORTB, 2
    CALL	rechazar
    RETURN
aceptar:
    BCF	funcionar, 0
    BSF	PORTE, 0
    BSF	PORTE, 1
    BSF	PORTE, 2
    ;colocar todo en rojo
    MOVF	tiempo_1_temporal, 0	    ;el tiempo que inc o dec
    MOVWF	tiempo1			    ;esto se muestra en el disp
    MOVWF	normal_1		    ;con esto opero
    MOVF	tiempo_2_temporal, 0	    ;el tiempo que inc o dec
    MOVWF	tiempo2			    ;esto se muestra en el disp
    MOVWF	normal_2		    ;con esto opero
    ;MOVF	normal_1, 0		    ;normal_1 a W para sumarlo con 
    ;ADDWF	tiempo2			    ;tiempo2 entonces tiempo2=normal_1+tiempo2
    MOVF	tiempo_3_temporal, 0	    ;el tiempo que inc o dec
    MOVWF	tiempo3			    ;esto se muestra en el disp
    MOVWF	normal_3		    ;con esto opero
    ;MOVF	tiempo2, 0		    ;tiempo2 a w, es decir normal_1+tiempo2
    ;ADDWF	tiempo3			    ;sumado con tiempo3=tiempo2+normal_1
    ;CLRF	cambio_modos
    
    
    CLRF	cambio_colores
    CLRF	cambio_colores_2
    CLRF	verdec
    CLRF	verdet
    CLRF	amarillo
    CLRF	resta
    CLRF	resta_t1
    CLRF	estado
    BSF		funcionar, 0
    RETURN
rechazar:
    BCF		funcionar, 0
    CLRF	cambio_modos
    CLRF	cambio_colores
    CLRF	cambio_colores_2
    BSF		funcionar, 0
    RETURN//</editor-fold>

;===============================================================================
;			    SUBRUTINAS DE INTERRUPCION				
;===============================================================================
    
//<editor-fold defaultstate="collapsed" desc="desplegar valores en display">
int_t0: 
    reiniciar
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
    MOVF	display+0,0
    MOVWF	PORTC
    BSF		PORTD, 0 
    GOTO	next1
display2:   ;parte unidades via1
    MOVF	display+1, 0
    MOVWF	PORTC
    BSF		PORTD, 1
    GOTO	next2
display3:
    MOVF	display+2, 0
    MOVWF	PORTC
    BSF		PORTD, 2
    GOTO	next3
display4:
    MOVF	display+3, 0
    MOVWF	PORTC
    BSF		PORTD, 3
    GOTO	next4
display5:
    MOVF	display+4, 0
    MOVWF	PORTC
    BSF		PORTD, 4
    GOTO	next5
display6:
    MOVF	display+5, 0
    MOVWF	PORTC
    BSF		PORTD, 5
    GOTO	next6
display7:
    BTFSS	funcionar, 1
    GOTO	next7
    MOVF	display+6, 0
    MOVWF	PORTC
    BSF		PORTD, 6
    GOTO	next7
display8:
    BTFSS	funcionar, 1
    GOTO	next8
    MOVF	display+7, 0
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
    CLRF	transistores	;para que vaya al primer display
    RETURN
    //</editor-fold>

END