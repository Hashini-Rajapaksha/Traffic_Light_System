
	;=====================================================================
; IE3064 - Smart Traffic Junction Controller
; Target    : Arduino Uno (ATmega328P, 16 MHz)
; Tool      : Microchip Studio 7 - AVR Assembler project (AVRASM2)
; Language  : AVR Assembly only 
;
; Direct translation of the C++ prototype:
;   millis()        -> Timer1 compare-match interrupt every 1 ms
;   digitalRead     -> pin-change interrupt on PD7 + 30 ms debounce
;   analogRead      -> direct ADC register access (ADC5)
;   digitalWrite    -> whole-port writes (PORTB / PORTC / PORTD)
;   timing values   -> tables in program memory, read with LPM
;   Serial          -> not used (calibrate the LDR with the C++ version)
;
; PIN MAP
;   PB0 D8  NS red      PB3 D11 EW red
;   PB1 D9  NS yellow   PB4 D12 EW yellow
;   PB2 D10 NS green    PB5 D13 EW green
;   PD7 D7  pedestrian button to GND (internal pull-up, PCINT23)
;   PC0 A0  pedestrian red      PC1 A1 pedestrian green
;   PC3 A3  segment a           PC2 A2 segment b
;   PD2..PD6 (D2..D6) segments c,d,e,f,g   (common anode: 0 = ON)
;   PC4 A4  buzzer 
;   PC5 A5  ADC5 LDR divider: 5V - LDR - A5 - 10k - GND
;=====================================================================

.include "m328pdef.inc"

;---------------- light patterns written to PORTB --------------------
; bit: 5=EW_G 4=EW_Y 3=EW_R 2=NS_G 1=NS_Y 0=NS_R
.equ L_NS_GO    = 0x0C          ; NS green  + EW red
.equ L_NS_YEL   = 0x0A          ; NS yellow + EW red
.equ L_ALL_RED  = 0x09          ; NS red    + EW red
.equ L_EW_GO    = 0x21          ; EW green  + NS red
.equ L_EW_YEL   = 0x11          ; EW yellow + NS red
.equ L_NIGHT    = 0x12          ; NS yellow + EW yellow

;---------------- states (same numbers as the C++ enum) --------------
.equ NS_READY    = 0
.equ NS_GO       = 1
.equ NS_STOPPING = 2
.equ AFTER_NS    = 3
.equ EW_READY    = 4
.equ EW_GO       = 5
.equ EW_STOPPING = 6
.equ AFTER_EW    = 7
.equ PEDESTRIAN  = 8

;---------------- constants ------------------------------------------
.equ NIGHT_THRESHOLD = 400      ; 10-bit ADC: <= 400 means dark
.equ RECOVERY_TIME   = 5000     ; ms of continuous light to leave night
.equ FLASH_INTERVAL  = 500      ; 500 ms on + 500 ms off = 1 Hz
.equ DEBOUNCE_TIME   = 30       ; ms
.equ ONE_SECOND      = 1000     ; ms

;=====================================================================
; VARIABLES IN SRAM
;=====================================================================
.dseg
.org SRAM_START
v_ticks:     .byte 1            ; 1 ms ticks waiting (written by ISR)
v_debounce:  .byte 1            ; ms until button is stable (ISR)
v_btnCheck:  .byte 1            ; 1 = button stable, check it (ISR)
v_stable:    .byte 1            ; accepted button level (0x80 = up)
v_state:     .byte 1            ; current state 0..8
v_resume:    .byte 1            ; state to continue after crossing
v_request:   .byte 1            ; pedestrian request flag
v_nightMode: .byte 1            ; 1 = night mode
v_showZero:  .byte 1            ; show "0" during the READY state
v_stateMs:   .byte 2            ; ms spent in current state (16-bit)
v_nightMs:   .byte 2            ; flash timer 0..999 (16-bit)
v_recovMs:   .byte 2            ; continuous light time (16-bit)
v_ldr:       .byte 2            ; last ADC reading (16-bit)
v_end:

;=====================================================================
; INTERRUPT VECTORS
;=====================================================================
.cseg
.org 0x0000
        jmp  RESET_HANDLER      ; reset
.org PCI2addr
        jmp  ISR_PCINT2         ; pin change on PORTD (button)
.org OC1Aaddr
        jmp  ISR_T1_COMPA       ; Timer1 compare A (1 ms)
.org INT_VECTORS_SIZE           ; program starts after the vector table

;=====================================================================
; PROGRAM-MEMORY TABLES  (address * 2 because LPM uses byte addresses)
;=====================================================================
; PORTB light pattern for each state (last 0 = padding to even size)
STATE_LIGHTS:
        .db L_NS_YEL,  L_NS_GO,  L_NS_YEL,  L_ALL_RED   ; 0..3
        .db L_EW_YEL,  L_EW_GO,  L_EW_YEL,  L_ALL_RED   ; 4..7
        .db L_ALL_RED, 0                                ; 8

; Duration of each state in ms
STATE_TIME:
        .dw 1000                ; 0 NS_READY     (READY_TIME)
        .dw 10000               ; 1 NS_GO        (GREEN_TIME)
        .dw 3000                ; 2 NS_STOPPING  (YELLOW_TIME)
        .dw 1000                ; 3 AFTER_NS     (ALL_RED_TIME)
        .dw 1000                ; 4 EW_READY
        .dw 10000               ; 5 EW_GO
        .dw 3000                ; 6 EW_STOPPING
        .dw 1000                ; 7 AFTER_EW
        .dw 5000                ; 8 PEDESTRIAN   (PED_TIME)

; 7-segment font for digits 0..9: bit0=a ... bit6=g, 1 = segment lit
; (kept on ONE line: AVRASM pads every .db line to an even byte count)
SEG_FONT:
        .db 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F

;=====================================================================
; RESET / SETUP
;=====================================================================
RESET_HANDLER:
        cli
        ldi  r16, HIGH(RAMEND)          ; stack pointer = top of SRAM
        out  SPH, r16
        ldi  r16, LOW(RAMEND)
        out  SPL, r16
        clr  r1                         ; r1 is always 0

        ; ---- clear all variables ----
        ldi  XL, LOW(v_ticks)
        ldi  XH, HIGH(v_ticks)
        ldi  r16, v_end - v_ticks
CLEAR_VARS:
        st   X+, r1
        dec  r16
        brne CLEAR_VARS

        ; ---- PORTB: six traffic outputs ----
        ldi  r16, 0x3F
        out  DDRB, r16
        ldi  r16, L_NS_GO
        out  PORTB, r16

        ; ---- PORTC: PC0..PC4 outputs, PC5 (LDR) input ----
        ldi  r16, 0x1F
        out  DDRC, r16
        ldi  r16, 0x0D                  ; ped red ON, seg a,b OFF, buzzer OFF
        out  PORTC, r16

        ; ---- PORTD: PD2..PD6 segments out, PD7 button in + pull-up ----
        ldi  r16, 0x7C
        out  DDRD, r16
        ldi  r16, 0xFC                  ; segments OFF, pull-up on PD7
        out  PORTD, r16

        ; ---- ADC: AVcc reference, right adjusted, channel ADC5 ----
        ldi  r16, (1<<REFS0) | 5
        sts  ADMUX, r16
        ldi  r16, (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
        sts  ADCSRA, r16                ; enable, 16 MHz / 128 = 125 kHz
        ldi  r16, (1<<ADC5D)
        sts  DIDR0, r16                 ; digital input off on ADC5

        ; ---- Timer1: CTC, 16 MHz / 64 / (249+1) = 1000 Hz ----
        sts  TCCR1A, r1
        sts  TCNT1H, r1                 ; 16-bit write: high byte first
        sts  TCNT1L, r1
        sts  OCR1AH, r1
        ldi  r16, 249
        sts  OCR1AL, r16
        ldi  r16, (1<<WGM12) | (1<<CS11) | (1<<CS10)
        sts  TCCR1B, r16
        ldi  r16, (1<<OCIE1A)
        sts  TIMSK1, r16

        ; ---- Pin-change interrupt on PD7 (PCINT23) ----
        ldi  r16, (1<<PCINT23)
        sts  PCMSK2, r16
        ldi  r16, (1<<PCIF2)            ; clear old flag (write 1)
        out  PCIFR, r16
        ldi  r16, (1<<PCIE2)
        sts  PCICR, r16

        ; ---- same start as the C++ setup() ----
        rcall SYNC_BUTTON
        ldi  r16, NS_GO
        rcall ENTER_STATE
        sei

;=====================================================================
; MAIN LOOP: run TICK_1MS once for every Timer1 tick
;=====================================================================
MAIN_LOOP:
        lds  r16, v_ticks
        tst  r16
        breq MAIN_LOOP
        cli                             ; v_ticks is shared with the ISR
        lds  r16, v_ticks
        dec  r16
        sts  v_ticks, r16
        sei
        rcall TICK_1MS
        rjmp MAIN_LOOP

;=====================================================================
; ONE MILLISECOND OF WORK 
;=====================================================================
TICK_1MS:
        rcall UPDATE_NIGHT_MODE
        lds  r16, v_nightMode
        tst  r16
        breq TICK_NORMAL
        sts  v_request, r1              ; night: ignore every request
        rcall SYNC_BUTTON
        rjmp TICK_OUT
TICK_NORMAL:
        rcall READ_BUTTON
        rcall UPDATE_TRAFFIC
TICK_OUT:
        rcall UPDATE_OUTPUTS
        ret

;=====================================================================
; ISR: Timer1 compare A - every 1 ms
;=====================================================================
ISR_T1_COMPA:
        push r16
        in   r16, SREG
        push r16

        lds  r16, v_ticks               ; ticks++ (stop at 255)
        cpi  r16, 255
        breq T1_DEBOUNCE
        inc  r16
        sts  v_ticks, r16
T1_DEBOUNCE:
        lds  r16, v_debounce            ; count debounce time down
        tst  r16
        breq T1_DONE
        dec  r16
        sts  v_debounce, r16
        brne T1_DONE
        ldi  r16, 1                     ; reached 0: pin quiet for 30 ms
        sts  v_btnCheck, r16
T1_DONE:
        pop  r16
        out  SREG, r16
        pop  r16
        reti

;=====================================================================
; ISR: pin change on PD7 - restart the debounce timer
;=====================================================================
ISR_PCINT2:
        push r16
        in   r16, SREG
        push r16
        ldi  r16, DEBOUNCE_TIME
        sts  v_debounce, r16
        pop  r16
        out  SREG, r16
        pop  r16
        reti

;=====================================================================
; ENTER_STATE: r16 = new state   (state = r16, stateMs = 0)
;=====================================================================
ENTER_STATE:
        sts  v_state, r16
        sts  v_stateMs, r1
        sts  v_stateMs+1, r1
        ret

;=====================================================================
; SYNC_BUTTON: accept current pin level, cancel pending debounce
;=====================================================================
SYNC_BUTTON:
        cli
        in   r16, PIND
        andi r16, (1<<PIND7)            ; 0x80 = released, 0 = pressed
        sts  v_stable, r16
        sts  v_debounce, r1
        sts  v_btnCheck, r1
        sei
        ret

;=====================================================================
; READ_BUTTON: called in NORMAL mode only
;=====================================================================
READ_BUTTON:
        cli                             ; take and clear the ISR flag
        lds  r16, v_btnCheck
        sts  v_btnCheck, r1
        sei
        tst  r16
        breq RB_DONE

        in   r16, PIND
        andi r16, (1<<PIND7)
        lds  r17, v_stable
        cp   r16, r17
        breq RB_DONE                    ; level did not change
        sts  v_stable, r16
        tst  r16
        brne RB_DONE                    ; it was a release

        lds  r17, v_state               ; pressed: ignore during crossing
        cpi  r17, PEDESTRIAN
        breq RB_DONE
        ldi  r17, 1
        sts  v_request, r17             ; one flag = one crossing
RB_DONE:
        ret

;=====================================================================
; READ_LDR: r25:r24 = 10-bit reading of ADC5
;=====================================================================
READ_LDR:
        lds  r16, ADCSRA
        ori  r16, (1<<ADSC)             ; start conversion
        sts  ADCSRA, r16
RL_WAIT:
        lds  r16, ADCSRA
        sbrc r16, ADSC                  ; wait until ADSC returns to 0
        rjmp RL_WAIT
        lds  r24, ADCL                  ; ADCL must be read first
        lds  r25, ADCH
        sts  v_ldr, r24
        sts  v_ldr+1, r25
        ret

;=====================================================================
; UPDATE_NIGHT_MODE
;=====================================================================
UPDATE_NIGHT_MODE:
        rcall READ_LDR
        ldi  r16, LOW(NIGHT_THRESHOLD+1)
        ldi  r17, HIGH(NIGHT_THRESHOLD+1)
        cp   r24, r16
        cpc  r25, r17
        brsh UN_LIGHT                   ; reading >= 401 -> light

        ; ---------- DARK ----------
        sts  v_recovMs, r1              ; any dark sample restarts recovery
        sts  v_recovMs+1, r1
        lds  r16, v_nightMode
        tst  r16
        brne UN_DONE                    ; already night
        ldi  r16, 1                     ; enter night mode
        sts  v_nightMode, r16
        sts  v_nightMs, r1
        sts  v_nightMs+1, r1
        sts  v_request, r1
        sts  v_showZero, r1
        ret

        ; ---------- LIGHT ----------
UN_LIGHT:
        lds  r16, v_nightMode
        tst  r16
        breq UN_DONE                    ; already normal
        lds  r24, v_recovMs             ; recovMs++
        lds  r25, v_recovMs+1
        adiw r24, 1
        sts  v_recovMs, r24
        sts  v_recovMs+1, r25
        ldi  r16, LOW(RECOVERY_TIME)
        ldi  r17, HIGH(RECOVERY_TIME)
        cp   r24, r16
        cpc  r25, r17
        brlo UN_DONE                    ; less than 5 s of light

        sts  v_nightMode, r1            ; leave night mode
        sts  v_recovMs, r1
        sts  v_recovMs+1, r1
        sts  v_request, r1
        rcall SYNC_BUTTON               ; ignore a button held at night
        ldi  r16, AFTER_EW              ; all-red, then NS_READY, NS_GO
        rcall ENTER_STATE
UN_DONE:
        ret

;=====================================================================
; UPDATE_TRAFFIC: state machine for NORMAL mode
;=====================================================================
UPDATE_TRAFFIC:
        lds  r24, v_stateMs             ; stateMs++
        lds  r25, v_stateMs+1
        adiw r24, 1
        sts  v_stateMs, r24
        sts  v_stateMs+1, r25

        lds  r16, v_state               ; Z = STATE_TIME + 2*state
        ldi  ZL, LOW(STATE_TIME*2)
        ldi  ZH, HIGH(STATE_TIME*2)
        mov  r17, r16
        lsl  r17
        add  ZL, r17
        adc  ZH, r1
        lpm  r22, Z+                    ; r23:r22 = duration
        lpm  r23, Z
        cp   r24, r22
        cpc  r25, r23
        brlo UT_DONE                    ; time not finished

        ; ---- time is up: choose next state ----
        cpi  r16, PEDESTRIAN
        breq UT_PED_END
        cpi  r16, AFTER_NS
        breq UT_CLEARANCE
        cpi  r16, AFTER_EW
        breq UT_CLEARANCE
        cpi  r16, NS_READY
        breq UT_READY_END
        cpi  r16, EW_READY
        breq UT_READY_END
        rjmp UT_NEXT

UT_READY_END:
        sts  v_showZero, r1             ; "0" disappears when green starts
        rjmp UT_NEXT

UT_CLEARANCE:
        lds  r17, v_request
        tst  r17
        breq UT_NEXT                    ; no request: continue normally
        sts  v_request, r1              ; request served
        inc  r16                        ; resume = next state
        andi r16, 0x07                  ; (3 -> 4, 7 -> 0)
        sts  v_resume, r16
        ldi  r16, PEDESTRIAN
        rjmp ENTER_STATE                ; tail call

UT_PED_END:
        sts  v_request, r1
        rcall SYNC_BUTTON
        ldi  r16, 1
        sts  v_showZero, r16
        lds  r16, v_resume
        rjmp ENTER_STATE

UT_NEXT:                                ; next = (state + 1) mod 8
        inc  r16
        andi r16, 0x07
        rjmp ENTER_STATE
UT_DONE:
        ret

;=====================================================================
; UPDATE_OUTPUTS: PORTB, PORTC, PORTD
;   r20 = segment pattern (1 = lit), r21 = PORTC bits PC0/PC1/PC4
;=====================================================================
UPDATE_OUTPUTS:
        lds  r16, v_nightMode
        tst  r16
        breq UO_NORMAL

        ; ---------- NIGHT: flash yellows, all else off ----------
        lds  r24, v_nightMs
        lds  r25, v_nightMs+1
        ldi  r16, L_NIGHT
        ldi  r17, HIGH(FLASH_INTERVAL)
        cpi  r24, LOW(FLASH_INTERVAL)
        cpc  r25, r17
        brlo UO_FLASH                   ; first 500 ms: ON
        clr  r16                        ; second 500 ms: OFF
UO_FLASH:
        out  PORTB, r16
        adiw r24, 1                     ; nightMs = (nightMs+1) mod 1000
        ldi  r17, HIGH(2*FLASH_INTERVAL)
        cpi  r24, LOW(2*FLASH_INTERVAL)
        cpc  r25, r17
        brlo UO_STORE
        clr  r24
        clr  r25
UO_STORE:
        sts  v_nightMs, r24
        sts  v_nightMs+1, r25
        clr  r20                        ; display blank
        clr  r21                        ; ped LEDs off, buzzer off
        rjmp WRITE_DISPLAY

        ; ---------- NORMAL / PEDESTRIAN ----------
UO_NORMAL:
        lds  r16, v_state
        ldi  ZL, LOW(STATE_LIGHTS*2)    ; PORTB = STATE_LIGHTS[state]
        ldi  ZH, HIGH(STATE_LIGHTS*2)
        add  ZL, r16
        adc  ZH, r1
        lpm  r17, Z
        out  PORTB, r17

        cpi  r16, PEDESTRIAN
        brne UO_NOT_CROSSING

        ; crossing: ped green + buzzer, digit = 5 - (stateMs / 1000)
        ldi  r21, (1<<PC1) | (1<<PC4)
        ldi  r18, 5
        lds  r24, v_stateMs
        lds  r25, v_stateMs+1
UO_DIV:
        ldi  r17, HIGH(ONE_SECOND)
        cpi  r24, LOW(ONE_SECOND)
        cpc  r25, r17
        brlo UO_DIGIT
        subi r24, LOW(ONE_SECOND)
        sbci r25, HIGH(ONE_SECOND)
        dec  r18
        rjmp UO_DIV
UO_DIGIT:
        ldi  ZL, LOW(SEG_FONT*2)        ; r20 = SEG_FONT[digit]
        ldi  ZH, HIGH(SEG_FONT*2)
        add  ZL, r18
        adc  ZH, r1
        lpm  r20, Z
        rjmp WRITE_DISPLAY

UO_NOT_CROSSING:
        ldi  r21, (1<<PC0)              ; ped red ON
        clr  r20                        ; blank unless "0" is needed
        lds  r17, v_showZero
        tst  r17
        breq WRITE_DISPLAY
        cpi  r16, NS_READY
        breq UO_ZERO
        cpi  r16, EW_READY
        brne WRITE_DISPLAY
UO_ZERO:
        ldi  ZL, LOW(SEG_FONT*2)
        ldi  ZH, HIGH(SEG_FONT*2)
        lpm  r20, Z                     ; pattern for "0"

        ; ---------- send r20 / r21 to the pins ----------
WRITE_DISPLAY:
        mov  r19, r20
        com  r19                        ; common anode: 1 = segment OFF
        in   r16, PORTD                 ; c..g = bits 2..6 = PD2..PD6
        andi r16, 0x83                  ; keep PD0, PD1, PD7 (pull-up)
        mov  r17, r19
        andi r17, 0x7C
        or   r16, r17
        out  PORTD, r16
        sbrc r19, 1                     ; segment b -> PC2
        ori  r21, (1<<PC2)
        sbrc r19, 0                     ; segment a -> PC3
        ori  r21, (1<<PC3)
        out  PORTC, r21                 ; PC5 stays 0 (LDR input)
        ret
