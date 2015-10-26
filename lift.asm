.include "m2560def.inc"

.equ ONE_SECOND     = 7812
.equ DEBOUNCE_LIMIT = 1560

; Door open is two bars at each end of the 8 bits of the LED to signify the open
; doors of a lift
; Door closed is all LEDs lit up
; Door moving will be a split pattern/shifting split pattern
.equ DOOR_OPEN      = 0x81
.equ DOOR_CLOSED    = 0xFF
.equ DOOR_MOVING    = 0x55

; use a register as a "status" register ??

;#######################
;#   KEYPAD DEFS       #
;#######################
.def row   = r16           ; current row number
.def col   = r17           ; current column number
.def rmask = r18           ; mask for current row during scan
.def cmask = r19           ; mask for current column during scan
.def temp1 = r20
.def temp2 = r21

; LIFT "STATUS" REGISTER bit 0 is emergency bit, set if * pressed, else clear
.def status = r22

.equ PORTLDIR    = 0xF0    ; PD7-4: output, PD3-0:input
.equ INITCOLMASK = 0xEF    ; scan from the rightmost column
.equ INITROWMASK = 0x01    ; scan from the top row
.equ ROWMASK     = 0x0F    ; for obtaining input form Port D

;#######################
;#     MODULES         #
;#######################
.include "modules/lcd.asm"
.include "modules/macros.asm"

;#######################
;#      MACROS         #
;#######################
.macro print_current_floor
    lds r2, FloorNumber
    print_digit r2
.endmacro

; TODO: check this works
.macro update_floor_number
    lds r2, FloorNumber
    ldi r3, @0
    add r2, r3
    sts FloorNumber, r2
.endmacro

.dseg
TempCounter:
    .byte 2
SecondCounter:
    .byte 2
DebounceCounter:
    .byte 2
MoveTimer:                 ; Lifts take 2 seconds to traverse floors
    .byte 2
FloorNumber:               ; Current Floor Number
    .byte 1
FloorQueue:                ; Multiple requests can be queued up
    .byte 4 ; Maximum of 10 floors in the queue - may have to increase size
            ; e.g. .byte 5, 4 bits to represent a floor number ??
DoorStatus:
    .byte 1 ; Status can be: closing, closed, opening, open

.cseg
.org 0x0000
    jmp RESET
.org OVF0addr
    jmp Timer0OVF

    jmp DEFAULT
DEFAULT: reti

RESET:
    ldi temp1, low(RAMEND)   ; initialize the stack
    out SPL, temp1
    ldi temp1, high(RAMEND)
    out SPH, temp1

    ; Keyboard
    ldi temp1, PORTLDIR      ; PA7:4/PA3:0, out/in
    sts DDRL, temp1

    ; LEDs
    ser temp1                ; PORTC is output
    out DDRC, temp1
    out PORTC, temp1

    ser temp1
    out DDRF, temp1
    out DDRA, temp1
    clr temp1
    out PORTF, temp1
    out PORTA, temp1

    ; Initial LCD Message
    ; ldi temp1, DOOR_OPEN
    ldi temp1, DOOR_CLOSED
    out PORTC, temp1

    ; Clear Counters
    clear TempCounter                 ; Initialize the temporary counter to 0
    clear SecondCounter               ; Initialize the second counter to 0
    clear FloorNumber
    clear DebounceCounter

    ; Clear Lift "STATUS" register
    set_status_bit_off CLEAR_FLAGS

    ; Timer Settings and Pre-Scaling
    ldi temp1, 0b00000000
    out TCCR0A, temp1
    ldi temp1, 0b00000010
    out TCCR0B, temp1                 ; Prescaling value = 8
    ldi temp1, 1<<TOIE0               ; = 128 microseconds
    sts TIMSK0, temp1                 ; T/C0 interrupt enable

    ; Initialise Lift on Floor 0
    lcd_clear_prompt
    lcd_pre_prompt
    do_lcd_data '0'
    ; lcd_emergency_message

    sei                               ; Enable global interrupt

    rjmp main

Timer0OVF:
    in temp1, SREG
    push temp1                      ; Prologue starts
    push YH                         ; Save all conflict registers in the prologue
    push YL
    push r27
    push r26
    push r25
    push r24                        ; Prologue ends
                                    ; Load the vlaue of the temporary counter
    CheckDebounceSet:
        compare_status_bit DEBOUNCE_ON
        breq NewDebounceCount
        rjmp NewSecond

    NewDebounceCount:
        lds r26, DebounceCounter
        lds r27, DebounceCounter+1
        adiw r27:r26, 1

        ldi temp1, high(DEBOUNCE_LIMIT)
        cpi r26, low(DEBOUNCE_LIMIT)
        cpc r27, temp1
        brne KeepDebouncing

        set_status_bit_off DEBOUNCE_OFF
        clear DebounceCounter
        clr r26
        clr r27

    NewSecond:
        lds r24, TempCounter
        lds r25, TempCounter+1
        adiw r25:r24, 1

        ldi temp1, high(ONE_SECOND)
        cpi r24, low(ONE_SECOND)
        cpc r25, temp1
        brne NotSecond

    rjmp EndIF

NotSecond:
    sts TempCounter, r24
    sts TempCounter+1, r25
    rjmp EndIF

KeepDebouncing:
    sts DebounceCounter, r26
    sts DebounceCounter+1, r27
    rjmp EndIF

EndIF:
    pop r24                         ; Epilogue starts
    pop r25                         ; Restore all conflict registers from the stack
    pop r26
    pop r27
    pop YL
    pop YH
    pop temp1
    out SREG, temp1
    reti                            ; Return from the interrupt

main:
    ldi cmask, INITCOLMASK   ; initial column mask
    clr col                  ; initial column

colloop:
    cpi col, 4
    breq main                ; If all keys are scanned, repeat.
    sts PORTL, cmask         ; otherwise scan a column

    ldi temp1, 0xFF          ; Slow down the scan operation
delay:
    dec temp1
    brne delay

    lds temp1, PINL          ; Read PORTL
    andi temp1, ROWMASK      ; Get the keypad output value
    cpi temp1, 0xF           ; Check if any row is low
    breq nextcol
                           ; If yes, find which row is low
    ldi rmask, INITROWMASK   ; Initialize for row check
    clr row

rowloop:
    cpi row, 4
    breq nextcol             ; the row scan is over
    mov temp2, temp1
    and temp2, rmask         ; check un-masked bit
    breq convert             ; if bit is clear, the key is pressed
    inc row
    lsl rmask
    jmp rowloop

nextcol:                   ; if row scan is over
    lsl cmask
    inc col                  ; increase the column value
    jmp colloop              ; go to the next column

convert:
    cpi col, 3               ; if the pressed key is in col 3
    breq main                ; we have  a letter - but we ignore non-digits for this part
                           ; if key is not in col 3 and
    cpi row, 3               ; if the key is in row3,
    breq symbols             ; we have a symbol or 0

    mov temp1, row           ; otherwise we have a number 1-9
    lsl temp1
    add temp1, row
    add temp1, col           ; temp1 = row*3 + col
    subi temp1, -1           ; Add 1 to the value since temp1 should start at 1 not 0
    jmp convert_end

symbols:
    cpi col, 0
    breq star
    cpi col, 1               ; Check if we have 0
    breq zero
    rjmp main                ; else ignore

zero:
    ldi temp1, 0             ; Set to zero

star:
    ; TODO: jmp to Emergency Function, stop all activity and goto Floor 0
    ; Emergency Function: Do Open/Close process at Floor 0. Strobe LED should
    ; blink several times.
    compare_status_bit DEBOUNCE_ON
    breq jump_main1
    set_status_bit_on DEBOUNCE_ON

    compare_status_bit EMERGENCY_ON ; Lift should resume only when * is pressed again.
    breq emergency_end
    rjmp emergency_start

jump_main1:
    jmp main

emergency_end:
    lcd_clear_prompt
    lcd_pre_prompt
    do_lcd_data '0' ; since we should be on Floor 0 - maybe change this to the 'CURRENT' floor
    set_status_bit_off EMERGENCY_OFF
    rjmp main

emergency_start:
    lcd_clear_prompt
    lcd_emergency_message
    set_status_bit_on EMERGENCY_ON
    rjmp main

convert_end:
    compare_status_bit EMERGENCY_ON
    breq jump_main2
    compare_status_bit DEBOUNCE_ON
    breq jump_main2
    set_status_bit_on DEBOUNCE_ON
    ; TODO:
    ; actually shouldn't write to the screen
    ; should push the floor number to the FloorQueue
    ; should check if we are already in motion
    ; if True, push floor number to FloorQueue
    ; else start moving lift
    push temp1
    lcd_clear_prompt
    lcd_pre_prompt
    pop temp1
    print_digit temp1
    rjmp main

jump_main2:
    jmp main

end: rjmp end

;#######################
;#    LCD FUNCTIONS    #
;#######################
; Send a command to the LCD (r16)
lcd_command:
  out PORTF, temp1
  rcall sleep_1ms
  lcd_set LCD_E
  rcall sleep_1ms
  lcd_clr LCD_E
  rcall sleep_1ms
  ret

lcd_data:
  out PORTF, temp1
  lcd_set LCD_RS
  rcall sleep_1ms
  lcd_set LCD_E
  rcall sleep_1ms
  lcd_clr LCD_E
  rcall sleep_1ms
  lcd_clr LCD_RS
  ret

lcd_wait:
  push temp1
  clr temp1
  out DDRF, temp1
  out PORTF, temp1
  lcd_set LCD_RW
lcd_wait_loop:
  rcall sleep_1ms
  lcd_set LCD_E
  rcall sleep_1ms
  in temp1, PINF
  lcd_clr LCD_E
  sbrc temp1, 7
  rjmp lcd_wait_loop
  lcd_clr LCD_RW
  ser temp1
  out DDRF, temp1
  pop temp1
  ret

sleep_1ms:
  push r24
  push r25
  ldi r25, high(DELAY_1MS)
  ldi r24, low(DELAY_1MS)
delayloop_1ms:
  sbiw r25:r24, 1
  brne delayloop_1ms
  pop r25
  pop r24
  ret

sleep_5ms:
  rcall sleep_1ms
  rcall sleep_1ms
  rcall sleep_1ms
  rcall sleep_1ms
  rcall sleep_1ms
  ret
