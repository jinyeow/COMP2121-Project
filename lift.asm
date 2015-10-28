.include "m2560def.inc"

.equ ONE_SECOND     = 7812
.equ TWO_SECONDS    = 7812 * 2
.equ THREE_SECONDS  = 7812 * 3
.equ PATTERN_SHIFT_INTERVAL = 7812 * 2 / 8
.equ DEBOUNCE_LIMIT = 3120

.equ MAX_FLOOR = 9
.equ MIN_FLOOR = 0

; Door open is two bars at each end of the 8 bits of the LED to signify the open
; doors of a lift
; Door closed is all LEDs lit up
; Door moving will be a split pattern/shifting split pattern
.equ DOOR_OPEN   = 0x81
.equ DOOR_CLOSED = 0xFF

.equ LIFT_MOVING_UP = 0x1C1C
.equ LIFT_MOVING_DOWN = 0x3838

.equ MOTOR_OFF   = 0x00
.equ MOTOR_ON    = 0xFF

; LIFT "STATUS" REGISTER
; bit 0 is emergency bit, set if * pressed, else clear
.def status           = r22

;#######################
;#   KEYPAD DEFS       #
;#######################
.def row   = r16           ; current row number
.def col   = r17           ; current column number
.def rmask = r18           ; mask for current row during scan
.def cmask = r19           ; mask for current column during scan
.def temp1 = r20
.def temp2 = r21
.def temp3 = r23

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

.macro update_floor_number
    lds r2, FloorNumber
    ldi r30, @0
    add r2, r30
    sts FloorNumber, r2
.endmacro

;#######################
;#       DSEG          #
;#######################
.dseg
TempCounter:
    .byte 2
SecondCounter:
    .byte 2
DebounceCounter:
    .byte 2
FloorNumber:               ; Current Floor Number
    .byte 1
FloorBits:
    .byte 2                ; Represent Lift's position as a bit to be shifted
                           ; left/right to denote moving up/down
                           ; by shifting left/right respectively
                           ; which can then be compared with FloorQueue to
                           ; determine if we need to open/close the door
FloorQueue:                ; Multiple requests can be queued up
    .byte 2 ; Each bit represents a floor (obviously not all bits used
CurrentPattern:
    .byte 2

;#######################
;#       CSEG          #
;#######################
.cseg
.org 0x0000
    jmp RESET
    jmp DEFAULT                     ; no handling for IRQ0
    jmp DEFAULT                     ; no handling for IRQ1
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
    out PORTC, temp1         ; out PORTC all LEDs lit up
    out DDRG, temp1
    out PORTG, temp1

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
    out PORTG, temp1

    ; Clear Counters
    clear TempCounter                 ; Initialize the temporary counter to 0
    clear SecondCounter               ; Initialize the second counter to 0
    clear FloorNumber
    clear FloorQueue
    clear DebounceCounter
    clear FloorBits

    ; Set FloorBits to Floor 0 initially.
    ldi r24, FLOOR_0
    sts FloorBits, r24
    sts CurrentPattern, r24

    ; Set CurrentPattern
    lds r24, CurrentPattern
    lds r25, CurrentPattern+1
    ldi  r24, low(LIFT_MOVING_UP)
    sts CurrentPattern, r24
    ldi  r24, high(LIFT_MOVING_UP)
    sts CurrentPattern+1, r24
    clr r24

    ; Clear Lift "STATUS" register
    set_status_bit_off CLEAR_FLAGS

    ; Timer Settings and Pre-Scaling
    ldi temp1, 0b00000000
    out TCCR0A, temp1
    ldi temp1, 0b00000010
    out TCCR0B, temp1                 ; Prescaling value = 8
    ldi temp1, 1<<TOIE0               ; = 128 microseconds
    sts TIMSK0, temp1                 ; T/C0 interrupt enable

    ; Motor PE2
    ldi temp1, 0xFF ; set to input
    out DDRE, temp1

    ldi temp1, MOTOR_OFF ; motor starts off
    sts OCR3BL, temp1
    clr temp1
    sts OCR3BH, temp1

    ldi temp1, (1<<CS30)
    sts TCCR3B, temp1
    ldi temp1, (1<<WGM30)|(1<<COM3B1)
    sts TCCR3A, temp1

    ; Initialise Lift on Floor 0
    lcd_clear_prompt
    lcd_pre_prompt
    do_lcd_data '0'

    sei                               ; Enable global interrupt

    rjmp main

;#######################
;# INTERRUPT ROUTINES  #
;#######################
Timer0OVF:
    in temp1, SREG
    push temp1                      ; Prologue starts
    push temp2
    push temp3
    push r29
    push r28
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
        breq StopDebounce
        jmp KeepDebouncing

    StopDebounce:
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
        breq OneSecond
        rjmp NotSecond

    OneSecond:
        clear TempCounter

        ; Update Second Counter
        lds r28, SecondCounter
        lds r29, SecondCounter+1
        adiw r29:r28, 1

    CheckMoving:
        clr r24
        clr r25
        lds r24, FloorQueue
        lds r25, FloorQueue+1
        cpi r24, 0x00
        ldi temp1, 0x00
        cpc r25, temp1
        breq NoCalls

        compare_status_bit MOVING_ON
        brne jump_MoveLift

        cpi r28, 2 ; Lift travels for 2 seconds - double check TIMER0 settings.
        sts SecondCounter, r28
        sts SecondCounter+1, r29
        brge ReachedFloor
        lds r26, CurrentPattern
        lds r27, CurrentPattern+1
        out PORTC, r26
        out PORTG, r27

    ; Count Time for Door open/close + Lift Travel
    NoCalls:
        rjmp EndIF

jump_MoveLift:
    jmp MoveLift

NotSecond:
    sts TempCounter, r24
    sts TempCounter+1, r25
    rjmp EndIF

KeepDebouncing:
    sts DebounceCounter, r26
    sts DebounceCounter+1, r27
    rjmp EndIF

ReachedFloor:
    push temp2
    push temp1
    push r27
    push r26
    push r25
    push r24

    ; Depending on the DIRECTION either add or sub 1
    compare_status_bit DIR_UP
    brne GoingDown
    update_floor_number 1 ; update FloorNumber
    rjmp AfterUpdateFloorNumber

    GoingDown:
        update_floor_number -1

    AfterUpdateFloorNumber:
    clr r24
    clr r25
    ldi r24, 0b00000001
    lds temp1, FloorNumber
    get_floor_in_bits r24, r25 ; FloorBits (need to update FloorBits in SRAM)
    sts FloorBits, r24
    sts FloorBits+1, r25

    lcd_clear_prompt      ; print FloorNumber to LCD
    lcd_pre_prompt
    print_current_floor

    clear SecondCounter
    set_status_bit_off MOVING_OFF

    ; Turn off motor
    ldi temp1, MOTOR_OFF
    sts OCR3BL, temp1
    clr temp1
    sts OCR3BH, temp1

    ; If floor is in queue then
    ; TODO: Open/Close doors

    ; drop floor from queue
    com r24
    com r25
    lds r26, FloorQueue
    lds r27, FloorQueue+1
    and r26, r24
    and r27, r25
    sts FloorQueue, r26
    sts FloorQueue+1, r27

    ; print_queue ; for debugging

    pop r24
    pop r25
    pop r26
    pop r27
    pop temp1
    pop temp2
    rjmp EndIF

EndIF:
    pop r24                         ; Epilogue starts
    pop r25                         ; Restore all conflict registers from the stack
    pop r26
    pop r27
    pop r28
    pop r29
    pop temp3
    pop temp2
    pop temp1
    out SREG, temp1
    reti                            ; Return from the interrupt

MoveLift: ; Activate lift
    push temp3
    push temp2
    push r27
    push r26
    push r25
    push r24
    push r31
    push r30

    clr r30 ; LOWER FLOORS BOOLEAN
    clr r31 ; HIGHER FLOORS BOOLEAN
    clear SecondCounter ; reset the SecondCounter
    clear TempCounter
    set_status_bit_on MOVING_ON ; the lift is moving

    ; scan FloorQueue FIXME
    ; determine if going UP or DOWN
    lds r26, FloorBits
    lds r27, FloorBits+1

    out PORTC, r26
    out PORTG, r27

    lds r24, FloorQueue
    lds r25, FloorQueue+1

    cp r26, r24   ; If FloorQueue < FloorBits then that means only Lower Floors
                  ; were called. Therefore DIR DOWN
    cpc r27, r25
    brge SetDirDown
    rjmp SetDirUp

    ; ; scan by taking current floor bits do max floor - curr floor and shift left
    ; ; that many times.
    ; ldi temp2, MAX_FLOOR
    ; lds temp3, FloorNumber
    ; sub temp2, temp3 ; MAX_FLOOR - current floor number
    ;                  ; temp2 holds counter for left shifting for scannning
    ;                  ; currently off-by-one error MAX - INITIAL FLOOR = 9
    ;                  ; but since we're on Floor 0 we only need to shift 8 times
    ;                  ; Fine for now. Add a subi temp2, 1 later.
    ; ScanHigher:
    ;     cpi temp2, 1
    ;     brlt ScanLowerPre
    ;     dec temp2
    ;     lsl r26          ; left shift r27:r26 (FloorBits) to check if any floors
    ;     rol r27          ; called that are HIGHER than the current floor

    ;     push r26
    ;     push r27
    ;     and r26, r24
    ;     and r27, r25
    ;     cpi r26, 0
    ;     brne SetDirUpTrue
    ;     cpi r27, 0       ; if r27:r26 == 0, then floor is not set/called
    ;     brne SetDirUpTrue
    ;     pop r27
    ;     pop r26
    ;     rjmp ScanHigher

    ; ; set a bool for higher floors if you find one
    ; SetDirUpTrue:
    ;     ser r31

    ; ; and vice versa for lower floors
    ; ScanLowerPre:
    ;     lds r26, FloorBits ; Reset r27:r26 to current floor's bits
    ;     lds r27, FloorBits+1
    ;     lds temp2, FloorNumber
    ;     ldi temp3, MIN_FLOOR
    ;     sub temp2, temp3
    ;     ScanLower:
    ;         cpi temp2, 0
    ;         breq SetDirUp
    ;         dec temp2
    ;         lsr r26
    ;         ror r27

    ;         push r26
    ;         push r27
    ;         and r26, r24
    ;         and r27, r25
    ;         cpi r26, 0
    ;         brne SetDirDownTrue
    ;         cpi r27, 0       ; if r27:r26 == 0, then floor is not set/called
    ;         brne SetDirDownTrue
    ;         pop r27
    ;         pop r26
    ;         rjmp ScanLower

    ; SetDirDownTrue:
    ;     ser r30

    ; ; if found both then continue in curr dir
    ; ; else change as appropriate
    ; cpi r31, 0xFF ; No higher floors if not equal
    ; brne SetDirDown
    ; cpi r30, 0xFF ; No lower floors if not equal
    ; brne SetDirUp
    ; rjmp ContinueInCurrentDirection

    SetDirDown:
        set_status_bit_off DIR_DOWN
        lsr r26
        ror r27
        rjmp ContinueInCurrentDirection

    SetDirUp:
        lsl r26
        rol r27
        set_status_bit_on DIR_UP

    ContinueInCurrentDirection:
        ; Spin Motor
        ldi temp2, MOTOR_ON
        sts OCR3BL, temp2
        ; clr temp2
        ; sts OCR3BH, temp2

    ; out shifting pattern to LEDs

    sts CurrentPattern, r26
    sts CurrentPattern+1, r27

    pop r30
    pop r31
    pop r24
    pop r25
    pop r26
    pop r27
    pop temp2
    pop temp3
    rjmp EndIF

;#######################
;#      MAIN CODE      #
;#######################
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
    jmp convert_end

star:
    ; TODO: jmp to Emergency Function, stop all activity and goto Floor 0
    ; Emergency Function: Do Open/Close process at Floor 0.
    ; Strobe LED should blink several times.
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
    print_current_floor
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

    ; NOTE: if floor called is current floor, disregard...or open the doors
    lds r31, FloorNumber
    cp r31, temp1
    breq jump_main2 ; should possibly go through door sequence

    ; Add Floor to FloorQueue
    rcall call_floor
    ; rjmp main

jump_main2:
    jmp main

call_floor: ; Adds the called floor to the FloorQueue
    push temp2
    push temp1
    push r25
    push r24

    clr r24
    clr r25
    ldi r24, 0b00000001 ; use temp2 as a temporary floor rep

    get_floor_in_bits r24, r25
    mov temp1, r24
    mov temp2, r25
    update_floor_queue temp1, temp2

    pop r24
    pop r25
    pop temp1
    pop temp2
    ret

end: rjmp end

;#######################
;#    LCD FUNCTIONS    #
;#######################
; Send a command to the LCD (r16)
lcd_command:
  push temp1
  out PORTF, temp1
  rcall sleep_1ms
  lcd_set LCD_E
  rcall sleep_1ms
  lcd_clr LCD_E
  rcall sleep_1ms
  pop temp1
  ret

lcd_data:
    push temp1
    out PORTF, temp1
    lcd_set LCD_RS
    rcall sleep_1ms
    lcd_set LCD_E
    rcall sleep_1ms
    lcd_clr LCD_E
    rcall sleep_1ms
    lcd_clr LCD_RS
    pop temp1
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
