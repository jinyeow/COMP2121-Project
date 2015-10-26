.include "m2560def.inc"

.equ SECOND = 7812
.equ DEBOUNCE_LIMIT = 1560
; .equ UP_PATTERN =
; .equ DOWN_PATTERN =
; .equ OPEN_PATTERN =
; .equ CLOSE_PATTERN =

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

.equ PORTLDIR    = 0xF0    ; PD7-4: output, PD3-0:input
.equ INITCOLMASK = 0xEF    ; scan from the rightmost column
.equ INITROWMASK = 0x01    ; scan from the top row
.equ ROWMASK     = 0x0F    ; for obtaining input form Port D

;#######################
;#     LCD DEFS        #
;#######################
.include "lcd.asm"

.dseg
TempCounter:
    .byte 2
SecondCounter:
    .byte 2
DebouceCounter:
    .byte 2
MoveTimer:                 ; Lifts take 2 seconds to traverse floors
    .byte 2
FloorNumber:               ; Current Floor Number
    .byte 1
FloorQueue:                ; Multiple requests can be queued up
    .byte 4
DoorStatus:
    .byte 1 ; Status can be: closing, closed, opening, open

.cseg
.org 0x0000
    jmp RESET

RESET:
    ldi temp1, low(RAMEND)   ; initialize the stack
    out SPL, temp1
    ldi temp1, high(RAMEND)
    out SPH, temp1

    ldi temp1, PORTLDIR      ; PA7:4/PA3:0, out/in
    sts DDRL, temp1
    ser temp1                ; PORTC is output
    out DDRC, temp1
    out PORTC, temp1

    ser temp1
    out DDRF, temp1
    out DDRA, temp1
    clr temp1
    out PORTF, temp1
    out PORTA, temp1

    ; Initialise Lift on Floor 0
    lcd_clear_prompt
    lcd_pre_prompt
    do_lcd_data '0'
    ; lcd_emergency_message

main:

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
