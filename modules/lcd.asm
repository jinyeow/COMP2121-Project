; LCD Related Macros, Definitions, Functions, etc

.equ LCD_RS = 7
.equ LCD_E  = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
; 4 cycles per iteration - setup/call-return overhead

.macro do_lcd_command
  ldi temp1, @0
  rcall lcd_command
  rcall lcd_wait
.endmacro

.macro do_lcd_data
  ldi temp1, @0
  rcall lcd_data
  rcall lcd_wait
.endmacro

.macro lcd_set
  sbi PORTA, @0
.endmacro

.macro lcd_clr
  cbi PORTA, @0
.endmacro

.macro do_lcd_data_reg
    mov temp1, @0
    rcall lcd_data
    rcall lcd_wait
.endmacro

.macro lcd_clear_prompt
    do_lcd_command 0b00111000 ; 2x5x7
    rcall sleep_5ms
    do_lcd_command 0b00111000 ; 2x5x7
    do_lcd_command 0b00111000 ; 2x5x7
    do_lcd_command 0b00001000 ; display off?
    do_lcd_command 0b00000001 ; clear display
    do_lcd_command 0b00000110 ; increment, no display shift
    do_lcd_command 0b00001110 ; cursor on, bar, no blink
.endmacro

.macro lcd_pre_prompt
    do_lcd_data 'F'
    do_lcd_data 'L'
    do_lcd_data 'O'
    do_lcd_data 'O'
    do_lcd_data 'R'
    do_lcd_data ':'
    do_lcd_data ' '
.endmacro

.macro lcd_new_line
    do_lcd_command 0b11000000 ; move to next line
.endmacro

.macro lcd_emergency_message
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data 'E'
    do_lcd_data 'm'
    do_lcd_data 'e'
    do_lcd_data 'r'
    do_lcd_data 'g'
    do_lcd_data 'e'
    do_lcd_data 'n'
    do_lcd_data 'c'
    do_lcd_data 'y'
    do_lcd_command 0b11000000 ; move to next line
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data ' '
    do_lcd_data 'C'
    do_lcd_data 'a'
    do_lcd_data 'l'
    do_lcd_data 'l'
    do_lcd_data ' '
    do_lcd_data '0'
    do_lcd_data '0'
    do_lcd_data '0'
.endmacro

;#######################
;#    PRINT MACROS     #
;#######################
.macro print_digit
    mov temp2, @0
    subi temp2, -'0'
    do_lcd_data_reg temp2
.endmacro
