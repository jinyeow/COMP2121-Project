; Miscellaneous Macros

;#######################
;#    TIMER MACRO      #
;#######################
; The macro clears a word (2 bytes) in memory
; the parameter @0 is the memory address for that word
.macro clear
    ldi YL, low(@0)        ; load the memory address to Y
    ldi YH, high(@0)
    clr temp1
    st Y+, temp1           ; clear the two bytes at @0 in SRAM
    st Y, temp1
.endmacro

;#######################
;#    PRINT MACROS     #
;#######################
.macro print_digit
    mov temp2, @0
    subi temp2, -'0'
    do_lcd_data_reg temp2
.endmacro

;#######################
;#   STATUS MACROS     #
;#######################
; Status Bits: [ 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 ]
;   0 => EMERGENCY
;   1 => DEBOUNCE FLAG
;   2 => LIFT MOVING
;   3 => DOOR MOVING
;   4 => DOOR OPEN/CLOSE

; Constants defined to set/clr bits in the Lift "STATUS" register (r22)
.equ EMERGENCY_ON  = 0b00000001
.equ EMERGENCY_OFF = 0b11111110
.equ DEBOUNCE_ON   = 0b00000010
.equ DEBOUNCE_OFF  = 0b11111101

; For the following macros, @0 should be one of the above defined constants
.macro set_status_bit_on
    ori status, @0
.endmacro

.macro set_status_bit_off
    andi status, @0
.endmacro

.macro compare_status_bit
    mov temp2, status
    andi temp2, @0
    cpi temp2, @0
.endmacro
