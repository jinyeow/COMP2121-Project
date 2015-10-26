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
.macro set_emergency
    sbr status, 1
.endmacro

.macro clr_emergency
    sbr status, 0
.endmacro
