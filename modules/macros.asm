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

; The macro clears a byte (1 byte) in a memory
; the parameter @0 is the memory address for that byte
.macro clear_byte
    ldi YL, low(@0)    ; load the memory address to Y
    clr temp1
    st Y, temp1        ; clear the byte at @0 in SRAM
.endmacro

;#######################
;#   STATUS MACROS     #
;#######################
; Status Bits: LSB [ 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 ] MSB
;   0 => EMERGENCY
;   1 => DEBOUNCE FLAG
;   2 => LIFT MOVING
;   3 => LIFT DIRECTION
;   4 => DOOR MOVING
;   5 => DOOR OPEN/CLOSE

; Constants defined to set/clr bits in the Lift "STATUS" register (r22)
.equ CLEAR_FLAGS    = 0x00
.equ EMERGENCY_ON   = 0b00000001
.equ EMERGENCY_OFF  = 0b11111110 ; can use com instruction with a temp reg instead?
.equ DEBOUNCE_ON    = 0b00000010
.equ DEBOUNCE_OFF   = 0b11111101
.equ MOVING_ON      = 0b00000100
.equ MOVING_OFF     = 0b11111011
.equ DIR_UP         = 0b00001000
.equ DIR_DOWN       = 0b11110111
.equ DOOR_MOV       = 0b00010000
.equ DOOR_NOT_MOV   = 0b11101111
.equ DOOR_IS_OPEN   = 0b00100000
.equ DOOR_IS_CLOSED = 0b11011111

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

;#######################
;#    FLOOR MACROS     #
;#######################
; low(FloorQueue)
.equ FLOOR_0 = 0b00000001
.equ FLOOR_1 = 0b00000010
.equ FLOOR_2 = 0b00000100
.equ FLOOR_3 = 0b00001000
.equ FLOOR_4 = 0b00010000
.equ FLOOR_5 = 0b00100000
.equ FLOOR_6 = 0b01000000
.equ FLOOR_7 = 0b10000000

; high(FloorQueue)
.equ FLOOR_8 = 0b00000001
.equ FLOOR_9 = 0b00000010

.macro get_floor_in_bits
    push temp1

    NotFloor:
        cpi temp1, 1
        brlt FloorFound
        dec temp1
        lsl @0
        rol @1
        rjmp NotFloor
    FloorFound:

    pop temp1
.endmacro

.macro update_floor_queue
    push temp2
    push temp1
    push r25
    push r24

    lds r24, FloorQueue
    lds r25, FloorQueue+1
    or r24, @0
    or r25, @1

    sts FloorQueue, r24
    sts FloorQueue+1, r25

    pop r24
    pop r25
    pop temp1
    pop temp2
.endmacro

.macro print_queue ; used for DEBUG purposes to see the FloorQueue
    push temp1
    push r25
    push r24

    clr r24
    clr r25
    lds r24, FloorQueue
    lds r25, FloorQueue+1
    print_digit r25
    do_lcd_data ':'
    print_digit r24

    pop r24
    pop r25
    pop temp1
.endmacro
