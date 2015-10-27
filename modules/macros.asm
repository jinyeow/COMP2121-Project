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
;#   STATUS MACROS     #
;#######################
; Status Bits: [ 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 ]
;   0 => EMERGENCY
;   1 => DEBOUNCE FLAG
;   2 => LIFT MOVING
;   3 => LIFT DIRECTION
;   4 => DOOR MOVING
;   5 => DOOR OPEN/CLOSE

; Constants defined to set/clr bits in the Lift "STATUS" register (r22)
.equ CLEAR_FLAGS   = 0x00
.equ EMERGENCY_ON  = 0b00000001
.equ EMERGENCY_OFF = 0b11111110 ; can use com instruction with a temp reg instead?
.equ DEBOUNCE_ON   = 0b00000010
.equ DEBOUNCE_OFF  = 0b11111101
.equ MOVING_ON     = 0b00000100
.equ MOVING_OFF    = 0b11111011
.equ DIR_UP        = 0b00001000
.equ DIR_DOWN      = 0b11110111
.equ DOOR_MOV      = 0b00010000
.equ DOOR_NOT_MOV  = 0b11101111
.equ DOOR_IS_OPEN  = 0b00100000
.equ DOOR_IS_CLOSE = 0b11011111

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

; Move the "current floor" represented by a bit in a WORD up
.macro move_current_floor_up
    push temp1
    push r25
    push r24

    lds r24, FloorBits
    lds r25, FloorBits+1

    lsl r24
    rol r25

    sts FloorBits, r24
    sts FloorBits+1, r25

    pop r24
    pop r25
    pop temp1
.endmacro

; Move the "current floor" represented by a bit in a WORD down
.macro move_current_floor_down
    push temp1
    push r25
    push r24

    lds r24, FloorBits
    lds r25, FloorBits+1

    lsr r24
    ror r25

    sts FloorBits, r24
    sts FloorBits+1, r25

    pop r24
    pop r25
    pop temp1
.endmacro

; Floors 0 - 7
.macro call_floor_low
    push temp1
    push r25
    push r24

    lds r24, FloorQueue
    lds r25, FloorQueue+1
    ori r24, @0

    sts FloorQueue, r24
    sts FloorQueue+1, r25

    pop r24
    pop r25
    pop temp1
.endmacro

; Floors 8 & 9
.macro call_floor_high
    push temp1
    push r25
    push r24

    lds r24, FloorQueue
    lds r25, FloorQueue+1
    ori r25, @0

    sts FloorQueue, r24
    sts FloorQueue+1, r25

    pop r24
    pop r25
    pop temp1
.endmacro

.macro leave_floor_queue_low
    push temp1
    push r25
    push r24

    lds r24, FloorQueue
    lds r25, FloorQueue+1
    ldi temp1, @0
    com temp1
    and r24, temp1

    sts FloorQueue, r24
    sts FloorQueue+1, r25

    pop r24
    pop r25
    pop temp1
.endmacro

.macro leave_floor_queue_high
    push temp1
    push r25
    push r24

    lds r24, FloorQueue
    lds r25, FloorQueue+1
    ldi temp1, @0
    com temp1
    and r25, temp1

    sts FloorQueue, r24
    sts FloorQueue+1, r25

    pop r24
    pop r25
    pop temp1
.endmacro
