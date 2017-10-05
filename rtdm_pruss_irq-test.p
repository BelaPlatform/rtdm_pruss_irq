.origin 0
.entrypoint START
#define us 100

.macro DELAY
.mparam time
        MOV r27, time
LOOP:
        SUB r27, r27, 1
        QBNE LOOP, r27, 0
.endm

START:
        DELAY ( 100 * 1000 * us)

        MOV R1, 0
QBBS DONT, R31.t0
        MOV R31.b0, (1 << 5) | 4
DONT:
        SET R30.t1
        CLR R30.t1
        JMP START

