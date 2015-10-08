	.global dac_output



;uint8_t DacTable_32[32] = {0x19,0x1e,0x23,0x27,0x2b,0x2e,0x30,0x32,
;		0x32,0x32,0x30,0x2e,0x2b,0x27,0x23,0x1e,
;		0x19,0x14,0xf,0xb,0x7,0x4,0x2,0x0,
;		0x0,0x0,0x2,0x4,0x7,0xb,0xf,0x14};
data_loc:
		.word 0x40012000 ;ADC14CTL
        .word 0x40004C23 ;p4out register
dac_output:
        ldr r1, [pc, #-8] ;r1 contains P4OUT
        ldr r2, [pc, #-16] ;r2 contains ADC14CTL
        ldr r4, [r2]
        orr r4, r4, #1
        ;turn on ADC sampling
        str r4, [r2]
inner_loop:
        mov r3, #0x19
   		strb r3, [r1]
   		mov r3, #0x1e
   		strb r3, [r1]
   		mov r3, #0x23
   		strb r3, [r1]
   		mov r3, #0x27
   		strb r3, [r1]
   		mov r3, #0x2b
   		strb r3, [r1]
   		mov r3, #0x2e
   		strb r3, [r1]
   		mov r3, #0x30
   		strb r3, [r1]
   		mov r3, #0x32
   		strb r3, [r1]
   		mov r3, #0x32
   		strb r3, [r1]
   		mov r3, #0x32
   		strb r3, [r1]
   		mov r3, #0x30
   		strb r3, [r1]
   		mov r3, #0x2e
   		strb r3, [r1]
   		mov r3, #0x2b
   		strb r3, [r1]
   		mov r3, #0x27
   		strb r3, [r1]
   		mov r3, #0x23
   		strb r3, [r1]
   		mov r3, #0x1e
   		strb r3, [r1]
   		mov r3, #0x19
   		strb r3, [r1]
   		mov r3, #0x14
   		strb r3, [r1]
   		mov r3, #0xf
   		strb r3, [r1]
   		mov r3, #0xb
   		strb r3, [r1]
   		mov r3, #0x7
   		strb r3, [r1]
   		mov r3, #0x4
   		strb r3, [r1]
   		mov r3, #0x2
   		strb r3, [r1]
   		mov r3, #0x0
   		strb r3, [r1]
   		mov r3, #0x0
   		strb r3, [r1]
   		mov r3, #0x0
   		strb r3, [r1]
   		mov r3, #0x2
   		strb r3, [r1]
   		mov r3, #0x4
   		strb r3, [r1]
   		mov r3, #0x7
   		strb r3, [r1]
   		mov r3, #0xb
   		strb r3, [r1]
   		mov r3, #0xf
   		strb r3, [r1]
   		mov r3, #0x14
   		strb r3, [r1]
        b inner_loop

