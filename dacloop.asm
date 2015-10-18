	.global simple_dac_output

data_loc:
		.word 0x40012098 ;ADC14MEM0
		.word 0x40012000 ;ADC14CTL
        .word 0x40004C23 ;p4out register

        ;r0 is output data table pointer
        ;r1 is dac table pointer
simple_dac_output:

		push {r4-r10}

        ldr r2, [pc, #-8] ;r2 contains P4OUT pointer
        ldr r3, [pc, #-16] ;r3 contains ADC14CTL pointer
        ldr r4, [pc, #-24] ;r4 contains ADC14MEM0 pointer
        ldr r5, [r3]
       	orr r5, r5, #1		;r4 sets the bit in ADC14CTL to enable conversions
       	mov r8, #0
       	mov r9, #0

inner_loop:

		;move value in P4OUT for DAC
		ldrb r6, [r1, r8] ;r8 is counter variable
   		strb r6, [r2]
   		;start ADC conversion
   		strb r5, [r3]

		add r8, r8, #1
		and r8, r8, #0x3F

		nop
		nop
		nop
		nop

		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop

   		;read conversion from ADC14MEM0
   		ldrh r7, [r4]
   		;store conversion into pointer arguments
   		strh r7, [r0, r9]
   		;increment data table index (2 bytes)
   		add r9, r9, #2
   		subs r10, r9, #0x3F
   		bne inner_loop

		pop {r4-r10}

		mov pc, lr



