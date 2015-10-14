	.global dac_output

data_loc:
		.word 0x11111111 ;DMA trigger register
		.word 0x40012000 ;ADC14CTL
        .word 0x40004C23 ;p4out register

;Inputs expected:	void* output_data
;				 	void* dac_table
;				 	void* dma control table pointer
;					int SAMPLE_EVERY calculation

dac_output:

		push {r0-r15}

        ldr r4, [pc, #-8]
        ldr r5, [pc, #-16]
        ldr r12, [pc, #-24]
        ldr r13, [r12]
        orr r13, r13, #1

        ldr r6, [r5]
       	orr r6, r6, #1
       	mov r7, #0
       	mov r8, #0

		mov r10, r3
		mov r14,
		mov r11, 14

       	;r0 is output data table pointer
        ;r1 is dac table pointer
        ;r2 is dma control pointer
        ;r3 is sample_every calculation
        ;r4 is P4OUT register
        ;r5 contains ADC14CTL pointer
        ;r6 contains trigger sampling ADC14CTL bit
        ;r7 contains dac_table index
        ;r8 contains data_table index
        ;r9 contains temporary values
		;r10 contains adc trigger counter
		;r11 contains dma transfer counter
		;r12 contains DMA trigger register pointer
        ;r13 contains DMA trigger register set bit

update_dac:
		;move value in P4OUT for DAC
		ldrb r9, [r1, r7]
   		strb r9, [r4]

trigger_check:
		;decerement adc trigger counter
		adds r10, r10, #-1
		beq trigger_sample
no_trigger_sample:
		;nop to ensure this takes the same number of cycles as when adc is triggered
		nop
		nop
		b transfer_check
trigger_sample:
		mov r10, r3
		;sets ADC14CTL[0] to 1 enabling conversion
		strb r6, [r5]

transfer_check:
		adds r11, r11, #-1
		breq dma_transfer
no_dma_transfer:
		nop
		nop
		nop
		nop
		nop
		nop
		b inc_pointers
dma_transfer:
		;move 16 bit data table array pointer up 32 indexes. (e.g. data_table[0] to data_table[32])
		add r0, r0, r8
		;store new destination address in dma control table (pointed to by r2)
		strb r0, [r2]
		;initialize dma transfer
		strb r13, [r12]
		;increment data_table index offset
		add r8, r8, #64
		;reset dma transfer counter
		mov r11, r14

inc_pointers:
		add r7, r7, #1
		add r15, r8, #-256
		brne update_dac

cleanup:
		pop {r0-r15}
		mov pc, lr
s
