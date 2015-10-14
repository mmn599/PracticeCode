	.global dac_output

data_loc:
		.word 0x40012000 ;ADC14CTL
        .word 0x40004C23 ;p4out register

;Inputs expected:	void* output_data
;				 	void* dac_table
;					int MEMORY_EVERY calculation
;					int SAMPLE_EVERY calculation

dac_output:

		push {r0-r15}

        ldr r4, [pc, #-16]	;TODO
        ldr r5, [pc, #-32]	;TODO
        ldr r12, [pc, #-48]	;TODO
        ldr r6, [r5]
       	orr r6, r6, #1

       	mov r7, #0
       	mov r8, #0

		mov r10, r2
		mov r11, r3

       	;r0 is output data table pointer
        ;r1 is dac table pointer
        ;r2 is memory_every calculation
        ;r3 is sample_every calculation
        ;r4 is P4OUT register
        ;r5 contains ADC14CTL pointer
        ;r6 contains trigger sampling ADC14CTL bit
        ;r7 contains dac_table index
        ;r8 contains data_table index
        ;r9 contains temporary values
		;r10 contains adc trigger counter
		;r11 contains memory grab counter
		;r12 contains ADC14MEM0

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
		strb r6, [r5]	;sets ADC14CTL[0] to 1 enabling conversion

memory_check:
		adds r11, r11, #-1
		breq dma_transfer
no_memory_grab:
		nop
		nop
		nop
		b inc_pointers
memory_grab:
		ldrh r9, [r12] ;load ADCMEM14 into r9
		strh r9, [r1, r8] ;strh and ldrh only takes 3 cycles due to pipeline
		add r8, r8, #2

inc_pointers:
		add r7, r7, #1
		add r15, r8, #-256
		brne update_dac

cleanup:
		pop {r0-r15}
		mov pc, lr
