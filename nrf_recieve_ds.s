Stack_Size    EQU 0x1024
top_of_stack  EQU 0x20000000 + Stack_Size
  
  PRESERVE8
  THUMB
 GET stm32f10x.asm
 GET nRF24l01.asm	 
 AREA	RESET,	DATA, READONLY
  DCD   top_of_stack  ;Top of Stack
  DCD   Init
	  space 52
 DCD SysTick_Handler
		space 24
	DCD     EXTI0_IRQHandler
		space 84;112;148;204
	DCD     TIM2_IRQHandler           ; TIM2
		space 32
	DCD      USART1_IRQHandler         ; USORT1	
		space 88  

	AREA const, DATA, READONLY
setup_nrf DCD 0x03220321, 0xFF240123, 0x03320331, 0x033c0226, 0x7e27FF50, 0x0F20063d, 0xA1A1A12B, 0xF130A1A1, 0xB1B1B1B1, 0xB1B1F12A, 0x0000B1B1 ;0xA1A1A12B, 0xF130A1A1, 0xF1F1F1F1, 0xF1F1F12A, 0x0000F1F1

	AREA   var, DATA, READwrite
usartbuff DCD 10
	space 40
spibuff DCD 10
	SPACE 40
usartnumbyte DCD 1
spinumbyte	DCD 1
temperbuff dcd 2
		space 8
dsbuff dcd 1
bmpbuff dcd 2
	space 8
shlakbuff DCD 10
	space 40


	

 AREA	text_code,	CODE, READONLY
	
 ENTRY
	
Init proc
	
;	mov32 r1, 0x61ef9101
;	lsr r1, 11
;	mov32 r2, 0xAD
;	clz r0, r2
;	mov32 r3, #32
;	sub r3, r3, r0
;	ror r2, r3
;	lsr r2, 11
;	mov32 r3, 0x10
;	mul r2, r2,r3
;	add r3, r2, r1
;	mul r1, r1, r1
	;neg r1, r1
	;movs r2, #5
	;mul r0, r1, r2
	;adds r3, r1, r2
	bl Init_mk
	bl Watchdog
	;bl USART_Init
	bl SPI1_Init
	bl DS_Init
	bl BMPInit
	bl Timer_Init
	bl EXTI_Init
	bl Interrupt_Init
	bl nrf_init
	
	b main
	endp
		
Init_mk proc
	
	;????????? ???????
	mov32 r0, AFIO_MAPR
	mov32 r1, AFIO_MAPR_SWJ_CFG_JTAGDISABLE
	str r1, [r0]
	
	mov32 r0, RCC_CR
	mov32 r1, RCC_CR_HSION
	str r1, [r0]
waithsion
	ldr r1, [r0]
	tst r1, RCC_CR_HSIRDY
	beq waithsion	
	
	mov32 r0, RCC_CFGR
	mov32 r1, RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL9 | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1
	str r1, [r0]
	
	mov32 r0, RCC_CR
	mov32 r1, RCC_CR_PLLON | RCC_CR_HSION | RCC_CR_HSITRIM
	str r1, [r0]
waitpllon
	ldr r1, [r0]
	tst r1, RCC_CR_PLLRDY
	beq waitpllon
	;??????????? ?????? ? ???? ?????? 
	MOV32 R0, FLASH_ACR
    MOV32 R1, FLASH_ACR_LATENCY_1 + FLASH_ACR_PRFTBE
    STR R1, [R0]
	
	mov32 r0, RCC_CFGR
	mov32 r1, RCC_CFGR_SW_PLL | RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL9 | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1
	str r1, [r0]
	; mov32 r0, RCC_APB1ENR
	 ;mov32 r1, RCC_APB1ENR_PWREN | RCC_APB1ENR_SPI2EN
	 ;str r1, [r0]

;???????? ?????????
	MOV32 R0, RCC_APB2ENR
	mov32 r1, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPBEN ;| RCC_APB2ENR_USART1EN  usart dliy otladki
	str r1, [r0]
	
	MOV32 R0, RCC_APB1ENR
	mov32 r1, RCC_APB1ENR_TIM2EN | RCC_APB1ENR_SPI2EN
	str r1, [r0]
	;usart pin init
	;mov32 r0, GPIOA_CRH
	;mov32 r1, GPIO_CRH_CNF10_0 | GPIO_CRH_MODE9_0 |GPIO_CRH_CNF9_1;GPIO_CRH_MODE10_0 |  ;
	;str r1, [r0]
	
	mov32 r0, GPIOA_CRL
	mov32 r1, GPIO_CRL_CNF5_1  | GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_1 | GPIO_CRL_MODE5_0 | GPIO_CRL_MODE4_0 | GPIO_CRL_MODE7_0 | GPIO_CRL_MODE1_0 |GPIO_CRL_CNF0_1;|GPIO_CRL_CNF2_1
	str r1, [r0]
	
	mov32 r0, GPIOA_BSRR
	mov32 r1, GPIO_BSRR_BS4 | GPIO_BSRR_BS0 | GPIO_BSRR_BS1
	str	r1, [r0]
	
	mov32 r0, GPIOB_CRH
	mov32 r1, GPIO_CRH_MODE12_0 | GPIO_CRH_CNF13_1 | GPIO_CRH_MODE13_0 | GPIO_CRH_CNF14_0 | GPIO_CRH_MODE15_0 | GPIO_CRH_CNF15_1
	str r1, [r0]
	
	mov32 r0, GPIOB_CRL
	mov32 r1, GPIO_CRL_MODE0_0  |GPIO_CRL_CNF0_0 |GPIO_CRL_CNF1_0 | GPIO_CRL_MODE1_1  ;|GPIO_CRL_CNF1_1; | GPIO_CRL_MODE1_0
	str r1, [r0]
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BS0 | GPIO_BSRR_BS1 | GPIO_BSRR_BS12
	str	r1, [r0]
	mov32 r11, usartbuff
	mov32 r10, spibuff
	mov32 r0, #100
	mov32 r1, #0
zero
	str r1, [r11]
	subs r0, 1
	add r11, #4
	bne zero
	mov32 r11, usartbuff;#0x20000000
	mov32 r10, spibuff
	mov32 r6, 0
	mov32 r7, #1
	bx lr
	endp
		
USART_Init proc
;234, 375  0.375
	mov32 r0, USART1_BRR
	mov32 r1, #3750
	str r1, [r0]
	
	;mov32 r0, USART1_CR2
	;mov32 r1, #1
	;str r1, [r0]
	
	mov32 r0, USART1_CR1
	mov32 r1, USART_CR1_UE | USART_CR1_RXNEIE |USART_CR1_RE  | USART_CR1_TE | USART_CR1_TCIE 
	str r1, [r0]
	
	MOV32 r0, USART1_SR
	mov32 r1, #0
	str r1, [r0]

	;mov32 r0,  USART1_DR
	;mov32 r1, #0x41
	;str r1, [r0]
	
	bx lr

 endp
	 
SPI1_Init PROC
	mov32 r0, SPI1_CR1
	mov32 r1, SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI
	str r1, [r0]
	mov32 r0, SPI2_CR1
	mov32 r1, SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI
	str r1, [r0]
	cpsie i
	
	bx lr
	ENDP

EXTI_Init PROC
	;????????????????? ?????????????????????pin 2
	;???????????
	;mov32 r0, AFIO_EXTICR1
	;mov32 r1, AFIO_EXTICR1_EXTI2
	;str r1, [r0]
	
	mov32 r0, EXTI_FTSR
	mov32 r1, EXTI_FTSR_TR0
	str r1, [r0]
	
	mov32 r0, EXTI_IMR
	mov32 r1, EXTI_IMR_MR0
	str r1, [r0]

	bx lr
	ENDP
		
Interrupt_Init PROC
;???????????????????????????????pin 2
	mov32 r0, NVIC_ISER0
	mov32 r1, NVIC_ISER_SETENA_6 | NVIC_ISER_SETENA_28
	str r1, [r0]
;?????????????????USART
	mov32 r7, #1
	mov32 r0, NVIC_ISER1
	MOV32 r1, NVIC_ISER_SETENA_5
	str r1, [r0]

	cpsie i
	
	bx lr
	ENDP

Timer_Init PROC
	;timer
	;mov32 r0, TIM2_SR
	;mov32 r1, TIM_SR_UIF
	;str r1, [r0]
	mov32 r0, TIM2_SMCR
	mov32 r1, #0;TIM_SMCR_SMS_2
	str r1, [r0]
	mov32 r0, TIM2_PSC
	mov32 r1, #0x8C9F
	str r1, [r0]	
	mov32 r0, TIM2_ARR
	mov32 r1, #0xEA60
	str r1, [r0]
	mov32 r0, TIM2_DIER
	mov32 r1, TIM_DIER_UIE
	str r1, [r0]
	mov32 r0, TIM2_CNT
	mov32 r1, #1
	str r1, [r0]
	mov32 r0, TIM2_CR1
	mov32 r1, TIM_CR1_CEN | TIM_CR1_ARPE
	str r1, [r0]
	bx lr
	ENDP
	
Watchdog PROC
	mov32 r0, IWDG_KR
	mov32 r1, #0x5555
	str r1, [r0]
	mov32 r0, IWDG_PR
	mov32 r1, #6
	str r1, [r0]
	mov32 r0, IWDG_RLR
	mov32 r1, #0xFFF
	str r1, [r0]
	mov32 r0, IWDG_KR
	mov32 r1, #0xCCCC
	str r1, [r0]
	bx lr
	ENDP

nrf_init proc
	mov32 r5, #4
	mov32 r9, #12;#21
	mov32 r11, setup_nrf
	ldr r2, [r11], 1
	mov32 r12, #2
loop_spi_init
	 
	;ldr r0, =nRF_Send
	push {lr}
	bl nRF_Send ;x r0
	pop {lr}
	subs r9, #1
	mov32ne r12, #2
	bne loop_spi_init
	;cmp r5, #3
	;subeq r11, #1
	;ldr r2, [r11], 1
	mov32 r9, #1
	mov32 r12, #6
	subs r5, #1
	bne loop_spi_init
	mov32 r11, shlakbuff
	mov32 r2, 0xE1
	str r2, [r11]
	mov32 r12, #1
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r11, shlakbuff
	mov32 r2, 0xE2
	str r2, [r11]
	mov32 r12, #1
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r11, usartbuff
	mov32 r10, spibuff
	bx lr
	ENDP

DS_Init PROC
	push {lr}
	bl resetds
	pop {lr}
	mov32 r2, #0xCC
	push {lr}
	bl Writeds
	pop {lr}
	;mov32 r2, #0x3F ;10bit resolution
	;push {lr}
	;bl Writeds
	;pop {lr}
	bx lr
	ENDP

BMPInit PROC
;	mov32 r12, #24
	mov32 r11, shlakbuff
;	mov32 r2, 0x88
;	str r2, [r11], 1
;	mov32 r9, #7
;	mov32 r2, 0xFFFFFFFF
;loopbmp
;	str r2, [r11], 4
;	subs r9, 1
;	bne loopbmp
;	mov32 r11, shlakbuff
;	ldr r2, [r11], 1
;	push {lr}
;	bl nRF_Send
;	pop {lr}
;	;SXTH r0,r0
;	mov32 r11, spibuff
;	add r11, #1
;	mov32 r10, big_tp
;	mov32 r12, #12
;loop_big_TP
;	ldrh r2, [r11], 2
;	;rev16 r1, r2
;	str r2, [r10], 2
;	subs r12, 1
;	bne loop_big_TP
;	mov32 r8, big_tp
	mov32 r10, spibuff
	mov32 r11, usartbuff
	mov32 r2, 0x0000FFD0
	str r2, [r11], 1
	mov32 r12, 2
	push {lr}
	bl SPI2_Send
	pop {lr}
	mov32 r2, 0x0000BF74
	str r2, [r11], 1
	mov32 r12, 2
	push {lr}
	bl SPI2_Send
	pop {lr}
	mov32 r2, 0x00009C75
	str r2, [r11], 1
	mov32 r12, 2
	push {lr}
	bl SPI2_Send
	pop {lr}
	mov32 r1, #0xFFFFFD
	push {lr}
	bl Delay2
	pop {lr}

;	mov32 r11, shlakbuff
;	mov32 r2, #0xFFFFFFFA
;	mov32 r12, #4
;	push {lr}
;	bl nRF_Send
;	pop {lr}
;	ldr r0, [r10]
;	rev r1, r0
;	lsr r1, #4
	
	bx lr
	ENDP


Delay2 PROC
		
	
	mov32 r0, SYST_RVR
	str r1, [r0]
	mov32 r0, SYST_CSR
	mov32 r1, SysTick_CTRL_ENABLE | SysTick_CTRL_TICKINT | SysTick_CTRL_CLKSOURCE
	str r1, [r0]
	mov32 r0, SYST_CVR
	str r1, [r0]
	wfi
	bx lr
	ENDP

SysTick_Handler PROC
		EXPORT SysTick_Handler            [WEAK]
	mov32 r0, SYST_CSR
	mov32 r1, #0
	str r1, [r0]
	bx lr
	ENDP
		
USART1_IRQHandler   PROC
	;push {r10, r11}
	MOV32 r0, USART1_SR
	ldrh r1, [r0, #0x00]
	and r1, USART_SR_TC
	cbnz r1, transmit
	
	ldrh r1, [r0, #0x00]
	and r1, USART_SR_RXNE
	cbnz r1, recive
	;pop {r10, r11}
	bx lr
transmit
	subs r7, 1
	beq exit
	mov32 r0,  USART1_DR
	ldr r1, [r10], 1
	str r1, [r0]
	;add r10, r10, #1
	;str r1, [r0]
	;LDMFD   r13!,{r0-r3,r6,pc}
exit
	MOV32 r0, USART1_SR
	mov32 r1, #0
	str r1, [r0]
	bx lr
	
recive
	mov32 r0, USART1_DR
	ldr r3, [r0]
	cmp r3, #0xAA
	beq exitrecive
	ldr r2, [r11]
	;orr r2, r3
	str r3, [r11], 1
	;add r11, r11, 1
	mov32 r0, usartbuff
	sub r0, r11, r0
	mov32 r1, usartnumbyte
	str r0, [r1]
	;pop {r10, r11}
	bx lr
exitrecive
	mov32 r6, #1
	mov32 r11, usartbuff
	;pop {r10, r11}
	bx lr
	ENDP
		
EXTI0_IRQHandler PROC
	mov32 r0, EXTI_PR
	mov32 r1, EXTI_PR_PR0
	str r1, [r0]
	mov32 r6, #2
	bx lr
	ENDP
		
TIM2_IRQHandler PROC
	mov32 r6, #4
	mov32 r0, TIM2_SR
	mov32 r1, #0
	str r1, [r0]
	bx lr	
	ENDP

main proc
loop
	mov32 r0, IWDG_KR
	mov32 r1, #0xAAAA
	str r1, [r0]
	
	cmp r6, #1 ;obrabotka usart komandi (vkluchit usart)
	beq parsentrycommand

	cmp r6, #2
	beq nrf_reciev ;obrabotat prerivanie nrf
	cmp r6, #3
	beq parsrecievcommand ;obrabotat komandi nrf
	cmp r6, #4
	beq send_temp ;prinuditelno otoslat temper
	cmp r6, #10
	beq soft_reset
	;wfi
 b loop

endp

parsentrycommand
	eor r6, r6
	mov32 r11, usartbuff
	ldrb r0, [r11]
	cmp r0, #1
	beq temper ; prinuditelno poluchit temper
	cmp r0, #2
	beq timer ; rabota s taimerom
	cmp r0, #3
	beq preparespi
	cmp r0, #4 ;obrabotka poluchenoy komandi
	beq send_temp
	b loop
temper
	push {lr}
	bl Starttemp
	pop {lr}
	b loop
timer
	push {lr}
	bl changetimer
	pop {lr}
	b loop
preparespi
	push {lr}
	bl send_nrf_command
	pop {lr}
	b loop
nrf_reciev
	push {lr}
	bl nrfirq
	pop {lr}
	b loop
send_temp
	;bl get_temp_ds
	push {lr}
	bl send_temp2
	pop {lr}
	b loop
send_timer_status
	push {lr}
	bl sendtimerstatus
	pop {lr}
exec_send_command
	push {lr}
	bl execsendcommand
	pop {lr}
	b loop

soft_reset
	b Init

parsrecievcommand
	eor r6, r6
	mov32 r11, shlakbuff
	mov r10, r11
	add r10, #2
	mov32 r2, #0x0000FF0A
	str r2, [r11]
	mov32 r12, #2
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r0, shlakbuff
	ldr r1, [r0]
	lsr r1, 24
	and r1, #0x000000FF
	ldrb r0, [r10, 1]
	cmp r1, r0
	bne loop
	add r10, #1
	ldrb r0, [r10, 1]
	;and r0, #1
	cmp r0, #1
	beq temper
	ldrb r0, [r10, 1]
	;and r0, #2
	cmp r0, #2
	beq timer
	ldrb r0, [r10, 1]
	;and r0, #3
	cmp r0, #3
	beq send_temp
	ldrb r0, [r10, 1]
	;and r0, #3
	cmp r0, #4
	beq send_timer_status
	cmp r0, #5
	beq exec_send_command
	cmp r0, 0x0A
	mov32eq r6, 0x0A
	beq loop
	;mov32 r1, #7307797 esli budut oshibki s otptavkoy raskomentit
	;push {lr}
	;bl Delay2
	;pop {lr}
err
	mov32 r11, shlakbuff
	mov32 r2, #0xE2
	str r2, [r11], 1
	mov32 r12, #1
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r11, shlakbuff
	mov32 r2, #0x6027
	str r2, [r11], 1
	mov32 r12, #2
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r11, usartbuff
	mov32 r10, spibuff
	push {lr}
	bl set_tx
	pop {lr}
	push {lr}
	bl senderror
	pop {lr}
	;eor r6, r6
	b loop

get_temp_ds PROC
	push {lr}
	bl resetds
	pop {lr}
	cmp r0, #0xFF
	bxeq lr
	push {lr}
	mov32 r2, #0xCC
	bl Writeds
	pop {lr}
	mov32 r2, #0x44 ;10bit resolution
	push {lr}
	bl Writeds
	pop {lr}
	
	push {lr}
	mov32 r1, 0xFFFFFF;#7200000
	bl Delay2
	pop {lr}
	push {lr}
	bl resetds
	pop {lr}
	push {lr}
	mov32 r2, #0xCC
	bl Writeds
	pop {lr}
	mov32 r2, #0xBE ;10bit resolution
	push {lr}
	bl Writeds
	pop {lr}
	nop
	nop
	nop
	nop
	nop
	push {lr}
	bl Readds
	pop {lr}
	eor r6, r6
	bx lr
	
	ENDP


Starttemp PROC
	mov32 r2, #0
	mov32 r3, #0
	mov32 r8, #0
	mov32 r11, temperbuff
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BR0
	str r1, [r0]
	mov32 r1, #648000
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BS0
	str r1, [r0]
	mov32 r1, #1400
	push {lr}
	bl Delay2
	pop {lr}
	;mov32 r0, GPIOB_CRL
	;mov32 r1, GPIO_CRL_CNF0_0
	;str r1, [r0]
	;mov32 r0, GPIOB_BSRR
	;mov32 r1, GPIO_BSRR_BR0
	;str r1, [r0]

	mov32 r0, GPIOB_IDR
	mov32 r1, GPIO_IDR_IDR0
	ldr r0, [r0]
	and r0, r1
	cmp r0, #1 ;?????? ?? ??????? ??????
	beq errortemp
	mov32 r1, #2800;2870;5740;#2880
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOB_IDR
	ldr r0, [r0]
	mov32 r1, GPIO_IDR_IDR0
	and r0, r1
	cmp r0, #1
	bne errortemp
	mov32 r1, #2800;#2870
	push {lr}
	bl Delay2
	pop {lr}
	b loopstart

errortemp ;?????? ??????? ??????
	;mov32 r0, GPIOB_CRL
	;mov32 r1, GPIO_CRL_MODE0_0
	;str r1, [r0]
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BS0
	str r1, [r0]
	cmp r3, #5
	eoreq r6, r6
	mov32ne r6, #0xFF;?????????? ??????? ?????? ??????????? ?? spi
	mov32 r11, usartbuff
	mov32 r10, spibuff
	bx lr ;?????? ??????? ??????

loopstart
	mov32 r0, GPIOB_IDR
	ldr r0, [r0]
	and r0, GPIO_IDR_IDR0
	cmp r0, 1
	bne loopstart
	mov32 r1, #1080;#1080;#980
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOB_IDR
	ldr r0, [r0]
	and r0, GPIO_IDR_IDR0
	cmp r0, 1
	bne zero2
	orr r8, #1
	lsl r8, 1
	add r2, #1
	cmp r2, #8
	lsreq r8, 1
	streq r8, [r11], 1
	eoreq r8, r8
	eoreq r2, r2
	addeq r3, #1
	cmp r3, 5
	beq errortemp
waitnoone	
	mov32 r0, GPIOB_IDR
	ldr r0, [r0]
	and r0, GPIO_IDR_IDR0
	cmp r0, 1											;ldr r0, =Delay2
	beq waitnoone											;mov32 r1, #1527;#1527;#980

	b loopstart
zero2
	lsl r8, 1
	add r2, #1
	cmp r2, #8
	lsreq r8, 1
	streq r8, [r11], 1
	eoreq r8, r8
	cmp r2, #8
	bne loopstart
	eor r2, r2
	add r3, #1
	cmp r3, 5
	beq errortemp
	b loopstart
	ENDP	

changetimer proc
	push {r10, r11}
	;cmp r6, #3
	mov32 r11, spibuff ;mov32eq r11, spibuff
	addeq r11, #1
	;mov32ne r11, usartbuff
	;eor r6, r6
	ldrb r0, [r11, 1]!
	cmp r0, #1
	beq stoptimer
	cmp r0, #2
	beq chantimer
	cmp r0, #3
	beq starttimer
	pop {r10, r11}
	bx lr
stoptimer
	mov32 r0, TIM2_CR1
	mov32 r1, #0;TIM_CR1_CEN
	str r1, [r0]
	pop {r10, r11}
	bx lr
chantimer
	ldrh r1, [r11, 1]!
	mov32 r0, TIM2_PSC
	rev16 r2, r1
	str r2, [r0]
	mov32 r0, TIM2_ARR
	ldrh r1, [r11, 2]!
	rev16 r2, r1
	str r2, [r0] 
	pop {r10, r11}
	bx lr
	
starttimer
	mov32 r0, TIM2_CR1
	mov32 r1, TIM_CR1_CEN
	ldr r2, [r0]
	and r2, r1
	cmp r2, TIM_CR1_CEN
	bne strttimer
	pop {r10, r11}
	bx lr
strttimer
	mov32 r0, TIM2_CNT
	mov32 r1, #0
	str r1, [r0]
	mov32 r0, TIM2_CR1
	mov32 r1, TIM_CR1_CEN | TIM_CR1_ARPE
	str r1, [r0]
	pop {r10, r11}
	bx lr
	endp

nRF_Send PROC
	mov32 r0, GPIOA_BSRR
	mov32 r4,  GPIO_BSRR_BR4
	str	r4, [r0]
	mov32 r3, SPI1_SR
	mov32 r0, SPI1_DR
	;ldr r8, [r0]
	str r2, [r0]
wait	
	ldr r1, [r3]
	and r1, SPI_SR_TXE
	cmp r1, SPI_SR_TXE
	bne wait
	
	;ldr r0, =Delay2
	mov32 r1, #300
	push {lr}
	bl Delay2;x r0
	pop {lr}
	mov32 r0, SPI1_DR
	ldr r4, [r0]
	str r4, [r10], 1
	;orr r5, r4
	;lsl r5, 8
	;add r11, r11, 1
	ldr r2, [r11], 1
	subs r12, #1
	beq exitnRF_Send
	str r2, [r0]
	b wait

exitnRF_Send
;	mov32 r0, usartbuff
;	sub r0, r11, r0
;	sub r0, 1
;	mov32 r10, spinumbyte
;	str r0, [r10]
	mov32 r10, spibuff
	mov32 r0, GPIOA_BSRR
	mov32 r4, GPIO_BSRR_BS4
	str	r4, [r0]
	;mov32 r6, 2
	bx lr
	ENDP

SPI2_Send PROC
	mov32 r0, GPIOB_BSRR
	mov32 r4,  GPIO_BSRR_BR12
	str	r4, [r0]
	mov32 r3, SPI2_SR
	mov32 r0, SPI2_DR
	;ldr r8, [r0]
	str r2, [r0]
wait2	
	ldr r1, [r3]
	and r1, SPI_SR_TXE
	cmp r1, SPI_SR_TXE
	bne wait2
	
	;ldr r0, =Delay2
	mov32 r1, #300
	push {lr}
	bl Delay2;x r0
	pop {lr}
	mov32 r0, SPI2_DR
	ldr r4, [r0]
	str r4, [r10], 1
	;orr r5, r4
	;lsl r5, 8
	;add r11, r11, 1
	ldr r2, [r11], 1
	subs r12, #1
	beq exitSPI2_Send
	str r2, [r0]
	b wait2

exitSPI2_Send

	mov32 r0, GPIOB_BSRR
	mov32 r4, GPIO_BSRR_BS12
	str	r4, [r0]
	;mov32 r6, 2
	bx lr
	ENDP

send_nrf_command PROC
	mov32 r9, usartnumbyte
	ldr r9, [r9]
	sub r9, #1
	mov32 r10, spibuff
	mov32 r11, usartbuff
	ldrb r0, [r11, 1]
	;and r0, W_TX_PAYLOAD
	cmp r0, W_TX_PAYLOAD
	beq settx
	bne notsettx
settx
	push {lr}
	bl set_tx
	pop {lr}
notsettx
	add r11, #1
	ldr r2, [r11], 1
;loop_spi_init2
	mov r12, r9
	push {lr}
	bl nRF_Send ;x r0
	pop {lr}
;	subs r9, #1
;	bne loop_spi_init2
	mov32 r11, usartbuff
	mov32 r10, spibuff
	ldrb r0, [r11, 1]
	;and r0, W_TX_PAYLOAD
	cmp r0, W_TX_PAYLOAD
	beq startsend
	cmp r0, W_TX_PAYLOAD_NOACK
	beq startsend
	bx lr
startsend
	mov32 r0, GPIOA_BSRR
	mov32 r4, GPIO_BSRR_BS1
	str	r4, [r0]
	mov32 r1, #350
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOA_BSRR
	mov32 r4,  GPIO_BSRR_BR1
	str	r4, [r0]
	bx lr
	ENDP
		
nrfirq proc
	eor r6, r6
	mov32 r11, shlakbuff
	mov32 r1, 0x0000FF07
	str r1, [r11]
	mov32 r10, spibuff
	mov32 r12, #2
	ldr r2, [r11], 1
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r0, spibuff
	ldr r1, [r0, 1]
	and r1, #32
	cbnz r1, resettx
	ldr r1, [r0, 1]
	and r1, #16
	cbnz r1, resettxflush
	ldr r1, [r0, 1]
	and r1, #64
	beq.w resetrx
resettx
	add r7, #1
	ldr r2, [r0, 1]
	and r2, #32
	mov32 r11, shlakbuff
	lsl r2, 8
	orr r2, #0x27
	;mov32 r2, #0x00002027 ;del
	str r2, [r11], 1
	mov32 r12, #2
	mov32 r10, spibuff
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r0, 0
	mov32 r10, spinumbyte
	str r0, [r10]
	;cmp r6, #2
	;eoreq r6, r6
	mov32 r11, usartbuff
	mov32 r10, spibuff
	push {lr}
	bl set_rx
	pop {lr}
	bx lr
resettxflush
	mov32 r11, shlakbuff
	mov32 r2, 0x000000E1
	str r2, [r11], 1
	mov32 r12, #1
	mov32 r10, spibuff
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r0, 0
	mov32 r10, spinumbyte
	str r0, [r10]
	mov32 r2, #0x00001027
	mov32 r11, shlakbuff
	str r2, [r11], 1
	mov32 r12, #2
	mov32 r10, spibuff	
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r0, 0
	mov32 r10, spinumbyte
	str r0, [r10]
	mov32 r11, usartbuff
	eoreq r6, r6
	push {lr}
	bl set_rx
	pop {lr}
	bx lr
resetrx
	mov32 r11, shlakbuff
	mov32 r2, 0xFF
	lsl r2, 8
	orr r2, R_RX_PL_WID
	str r2, [r11], 1
	mov32 r12, #2
	mov32 r10, spibuff
	push {lr}
	bl nRF_Send
	pop {lr}
	;mov32 r0, 0
	mov32 r0, spinumbyte
	;str r0, [r10]
	mov32 r10, spibuff
	ldr r1, [r10, 1]
	str r1, [r0]
	;mov32 r1, 10 ;??????? ?????
	cmp r1, #32
	bhi flushrx
	mov32 r11, shlakbuff
	mov32 r2, R_RX_PAYLOAD
	str r2, [r11], 1
	;mov32 r1, 10 ;??????? ?????
	mov32 r2, 0xFF
looppreparedata
	str r2, [r11], 1
	subs r1, 1
	bne looppreparedata
	ldr r12, [r10, 1]
	;mov32 r12, 10 ;??????? ?????
	add r12, 1
	push {r12}
	
	mov32 r11, shlakbuff
	mov32 r10, spibuff
	ldr r2, [r11], 1
	push {lr}
	bl nRF_Send
	pop {lr}

	mov32 r1, spinumbyte
	b exitreaddata
	;b waitusartsend
flushrx
	mov32 r11, shlakbuff
	mov32 r2, 0x000000E2
	str r2, [r11], 1
	mov32 r12, #1
	mov32 r10, spibuff
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r0, 0
	mov32 r10, spinumbyte
	str r0, [r10]
exitreaddata
	;eor r6, r6
	mov32 r2, #0x00004027
	mov32 r11, shlakbuff
	str r2, [r11], 1
	mov32 r12, #2
	mov32 r10, shlakbuff+2	
	push {lr}
	bl nRF_Send
	pop {lr}
	pop {r12}
	mov32 r10, spinumbyte
	str r12, [r10]
	mov32 r11, usartbuff
	mov32 r10, spibuff
	mov32 r6, #3
	bx lr
	endp

send_temp2 PROC
	;eor r6, r6
	push {lr}
	bl set_tx
	pop {lr}
	push {r10, r11, r12}
	push {lr}
	bl Starttemp
	pop {lr}
	cmp r6, #0xFF
	bne noerrdht
	mov32 r1, temperbuff
	mov32 r2, #0xFFFFFFFF
	str r2, [r1], 4
	strb r2, [r1]
	
noerrdht
	push {lr}
	bl get_temp_ds
	pop {lr}
	cmp r0, #0xFF
	bne noerrds
	mov32 r1, dsbuff
	mov32 r2, #0x00FFFFFF
	str r2, [r1]
noerrds
	push {lr}
	bl get_data_bmp
	pop {lr}
	;bx lr ;;;;;
	mov32 r12, #15
	mov32 r11, shlakbuff
	mov32 r1, W_TX_PAYLOAD
	str r1, [r11], 1
	mov32 r1, temperbuff
	ldr r0, [r1], 4
	str r0, [r11], 4
	ldr r0, [r1]
	str r0, [r11], 1
	mov32 r1, dsbuff
	ldr r0, [r1]
	str r0, [r11], 3
	mov32 r1, bmpbuff
	add r1, #1
	ldr r0, [r1], 4
	str r0, [r11], 4
	ldr r0, [r1]
	str r0, [r11]
	
	mov32 r11, shlakbuff
	mov32 r10, spibuff
	ldr r2, [r11], 1
	push {lr}
	bl nRF_Send
	pop {lr}
	push {lr}
	bl startsend
	pop {lr}
	pop {r10, r11, r12}
exitnrfsend
	;push {lr}
	;bl set_rx
	;pop {lr}
	
	eor r6, r6
	bx lr
senderror
	eor r6, r6
	mov32 r2, #0x525245A0
	mov32 r11, shlakbuff
	str r2, [r11]
	mov32 r12, #4
	add r11, #1
	push {lr}
	bl nRF_Send
	pop {lr}
	push {lr}
	bl startsend
	pop {lr}	
	b exitnrfsend	
	ENDP

sendtimerstatus PROC
	eor r6, r6
	push {lr}
	bl set_tx
	pop {lr}
	push {r10, r11, r12}
	mov32 r11, shlakbuff
	mov32 r10, spibuff
	mov32 r1, W_TX_PAYLOAD
	str r1, [r11], 1
	mov32 r0, TIM2_CR1
	ldr r1, [r0]
	and r1, TIM_CR1_CEN
	str r1, [r11], 4
	mov32 r0, TIM2_ARR
	ldr r1, [r0]
	str r1, [r11], 4
	mov32 r0, TIM2_PSC
	ldr r1, [r0]
	str r1, [r11], 4
	mov32 r11, shlakbuff
	mov32 r12, #13
	ldr r2, [r11], 1
	push {lr}
	bl nRF_Send
	pop {lr}
	push {lr}
	bl startsend
	pop {lr}
	pop {r10, r11, r12}
	bx lr
	endp

set_tx proc
	push {r10, r11, r0, r1, r2}
	mov32 r11, shlakbuff
	mov r10, r11
	add r10, #2
	mov32 r2, #0x00000E20
	str r2, [r11],1
	mov32 r12, #2
	push {lr}
	bl nRF_Send
	pop {lr} 
	mov32 r0, GPIOA_BSRR
	mov32 r1, GPIO_BSRR_BR1
	str	r1, [r0]
	mov32 r1, #7759840
	push {lr}
	bl Delay2
	pop {lr}
	pop {r10, r11, r0, r1, r2}
	bx lr
	ENDP
	
set_rx proc
	push {r10, r11, r0, r1, r2}
	mov32 r11, shlakbuff
	mov r10, r11
	add r10, 2
	mov32 r2, #0x00000F20
	str r2, [r11], 1
	mov32 r12, #2
	push {lr}
	bl nRF_Send
	pop {lr} 
	mov32 r0, GPIOA_BSRR
	mov32 r1, GPIO_BSRR_BS1
	str	r1, [r0]
	;mov32 r1, #5759840
	;push {lr}
	;bl Delay2
	;pop {lr}
	pop {r10, r11, r0, r1, r2}
	bx lr
	ENDP
		
resetds proc
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BR1
	str r1, [r0]
	mov32 r1, #17280
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BS1
	str r1, [r0]
	mov32 r1, #1080
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOB_IDR
	mov32 r1, GPIO_IDR_IDR1
	ldr r0, [r0]
	and r0, r1
	cmp r0, #2 ;?????? ?? ??????? ??????
	beq errords
	mov32 r1, #3888
	push {lr}
	bl Delay2
	pop {lr}
	bx lr
	
errords
	mov32 r0, #0xFF
	bx lr
	ENDP

Readds PROC
	eor r2, r2
	mov32 r12, #8
	mov32 r9, #8
	push {r11}
	mov32 r11, dsbuff
dsloop
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BR1
	str r1, [r0]
	mov32 r1, #10
loopdsrd2
	subs r1, #1
	bne loopdsrd2
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BS1
	str r1, [r0]
	mov32 r1, #10
loopdsrd1
	subs r1, #1
	bne loopdsrd1
	mov32 r0, GPIOB_IDR
	mov32 r1, GPIO_IDR_IDR1
	ldr r0, [r0]
	and r0, r1
	cmp r0, #2 ;?????? ?? ??????? ??????
	;orreq r2, #1
	orreq r2, 0x80
	mov32 r1, 1620
	push {lr}
	bl Delay2
	pop {lr}
	
	subs r12, #1
	lsrne r2, 1
	bne dsloop
	str r2, [r11], 1
	mov32 r12, #8
	eor r2, r2
	subs r9, 1
	bne dsloop
	eor r12, r12
	pop {r11}
	bx lr
	endp

Writeds PROC
	mov32 r12, #8
dsloopwrite
	mov r3, r2
	lsr r2, 1
	and r3, #1
	cmp r3, #1
	beq dsone
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BR1
	str r1, [r0]
	mov32 r1, #4320
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BS1
	str r1, [r0]
	mov32 r1, #10
loopdswr1
	subs r1, #1
	bne loopdswr1
	subs r12, #1
	bne dsloopwrite
	bx lr
dsone
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BR1
	str r1, [r0]
	mov32 r1, #10
loopdswr2
	subs r1, #1
	bne loopdswr2
	mov32 r0, GPIOB_BSRR
	mov32 r1, GPIO_BSRR_BS1
	str r1, [r0]
	mov32 r1, #1650
	push {lr}
	bl Delay2
	pop {lr}
	subs r12, #1
	bne dsloopwrite
	
	bx lr
	ENDP

get_data_bmp PROC
	push {r10, r11}
	mov32 r11, shlakbuff
	mov32 r10, bmpbuff


	
	
	mov32 r2, #0x00FFFFFF
	str r2, [r11, 4]
	mov32 r2, #0xFFFFFFF7
	str r2, [r11] ,1
	mov32 r12, #7
	push {lr}
	bl SPI2_Send
	pop {lr}
	pop {r10, r11}
	bx lr
	ENDP

execsendcommand PROC
	push {lr}
	bl set_tx
	pop {lr}
	mov32 r12, spinumbyte
	ldr r12, [r12]
	sub r12, #2
	push {r12}
	mov r0, r12
	mov32 r11, shlakbuff
	add r10, #2
copymem
	ldr r1, [r10], 4
	str r1, [r11], 4
	subs r0, #4
	bpl copymem
	mov32 r11, shlakbuff
	mov32 r10, spibuff
	ldr r2, [r11]
	push {lr}
	bl nRF_Send
	pop {lr}
	;mov32 r12, spinumbyte
	;ldr r12, [r0]
	pop {r12}
	add r10, #1
	mov r0, r12
	mov32 r11, shlakbuff
	mov32 r1, #0xFF
	lsl r1, 8
	orr r1, W_TX_PAYLOAD
	str r1, [r11], 2
copymem1
	ldr r1, [r10], 4
	str r1, [r11], 4
	subs r0, #4
	bpl copymem1
	mov32 r11, shlakbuff
	mov32 r10, spibuff
	ldr r2, [r11], 1
	add r12, #1
	push {lr}
	bl nRF_Send
	pop {lr}
	mov32 r0, GPIOA_BSRR
	mov32 r4, GPIO_BSRR_BS1
	str	r4, [r0]
	mov32 r1, #350
	push {lr}
	bl Delay2
	pop {lr}
	mov32 r0, GPIOA_BSRR
	mov32 r4,  GPIO_BSRR_BR1
	str	r4, [r0]
	bx lr
	ENDP
	END