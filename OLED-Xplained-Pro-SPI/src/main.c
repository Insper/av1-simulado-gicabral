#include "asf.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define LED1_PIO_ID		ID_PIOA
#define LED1_PIO		PIOA
#define LED1_PIN		0
#define LED1_IDX_MASK   (1 << LED1_PIN)

#define LED2_PIO_ID		ID_PIOC
#define LED2_PIO		PIOC
#define LED2_PIN		30
#define LED2_IDX_MASK   (1 << LED2_PIN)

#define LED_PIO_ID	    ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		      8
#define LED_IDX_MASK    (1<<LED_PIN)

#define LED3_PIO_ID	    ID_PIOB
#define LED3_PIO        PIOB
#define LED3_PIN		  2
#define LED3_IDX_MASK    (1<<LED3_PIN)

/* variaveis globais                                                    */
/************************************************************************/

volatile char flag_tc = 0;
volatile char flag_tc2 = 0;
volatile char flag_tc3 = 0;




void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);



/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void io_init(void);
void pisca_led(int n, int t, int n_led);
void pin_toggle(Pio *pio, uint32_t mask);

void io_init(void){
	/* led */
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_IDX_MASK, PIO_DEFAULT);
}


/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc = 1;
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc2 = 1;
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc3 = 1;
}

void pisca_led(int n, int t, int n_led){
	if(n_led == 1){
		for (int i=0;i<n;i++){
			pio_clear(LED1_PIO, LED1_IDX_MASK);
			delay_ms(t);
			pio_set(LED1_PIO, LED1_IDX_MASK);
			delay_ms(t);
		}
	}
	if(n_led == 3){
		for (int i=0;i<n;i++){
			pio_clear(LED3_PIO, LED3_IDX_MASK);
			delay_ms(t);
			pio_set(LED3_PIO, LED3_IDX_MASK);
			delay_ms(t);
		}
	}
	if(n_led == 2){
		for (int i=0;i<n;i++){
			pio_clear(LED2_PIO, LED2_IDX_MASK);
			delay_ms(t);
			pio_set(LED2_PIO, LED2_IDX_MASK);
			delay_ms(t);
		}
	}
}


/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, estado, 0, 0);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_IDX_MASK, estado, 0, 0);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_IDX_MASK, estado, 0, 0 );
};


/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

int main (void)
{
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Configura Leds */
	LED_init(0);
	
	
	board_init();
	sysclk_init();
	delay_init();
	sysclk_init();
	io_init();


  // Init OLED
	gfx_mono_ssd1306_init();
	
	TC_init(TC0, ID_TC1, 1, 5);
	TC_init(TC0, ID_TC0, 0, 1);
	TC_init(TC0, ID_TC2, 2, 10);
	

  
  // Escreve na tela um circulo e um texto
	//gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
    //gfx_mono_draw_string("rique", 50,16, &sysfont);
	

	while(1) {
		if(flag_tc){
			pisca_led(1,100,1);
			flag_tc = 0;
		}
		if(flag_tc2){
			pisca_led(1,100,2);
			flag_tc2 = 0;
		}
		if(flag_tc3){
			pisca_led(1,100,3);
			flag_tc3 = 0;
		}
	
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);

  }  
  return 0;
}