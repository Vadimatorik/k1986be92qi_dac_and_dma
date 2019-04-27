#include "1986be9x_config.h"
#include "1986BE9x.h"
#include "1986BE9x_uart.h"
#include "1986BE9x_port.h"
#include "1986BE9x_rst_clk.h"
#include "1986BE9x_it.h"
#include "mlt_lcd.h"
#include "MilFlash.h"

//---------------------------------------------------------
//ЦАП.
//---------------------------------------------------------
#define PCLK_EN(DAC)             (1<<18)                                //Маска включения тактирования ЦАП. 
#define CFG_Cfg_ON_DAC0          (1<<2)                                 //Маска включения ЦАП1.                    
#define CFG_Cfg_ON_DAC1          (1<<3)
void DAC_Init (void)
{
	RST_CLK->PER_CLOCK |= PCLK_EN(DAC);             //Включаем тактирование ЦАП.
	DAC->CFG = CFG_Cfg_ON_DAC1;                     //Включаем ЦАП2. Ассинхронно. От внутреннего источника.
}

#define CLKSOURCE (1<<2)                          //Указывает источник синхросигнала: 0 - LSI, 1 - HCLK.
#define TCKINT    (1<<1)                          //Разрешает запрос на прерывание от системного таймера.
#define ENABLE    (1<<0)                          //Разрешает работу таймера.

//---------------------------------------------------------
//Прерывание 10000000 раз в секунду. 
//---------------------------------------------------------


uint16_t C_4[100] = {0x7FF, 0x803, 0x808, 0x80C, 0x810, 0x815, 0x819, 0x81D, 0x821, 0x825, 0x828, 0x82C, 0x82F, 0x832, 0x835, 0x838, 0x83A, 0x83C, 0x83E, 0x840, 0x842, 0x843, 0x844, 0x844, 0x845, 0x845, 0x845, 0x844, 0x844, 0x843, 0x842, 0x840, 0x83E, 0x83C, 0x83A, 0x838, 0x835, 0x832, 0x82F, 0x82C, 0x828, 0x825, 0x821, 0x81D, 0x819, 0x815, 0x810, 0x80C, 0x808, 0x803, 0x7FF, 0x7FB, 0x7F6, 0x7F2, 0x7EE, 0x7E9, 0x7E5, 0x7E1, 0x7DD, 0x7D9, 0x7D6, 0x7D2, 0x7CF, 0x7CC, 0x7C9, 0x7C6, 0x7C4, 0x7C2, 0x7C0, 0x7BE, 0x7BC, 0x7BB, 0x7BA, 0x7BA, 0x7B9, 0x7B9, 0x7B9, 0x7BA, 0x7BA, 0x7BB, 0x7BC, 0x7BE, 0x7C0, 0x7C2, 0x7C4, 0x7C6, 0x7C9, 0x7CC, 0x7CF, 0x7D2, 0x7D6, 0x7D9, 0x7DD, 0x7E1, 0x7E5, 0x7E9, 0x7EE, 0x7F2, 0x7F6, 0x7FB};
#define HCLK_SEL(CPU_C3)       (1<<8)
#define CPU_C1_SEL(HSE)        (1<<1)
#define CPU_C2_SEL(CPU_C2_SEL) (1<<2)
#define PCLK_EN(RST_CLK)       (1<<4)
#define HS_CONTROL(HSE_ON)     (1<<0)
#define REG_0F(HSI_ON)        ~(1<<22)
#define RTC_CS(ALRF)           (1<<2)
#define PCLK(BKP)              (1<<27)

#define RST_CLK_ON_Clock()       RST_CLK->PER_CLOCK |= PCLK_EN(RST_CLK)                 //Включаем тактирование контроллера тактовой частоты (по умолчанию включено).
#define HSE_Clock_ON()           RST_CLK->HS_CONTROL = HS_CONTROL(HSE_ON)               //Разрешаем использование HSE генератора. 
#define HSE_Clock_OffPLL()       RST_CLK->CPU_CLOCK  = CPU_C1_SEL(HSE)|HCLK_SEL(CPU_C3);//Настраиваем "путь" сигнала и включаем тактирование от HSE генератора.

#define PLL_CONTROL_PLL_CPU_ON  (1<<2)                                                  //PLL включена. 
#define PLL_CONTROL_PLL_CPU_PLD (1<<3)                                                  //Бит перезапуска PLL.
void HSE_PLL (uint8_t PLL_multiply)                                                              //Сюда передаем частоту в разах "в 2 раза" например. 
{
	RST_CLK->PLL_CONTROL  = RST_CLK->PLL_CONTROL&(~(0xF<<8));                                      //Удаляем старое значение.
	RST_CLK->PLL_CONTROL |= PLL_CONTROL_PLL_CPU_ON|((PLL_multiply-1)<<8)|PLL_CONTROL_PLL_CPU_PLD;  //Включаем PLL и включаем умножение в X раз, а так же перезапускаем PLL.
	RST_CLK->CPU_CLOCK   |= HCLK_SEL(CPU_C3)|CPU_C2_SEL(CPU_C2_SEL)|CPU_C1_SEL(HSE);               //Настриваем "маршрут" частоты через PLL и включаем тактирование от HSE.
}

//---------------------------------------------------------
//Настраиваем выход, подключенный к усилителю. 
//---------------------------------------------------------
#define PER_CLOCK_PORTE              (1<<25)      //Бит включения тактирования порта E.
#define PORT_OE_OUT_PORTE_0          (1<<0)       //Включение этого бита переводит PORTE_0 в "выход". 
#define ANALOG_EN_DIGITAL_PORTE_0    (1<<0)       //Включаем цифровой режим бита порта PORTE_0.
#define PWR_MAX_PORTE_0              (3<<0)       //Включение данных бит переключает PORTE_0 в режим максимальной скорости.

#define PORT_RXTX_PORTE_0_OUT_1      (1<<0)       //Маска порта для подачи "1" на выход.

void Buzzer_out_init (void)
{
	RST_CLK->PER_CLOCK |= PER_CLOCK_PORTE;          //Включаем тактирование порта E.
	PORTE->OE |= PORT_OE_OUT_PORTE_0;               //Выход. 
	PORTE->ANALOG |= ANALOG_EN_DIGITAL_PORTE_0;     //Цифровой.
	PORTE->PWR |= PWR_MAX_PORTE_0;                  //Максимальная скорость (около 10 нс).
}

void Buzzer_out_DAC_init (void)
{
	RST_CLK->PER_CLOCK |= PER_CLOCK_PORTE;          //Включаем тактирование порта E.
	PORTE->OE |= PORT_OE_OUT_PORTE_0;               //Выход. 
	PORTE->ANALOG = 0;                              //Аналоговый.
	PORTE->PWR |= PWR_MAX_PORTE_0;                  //Максимальная скорость (около 10 нс).
}
#define CFG_master_enable             (1<<0)      //Маска разрешает работу контроллера.
#define PCLK_EN_DMA                   (1<<5)      //Маска включает тактирование DMA.

//Параметры для нашей структуры. 
#define dst_src       (3<<30)                 //Источник - 16 бит (полуслово).
#define src_inc       (1<<26)                 //Источник смещается на 16 бит после каждой передачи. 
#define src_size      (1<<24)                 //Отправляем по 16 бит.
#define dst_size      (1<<28)                 //Принимаем по 16 бит. (Приемник и передатчик должны иметь одинаковые размерности).
#define dst_prot_ctrl                         //Здесь настраивается различного рода защита (буферизация, привилегированный режим, )
#define R_power       (0<<14)                 //Арбитраж (приостановка передачи до внешнего сигнала, разрешающего ее продолжение) после каждой передачи. 
#define n_minus_1     (99<<4)                 //100  передачь DMA. 
#define next_useburst (0<<3)                  //Так и не удалось понять, что это...
#define cycle_ctrl    (1<<0)                  //Обычный режим.

//Настраиваем структуру.
#define ST_DMA_DAC_STRYKT dst_src|src_inc|src_size|dst_size|R_power|n_minus_1|next_useburst|cycle_ctrl
struct DAC_ST
{
	uint32_t Destination_end_pointer;                                     //Указатель конца данных приемника.
	uint32_t Source_end_pointer;                                          //Указатель конца данных источника
	uint32_t channel_cfg;                                                 //Конфигурация канала.
	uint32_t NULL;                                                        //Пустая ячейка. 
} 
__align(1024) DAC_ST; 
struct DAC_ST DAC_ST_ADC[8] ;

void DMA_and_DAC (void) 
{
  DAC_ST_ADC[7].Destination_end_pointer = (uint32_t)C_4 + sizeof(C_4) - 1;           //Указатель на последний адрес источника (C_4 - массив значений синусоидального сигнала в 100 значений).
  DAC_ST_ADC[7].Source_end_pointer = (uint32_t)&(DAC->DAC2_DATA);                    //Указатель на последний (не меняется) адрес приемника (регистр данных DAC)
  DAC_ST_ADC[7].channel_cfg = (uint32_t)(ST_DMA_DAC_STRYKT);                         //Структура настройки канала. 
  DAC_ST_ADC[7].NULL = (uint32_t)0;                                                  //Первичная струтура.
  RST_CLK->PER_CLOCK|=PCLK_EN_DMA;                                                   //Включаем тактирование DMA.
  DMA->CTRL_BASE_PTR = (uint32_t)&DAC_ST_ADC;                                        //Указываем адрес массива структур. 
  DMA->CFG = CFG_master_enable;                                                      //Разрешаем работу DMA.
} 

void Init_SysTick (void)                          
{
   SysTick->LOAD = 80000000/261.63/100-1;                 
   SysTick->CTRL |= CLKSOURCE|TCKINT|ENABLE;
}

volatile uint16_t Loop = 0;
volatile uint32_t Delay_dec = 0; 
void SysTick_Handler (void)
{
	if ((DAC_ST_ADC[7].channel_cfg & (0x3FF<<4)) == 0) {
	DAC_ST_ADC[7].channel_cfg = (uint32_t)(ST_DMA_DAC_STRYKT);}      //Перенастраиваем DMA.
  DMA->CHNL_ENABLE_SET   = 1<<8;                                   //Разрешаем работу канала DMA 8.
  DMA->CHNL_SW_REQUEST   = 1<<8;                                   //Запускаем цикл ДМА.
}

int main (void)
{
  HSE_Clock_ON();                                  //Разрешаем использование HSE генератора. 
  HSE_Clock_OffPLL();                              //Настраиваем "путь" сигнала и включаем тактирование от HSE генератора.
  Buzzer_out_DAC_init();                           //Настраиваем порт для ЦАП.
  DAC_Init();                                      //Настраиваем ЦАП.
  HSE_PLL(10);                                     //8 Мгц -> 80 Мгц. 
  
	DMA_and_DAC();
	Init_SysTick();                                  //Инициализируем системный таймер для прерываний.
  while (1) { 

}
	}

	