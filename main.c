#include "stm32f10x.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>	

uint8_t led_status = 0;
uint8_t button_state = 0;
uint8_t flash_freq = 0;
uint8_t status = 1;
uint8_t resetS = 0;

const char* bearings[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};

#define slAddr 0x0D
#define PI acos(-1.0)

uint8_t buffer[6];
int16_t x,y,z;
float x_offset, y_offset, z_offset;
float x_scale, y_scale, z_scale;

void SystemClock_Config(void);
void DWT_Init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void gpio_init(void); 
//void uart_init(void);

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t saddr, uint8_t maddr, uint8_t data);
uint8_t i2c_read(uint8_t saddr,uint8_t maddr, uint8_t *data, uint8_t length);

void qmc5883_init(void);
void qmc5883_calibrate(void);
int qmc5883_azimuth(void);
int qmc5883_bearing(int azimuth);
void qmc5883_direction(char* myArray, int azimuth);

void EXTI_Init(void);
void Timer2_Init(void);
void EXTI15_10_IRQHandler(void);
void TIM2_IRQHandler(void);

void send8BitLCD(char D);
void sendCMD2LCD(char cmd);
void sendChar2LCD(char Char);
void sendString2LCD(char *str);
void lcd_init(void);

/*void uartSend(uint8_t *data, uint8_t size){
	for (uint8_t i = 0; i < size; i++) {
		USART1->DR = *(data + i);
    while (!(USART1->SR & USART_SR_TXE));  
  }
}*/

int main(void){
	char str[100];
  DWT_Init();
  SystemClock_Config();
  gpio_init();
	EXTI_Init();
  Timer2_Init();
  //uart_init();
	lcd_init();
  i2c_init();
  qmc5883_init();
	sendString2LCD("Compass Setup");
	delay_ms(3);
	qmc5883_calibrate(); //calibrate
	sendCMD2LCD(0x01);
	delay_ms(3);
  while (1) {
		sendCMD2LCD(0x01);//clear lcd
		delay_ms(3);
		if(status ==1){ //Check status of process
		char azi[100] = {0};
		//read data from qmc5883l
		i2c_read(slAddr, 0x00, buffer, 6);
    x = (((int16_t)buffer[1]) << 8) | buffer[0];
    y = (((int16_t)buffer[3]) << 8) | buffer[2];
    z = (((int16_t)buffer[5]) << 8) | buffer[4];
		//x,y,z after calibrate
		x = (x - x_offset) * x_scale;
		y = (y - y_offset) * y_scale;
		z = (z - z_offset) * z_scale;
		int heading = ((int)(atan2(y,x)*180/ PI) % 360);
		if(heading < 0) heading +=360;
		qmc5883_direction(azi,heading);  
		sprintf(str,"%d",heading);
		strcat(azi,str);
		sendString2LCD(azi);
		delay_ms(3);
    sprintf(str, "%d %d %d\n\r", x, y, z);
    delay_ms(500);
		}		else{
			sendString2LCD("Pause");
			delay_ms(3);
		}
  }
  return 1;
}

void SystemClock_Config(void) {
  RCC->CR |= RCC_CR_HSION; //Enable HSI clock
  while (!(RCC->CR & RCC_CR_HSIRDY)); //Wait HSI on
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; //Enable AFIO
  RCC->APB2ENR |= (1<<2) | (1<<4) | (1<<3); //Enable clock GPIOA,GPIOB,GPIOC.
}

void gpio_init(void) {
	
/*  // UART1 TX (PA9)
  GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
  GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1);
  // UART1 RX (PA10)
  GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
  GPIOA->CRH |= (GPIO_CRH_CNF10_0);
*/
	
	//GPIO Button and Led
  GPIOB->CRH &= ~(3UL << 10U); //reset CNFPB10
  GPIOB->CRH |= (2UL << 10U); /* Input mode with pull-up/ pull-down for PB10 */
  GPIOB->ODR |= (1UL << 10U); /* pull-up for PB10 */
	
	GPIOB->CRH &= ~(3UL << 22U); //reset CNFPB10
  GPIOB->CRH |= (2UL << 22U); /* Input mode with pull-up/ pull-down for PB13 */
  GPIOB->ODR |= (1UL << 13U); /* pull-up for PB13 */
	
	GPIOB->CRL |= 3UL << 0U; /* MODE for PB0 */
  GPIOB->CRL &= ~(3UL << 2U); /* CNF for PB0 */
	GPIOB->BRR |= 1UL << 0U; /* Reset PB0 */

  GPIOB->CRL |= 3UL << 4U; /* MODE for PB1 */
  GPIOB->CRL &= ~(3UL << 6U); /* CNF for PB1 */
  GPIOB->BRR |= 1UL << 1U; /* Reset PB1 */
	
	
	// LCD Pins Init (GPIOA Pins 0-7, GPIOC Pins 14-15)
  // GPIOA: Data, GPIOC Pin 14 RS, GPIO Pin 15 E
	GPIOA->ODR &= 0;
  GPIOC->ODR &= 0;
	
	GPIOA->CRL &= ~(0xFFFFFFFF);   
	GPIOA->CRL |= 0x33333333;     
	GPIOA->ODR &= ~(0xFF); 
  GPIOC->CRH &= ~((0xF<<24)|(0xF)<<28);
	GPIOC->CRH |= (0x2 << 24) | (0x2 << 28);
}

//Init external interupt
void EXTI_Init(void)
{
  AFIO->EXTICR[2] |= (1 << 8); /* Map PB10 to EXTI10 */
	AFIO->EXTICR[3] |= (1<<4); /* Map PB13 to EXTI10 */
  EXTI->IMR |= (1 << 10) | (1<<13); /* Interrupt request from Line 10,13 is not masked */
  EXTI->EMR |= (1 << 10) | (1<<13); /* Event request from Line 10,13 is not masked */
  EXTI->FTSR |= (1 << 10) | (1<<13); /* Falling trigger enabled (for Event and Interrupt) for input line */
  __NVIC_SetPriority(EXTI15_10_IRQn, 0); /* Set priority for EXTI15_10 */
  __NVIC_EnableIRQ(EXTI15_10_IRQn); /* Enable EXTI15_10 interrupt */
}

//Init timer
void Timer2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock 
    TIM2->PSC = 8000 - 1; // Prescaler value
    TIM2->ARR = 8000 - 1; // Auto-reload value 
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt 
    TIM2->CR1 |= TIM_CR1_CEN; // Enable timer
    NVIC_SetPriority(TIM2_IRQn, 1); // Set interrupt priority 
    NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt 
}

void EXTI15_10_IRQHandler(void){
    if (EXTI->PR & (1UL << 10U)) { //Check button PB10
        delay_ms(50); // Simple debounce delay
        if (!(GPIOB->IDR & (1UL << 10U))) { // Check if the button is still pressed
            button_state = !button_state; // Toggle button state
						status = !status; //Toggle status process
					
        }
        EXTI->PR |= 1UL << 10U; /* Clear pending interrupt */
    }
		if (EXTI->PR & (1UL << 13U)) { //Check button PB13
        delay_ms(50); // Simple debounce delay
        if (!(GPIOB->IDR & (1UL << 13U))) { // Check if the button is still pressed
					NVIC_SystemReset();
        }
        EXTI->PR |= 1UL << 13U; /* Clear pending interrupt */
    }
}

void TIM2_IRQHandler(void) 
{
    if (TIM2->SR & TIM_SR_UIF) { //Check Timer Interupt
        TIM2->SR &= ~TIM_SR_UIF;  //Clear Timer Interupt pending
        
        if (button_state){
            if (led_status){
                led_status = 0;
                GPIOB->BRR |= 1UL << 1U; // Reset PC15
								GPIOB->BRR |= 1UL << 0U; // Reset PB0  
            } 
            else {  
                led_status = 1; 
                GPIOB->BSRR |= 1UL << 1U; // Set PC15
								GPIOB->BRR |= 1UL << 0U; // Reset PB0  
            }
        }
        else {
            if (led_status) {
                led_status = 0;
                GPIOB->BRR |= 1UL << 0U; // Reset PB0  
								GPIOB->BRR |= 1UL << 1U; // Reset PC15
            } 
            else {
                led_status = 1;
                GPIOB->BSRR |= 1UL << 0U; // Set PB0
								GPIOB->BRR |= 1UL << 1U; // Reset PC15
            }
            GPIOC->BRR |= 1UL << 15U; // Reset PC15
        }
        
        // Adjust flashing frequency based on button state
        if (button_state)
            TIM2->ARR = 2000 - 1; // 2 Hz (0.5 second period)
        else
            TIM2->ARR = 4000 - 1; // 4 Hz (0.25 second period)
    }
}


/*void uart_init(void) {
  USART1->CR1 |= USART_CR1_TE | USART_CR1_RE |USART_CR1_RXNEIE| USART_CR1_UE;
  USART1->BRR = (uint16_t) 0x1D4C;  
}*/

void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
}

void delay_ms(uint32_t ms) {
  uint32_t start = DWT->CYCCNT;
  uint32_t count = ms * (SystemCoreClock / 1000);
  while ((DWT->CYCCNT - start) < count);
}

void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t count = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start) < count);
}

void i2c_init(void){
  RCC->APB1ENR |= (1<<21); //Enable clock I2C
	
	GPIOB->CRL |= ((0xB << 24) | (0xB << 28)); // Set Mode, CNF PB6 PB7
  // Cau hình I2C1
  I2C1->CR2 |= 36; 
  I2C1->CCR |= 180; // 100kHz
  I2C1->TRISE = 37; // 1000ns / (1/36MHz) + 1
  I2C1->CR1 |= I2C_CR1_PE; // Enable I2C1
}

void i2c_start(void){
	I2C1->CR1 |= I2C_CR1_START; //Set start bit
	while (!(I2C1->SR1 & I2C_SR1_SB));// wait bit start is generated
}

void i2c_stop(void){
	I2C1->CR1 |= I2C_CR1_STOP; // set stop bit
}

uint8_t i2c_write(uint8_t saddr, uint8_t maddr, uint8_t data) {
	while (I2C1->SR2 & I2C_SR2_BUSY); //Check busy and i2c ready 
	i2c_start(); //start
  I2C1->DR = saddr << 1; //send slave address and write bit 
  while (!(I2C1->SR1 & I2C_SR1_ADDR)); //wait send
  (void)I2C1->SR2;  
  while (!(I2C1->SR1 & I2C_SR1_TXE)); //wait data res empty
	I2C1->DR = maddr; //send memory slave address
  while (!(I2C1->SR1 & I2C_SR1_TXE)); 
  I2C1->DR = data; //send data
  while (!(I2C1->SR1 & I2C_SR1_BTF)); // wait tranfer finish
  i2c_stop(); //stop
	return 0;
}

uint8_t i2c_read(uint8_t saddr,uint8_t maddr, uint8_t *data, uint8_t length){
	
	while (I2C1->SR2 & I2C_SR2_BUSY);
	i2c_start();
	I2C1->DR=saddr<<1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2;
	while(!(I2C1->SR1&I2C_SR1_TXE));
	I2C1->DR = maddr;
	while(!(I2C1->SR1&I2C_SR1_TXE));
	i2c_start();
	I2C1->DR=saddr<<1|1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2;
	I2C1->CR1|=I2C_CR1_ACK; //enable ack
	while(length>0U)
	{
		if(length==1U)
		{
			I2C1->CR1&=~I2C_CR1_ACK; //not ack
			I2C1->CR1|=I2C_CR1_STOP; //stop
			while(!(I2C1->SR1&I2C_SR1_RXNE));//wait receive buffer not empty
			*data++=I2C1->DR;//read data
			break;
		}
		else
		{
			while(!(I2C1->SR1&I2C_SR1_RXNE));
			(*data++)=I2C1->DR;
			length--;
		}
	}
	return 0;
}

void qmc5883_init(void){
	i2c_write(slAddr, 0x0A, 0x80);
	i2c_write(slAddr, 0x09, 0x1D);
	i2c_write(slAddr, 0x0A, 0x41);
	i2c_write(slAddr, 0x0B, 0x01);
}

int qmc5883_azimuth(void){
	float heading = atan2(y,x)*180/ PI;
	return (int)heading % 360;
}

int qmc5883_bearing(int azimuth){
	int index = ((azimuth ) / 45) % 8;
	return index;
}

void qmc5883_direction(char* myArray, int azimuth){
	int directionIndex = qmc5883_bearing(azimuth);
  strcpy(myArray, bearings[directionIndex]);
}

void qmc5883_calibrate(void) {
			char tmp[100]= "SS";

	int16_t x_min = 32767, y_min = 32767, z_min = 32767;
	int16_t x_max = -32767, y_max = -32767, z_max = -32767;
	for (uint16_t i = 0; i < 500; i++) {
    i2c_read(slAddr, 0x00, buffer, 6);
    x = (((int16_t)buffer[1]) << 8) | buffer[0];
    y = (((int16_t)buffer[3]) << 8) | buffer[2];
    z = (((int16_t)buffer[5]) << 8) | buffer[4];
		sprintf(tmp,"%d %d %d\n\r", x,y,z);
    if (x < x_min) x_min = x; 
    if (y < y_min) y_min = y;
    if (z < z_min) z_min = z;
    if (x > x_max) x_max = x;
    if (y > y_max) y_max = y;
    if (z > z_max) z_max = z;
		
    delay_ms(20);
  }

	sprintf(tmp,"\n\r%d %d %d %d %d %d\n\r", x_max, y_max, z_max, x_min, y_min, z_min);
	delay_ms(1000);
	
  x_offset = x_max/2 + x_min/ 2;
  y_offset = y_max/2 + y_min/ 2;
  z_offset = z_max/2 + z_min/ 2;

  x_scale = (float)((x_max/2) - (float) (x_min/2));
  y_scale = (float)((y_max/2) - (float)(y_min/2));
  z_scale = (float)((z_max/2) - (float) (z_min/2));

  float avg_scale = (x_scale + y_scale + z_scale) / 3;

  x_scale = avg_scale / x_scale;
  y_scale = avg_scale / y_scale;
   z_scale = avg_scale / z_scale;
}

// LCD
void send8BitLCD(char D) {
	GPIOA->BSRR = ((0xFF) << 16);
	GPIOA->BSRR = (D & 0xFF); 
	delay_us(1);
}

void sendCMD2LCD(char cmd) {
	GPIOC->ODR &= ~(1<<14);
  send8BitLCD(cmd);
  GPIOC->ODR &= ~(1<<15);
	GPIOC->ODR |= (1<<15);
	delay_us(1);
  GPIOC->ODR &= ~(1<<15);
  delay_us(1000);
}

void sendChar2LCD(char Char) {
  GPIOC->ODR |= (1<<14);
  send8BitLCD(Char);
  GPIOC->ODR &= ~(1<<15);
	GPIOC->ODR |= (1<<15);
	delay_us(1);
  GPIOC->ODR &= ~(1<<15);
  delay_us(1000);
}

void sendString2LCD(char *str) {
    for (int i = 0; str[i] != '\0'; i++) {
        sendChar2LCD(str[i]);
    }
}

void lcd_init(void) {   
		delay_ms(15); 
    sendCMD2LCD(0x30); 
    delay_ms(3); 
    sendCMD2LCD(0x30); 
    delay_ms(3);  
    sendCMD2LCD(0x30); 
    delay_ms(3); 
    sendCMD2LCD(0x38); 
    delay_ms(3); 
    sendCMD2LCD(0x08);
    delay_ms(3); 
    sendCMD2LCD(0x01);
    delay_ms(3); 
    sendCMD2LCD(0x06);
    delay_ms(3); 
    sendCMD2LCD(0x0C); 
    delay_ms(3); 
}
