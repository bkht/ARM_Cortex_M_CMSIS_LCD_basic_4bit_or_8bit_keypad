#include "stm32f4xx.h"                  // Device header
#include <stdio.h>

// Display used: 
// Newhaven 4x20 character OLED (green)
// NHD-0420DZW-AG5_Character_OLED_Display_Module
// Note, command delays according to datasheet (600uS, 2mS)
// PORTE
// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// D7 D6 D5 D4 D3 D2 D1 D0 RW RS  E
// PORTD KEYPAD, LEDs
// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// LB LR LO LG C3 C2 C1 C0       LE    R3 R2 R1 R0

// Use the display in 4-bit mode, otherwise use it in 8-bit mode
#define DISPLAY_4_BIT_MODE
#define DISPLAY_SIZE_X 20
#define DISPLAY_SIZE_Y  4

// Keypad connections
// Top view, bottom 1-8 from left to right:
//  1   2   3   4   5   6   7   8
// C0  C1  C2  C3  R0  R1  R2  R3
// D8  D9  D10 D11 D0  D1  D2  D3
// IN  IN  IN  IN  OUT OUT OUT OUT
// Keys:
// C0 C1 C2 C3
//  1  2  3  A  R0
//  4  5  6  B  R1
//  7  8  9  C  R2
//  *  0  #  D  R3

#define KEYPAD GPIOD
#define K_R0  (1U << 0)
#define K_R1  (1U << 1)
#define K_R2  (1U << 2)
#define K_R3  (1U << 3)
#define K_C0  (1U << 8)
#define K_C1  (1U << 9)
#define K_C2  (1U << 10)
#define K_C3  (1U << 11)

#define LED_LD8_R (1U<<5)
#define LED_LD4_G (1U<<12)
#define LED_LD3_O (1U<<13)
#define LED_LD5_R (1U<<14)
#define LED_LD6_B (1U<<15)
#define USER_BUTTON (1U<<0)

#define LCKK (1U<<16)

#define HD44780_E  (1U << 5)
#define HD44780_RS (1U << 6)
#define HD44780_RW (1U << 7)
#define HD44780_D4 (1U << 12)
#define HD44780_D5 (1U << 13)
#define HD44780_D6 (1U << 14)
#define HD44780_D7 (1U << 15)

#define DATA_REGISTER 1
#define COMMAND_REGISTER 0

void LCD_Init(void);

void LCD_Cmd(uint8_t command);
void LCD_Data(uint8_t data);
void LCD_Write(uint8_t data, uint8_t control);

void LCD_GotoXY(uint8_t x, uint8_t y);
void LCD_Char(uint8_t character);
void LCD_String(char *str);
void LCD_Increment_Cursor_Position(void);
void LCD_Clear(void);

uint8_t getKey(void);

void GPIO_Init(void);

void delay_ms(uint16_t n);
void delay_us(uint32_t n);

typedef struct cursor_position { 
    uint8_t x; 
    uint8_t y; 
} cursor_position;

cursor_position cp;

int main()
{
	uint8_t key = 0;
	uint8_t key_old = 0;
	
	GPIO_Init();
	
	LCD_Init();

	GPIOD->BSRR = (LED_LD5_R | LED_LD4_G | LED_LD6_B);
	delay_ms(1000);
	GPIOD->BSRR = ((LED_LD5_R | LED_LD4_G | LED_LD6_B) << 16);

	LCD_GotoXY(0,0);
	LCD_String("Hello");
	
	LCD_GotoXY(3,1);
	LCD_String("Display");

	LCD_GotoXY(6,2);
	LCD_String("in");
	
	LCD_GotoXY(9,3);
#ifdef DISPLAY_4_BIT_MODE
	LCD_String("4-bit mode");
#else
	LCD_String("8-bit mode");
#endif
	LCD_GotoXY(0,0);
	while (1) {
		key = getKey();
		if ((key != 0) && (key_old == 0)) {
			//LCD_GotoXY(15,0);
			LCD_Char(key);
		}
		key_old = key;
		
	}
}

void LCD_GotoXY(uint8_t x, uint8_t y)
{
	uint8_t address = 0;
	cp.x = x;
	cp.y = y;
	
	switch (y) {
		case 1:
			address = 0x40 + x;	// Line 1: 0x40, 0x41, 0x42 .. 0x53
			break;
		case 2:
			address = 0x14 + x;	// Line 2: 0x14, 0x15, 0x16 .. 0x27
			break;
		case 3:
			address = 0x54 + x;	// Line 3: 0x54, 0x55, 0x56 .. 0x67
			break;
		default:
			address = 0x00 + x;	// Line 0: 0x00, 0x01, 0x02 .. 0x13
	}
	LCD_Cmd(address | 0x80);
}

void LCD_Char(uint8_t character)
{
	LCD_Data(character);
	LCD_Increment_Cursor_Position();
}

void LCD_String(char *str)
{
	while(*str) {
		LCD_Data(*str++);
		LCD_Increment_Cursor_Position();
	}
}

void LCD_Increment_Cursor_Position(void)
{
	cp.x++;
	if (cp.x == DISPLAY_SIZE_X) {
		cp.x = 0;
		cp.y++;
		LCD_GotoXY(cp.x, cp.y);
		if (cp.y == DISPLAY_SIZE_Y) {
			cp.y = 0;
			LCD_GotoXY(cp.x, cp.y);
		}
	}
}

void LCD_Clear(void)
{
	LCD_Cmd(0x01);	// Display clear
	LCD_Cmd(0x80);	// Set DDRAM address to 0
	cp.x = 0;
	cp.y = 0;
}

void LCD_Init(void)
{
	// Enable clock to GPIO Port
	// Set GPIO Pins as output pins
	// Enable GPIO pins
	// Function set command 0x28 = 4-bit, 2 display lines, 5x7 font
	// Entry mode set command 0x06 incremenet automatically
	// Display control 0x0F Turn on display, cursor blinking
	// Display control 0x01 Clear display

	// Wait for power stabilization: >= 1ms
	delay_ms(2);
	
#ifdef DISPLAY_4_BIT_MODE
	// Use the display in 4-bit mode
	// Accoring to the datasheet, write this preceeding nibble to the display
	LCD_Write(0x20, COMMAND_REGISTER);
	
    // Function set command - 0x38 = 4-bit, 2 lines, 5x7 font
	LCD_Cmd(0x28);	// 00111000
#else
	// Use the display in 8-bit mode
    // Function set command - 0x38 = 8-bit, 2 lines, 5x7 font
	LCD_Cmd(0x38);	// 00111000
#endif

    // Display off
	LCD_Cmd(0x08);	// 00001000
	
    // Display clear
	LCD_Cmd(0x01);	// 00000001
	
    // Function mode set command - 0x06 = increment cursor automatically
	LCD_Cmd(0x06);	// 00000110
	
	LCD_Cmd(0x02);	// 00000010 Home Command

	// Display control - 0x0F = turn on display, cursor blinking
	LCD_Cmd(0x0F);	// 00001111
	
	LCD_Cmd(0x80);	// 10000000 Set DDRAM address to 0

	LCD_Clear();
}

void LCD_Cmd(uint8_t command) {
#ifdef DISPLAY_4_BIT_MODE
	// Use the display in 4-bit mode
	// Write upper 4-bits of command RS = 0
	// Write lower 4-bits of command
	// Secure command, E=1 for a brief moment
	// Delay, allow the display to catch-up with the MCU
	LCD_Write(command & 0xF0, COMMAND_REGISTER); 	// Command high nibble, RS=0
	LCD_Write(command << 4, COMMAND_REGISTER);	// Command low nibble, RS=0
    // Delay - Allow the display to catch-up with the MCU
#else
	// Use the display in 8-bit mode
    // Select Command Register - RS = 0
	LCD_Write(command, COMMAND_REGISTER); 	// Data high nibble, RS=1
#endif
	if (command <= 1) {
		delay_ms(2);
	} else {
		delay_us(600);
	}
}

void LCD_Data(uint8_t data) {
#ifdef DISPLAY_4_BIT_MODE
	// Use the display in 4-bit mode
	// Write upper 4-bits of command RS = 1
	// Write lower 4-bits of command
	// Secure command, E=1 for a brief moment
	// Delay, allow the display to catch-up with the MCU
	LCD_Write(data & 0xF0, DATA_REGISTER); 	// Data high nibble, RS=1
	LCD_Write(data << 4, DATA_REGISTER);	// Data low nibble, RS=1
    // Delay - Allow the display to catch-up with the MCU
#else
	// Use the display in 8-bit mode
    // Select Data Register - RS = 1
	LCD_Write(data, DATA_REGISTER); 	// Data high nibble, RS=1
#endif
	delay_us(4);
}

void LCD_Write(uint8_t data, uint8_t control) {
	// Select Command Register - RS=0, RW=0, E=0
	GPIOE->BSRR = ((HD44780_RW | HD44780_RS | HD44780_E) << 16); // RS=0, RW=0, E=0
	if (control == DATA_REGISTER) {
		// Select Data Register - RS=1, RW=0, E=0
		GPIOE->BSRR = HD44780_RS; // RS=1
	}

#ifdef DISPLAY_4_BIT_MODE
// Use the display in 4-bit mode
// data                    control
// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// D7 D6 D5 D4 -- -- -- -- RW RS  E
    // Write data
	// Set up data bits 4-7 on GPIOE12-15
//	GPIOE->ODR &= 0x0fff;
//	GPIOE->ODR |= ((data & 0xF0) << 8);
	GPIOE->BSRR = 0xf0000000;	// Reset bits 12-15
	GPIOE->BSRR = ((data & 0xF0) << 8);		// Extract upper 4bits from data and set bit 12-15 accordingly
#else
// Use the display in 8-bit mode
// data                    control
// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
// D7 D6 D5 D4 D3 D2 D1 D0 RW RS  E
    // Write data
	// Set up data bits 0-7 on GPIOE8-15
//	GPIOE->ODR &= 0x00ff;
//	GPIOE->ODR |= (data << 8);
	GPIOE->BSRR = 0xff000000;	// Reset bits 8-15
	GPIOE->BSRR = (data << 8);		// Set bit 12-15 according to data
#endif
    // Secure command - E = 1 for a brief moment (20uS)
	GPIOE->BSRR = HD44780_E; // E=1, to secure command
	delay_us(2);
	GPIOE->BSRR = (HD44780_E << 16); // E=0

    // Set back to command register - RS = 0
//	GPIOE->BSRR = (HD44780_RS << 16); // reset RS=0
}

uint8_t getKey(void) {
	// Keypad connections
	// Top view, bottom 1-8 from left to right:
	//  1   2   3   4   5   6   7   8
	// C0  C1  C2  C3  R0  R1  R2  R3
	// D8  D9  D10 D11 D0  D1  D2  D3
	// IN  IN  IN  IN  OUT OUT OUT OUT
	// Keys:
	// C0 C1 C2 C3
	//  1  2  3  A  R0
	//  4  5  6  B  R1
	//  7  8  9  C  R2
	//  *  0  #  D  R3
	// GPIOD 0000 CCCC 0000 RRRR
	//            3210      3210
	const uint8_t keymap[4][4]={
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
	};
	uint8_t k_row, k_col;
	// Clear bits for R0-R3=0
	GPIOD->BSRR = ((K_R0 | K_R1 | K_R2 | K_R3) << 16);
	k_col = (GPIOD->IDR & 0x0f00) >> 8;	// 0000 CCCC 0000 RRRR
	if (k_col == 0x0f) {
		return 0;	// No key pressed
	}
	while (1) {
		k_row = 0;
		GPIOD->BSRR = (K_R0 | K_R1 | K_R2 | K_R3); // R0-R3=1
		GPIOD->BSRR = (K_R0 << 16);	// R0=0
		delay_ms(10);
		k_col = (GPIOD->IDR & 0x0f00) >> 8;	// 0000 CCCC 0000 RRRR
		if (k_col != 0x0f) {
			break;
		}
		k_row = 1;
		GPIOD->BSRR = (K_R0 | K_R1 | K_R2 | K_R3); // R0-R3=1
		GPIOD->BSRR = (K_R1 << 16);	// R1=0
		delay_ms(10);
		k_col = (GPIOD->IDR & 0x0f00) >> 8;	// 0000 CCCC 0000 RRRR
		if (k_col != 0x0f) {
			break;
		}
		k_row = 2;
		GPIOD->BSRR = (K_R0 | K_R1 | K_R2 | K_R3); // R0-R3=1
		GPIOD->BSRR = (K_R2 << 16);	// R1=0
		delay_ms(10);
		k_col = (GPIOD->IDR & 0x0f00) >> 8;	// 0000 CCCC 0000 RRRR
		if (k_col != 0x0f) {
			break;
		}
		k_row = 3;
		GPIOD->BSRR = (K_R0 | K_R1 | K_R2 | K_R3); // R0-R3=1
		GPIOD->BSRR = (K_R3 << 16);	// R1=0
		delay_ms(10);
		k_col = (GPIOD->IDR & 0x0f00) >> 8;	// 0000 CCCC 0000 RRRR
		if (k_col != 0x0f) {
			break;
		}
		return 0;	// No key pressed (not expected)
	}
	if (k_col == 0x0E) {	// 1110
		return keymap[k_row][0];
	}
	if (k_col == 0x0D) {	// 1101
		return keymap[k_row][1];
	}
	if (k_col == 0x0B) {	// 1011
		return keymap[k_row][2];
	}
	if (k_col == 0x07) {	// 0111
		return keymap[k_row][3];
	}
	return 0;
}

void GPIO_Init(void) {
	// PORTA User Button
    // Enable clock to GPIO Port
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;            // enable clock for GPIOA (button)

	// These bits are written by software to configure the I/O direction mode.
	// 00: Input (reset state)
	// 01: General purpose output mode
	// All inputs
	// PORTA User Button is input
	// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	//                                              UB
	// 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
	//   0     0     0     0     0     0     0     0
//	GPIOA->MODER = 0x00000000;
//	GPIOA->OSPEEDR = 0;
	//	These bits are written by software to configure the output type of the I/O port.
	//	0: Output push-pull (reset state)
	//	1: Output open-drain
//	GPIOA->OTYPER = 0;
//	GPIOA->PUPDR = 0;

	// Now, lock GPIOA configuration
//	GPIOA->LCKR = (USER_BUTTON | LCKK);
//	GPIOA->LCKR = (USER_BUTTON);
//	GPIOA->LCKR = (USER_BUTTON | LCKK);
//	uint32_t lockedA = GPIOA->LCKR;
//	lockedA = GPIOA->LCKR;	// 2nd read needed, as the LCKK bit is updated after the first read

	// PORTD KEYPAD + LEDs
    // Enable clock to GPIO Port
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;            // enable clock for GPIOD (LEDs)
	
	// These bits are written by software to configure the I/O direction mode.
	// 00: Input (reset state)
	// 01: General purpose output mode
	// PORTD KEYPAD + LEDs: LEDs are outputs, Keypad Colums are inputs, Keypad Rows are outputs
	// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	// LB LR LO LG C3 C2 C1 C0       LE    R3 R2 R1 R0
	// 01 01 01 01 00 00 00 00 00 00 01 00 01 01 01 01
	//   5     5     0     0     0     4     5     5
	GPIOD->MODER = 0x55000455;
	GPIOD->OSPEEDR = 0;
	//	These bits are written by software to configure the output type of the I/O port.
	//	0: Output push-pull (reset state)
	//	1: Output open-drain
	// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	// LB LR LO LG C3 C2 C1 C0       LE    R3 R2 R1 R0
	//  0  0  0  0  0  0  0  0  0  0  0  0  1  1  1  1
	//       0           0           0           f
	GPIOD->OTYPER = 0x000f;
	//	These bits are written by software to configure the I/O pull-up or pull-down
	//	00: No pull-up, pull-down
	//	01: Pull-up
	//	10: Pull-down	
	// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	// LB LR LO LG C3 C2 C1 C0       LE    R3 R2 R1 R0
	// 00 00 00 00 01 01 01 01 00 00 00 00 00 00 00 00
	//   0     0     5     5     0     0     0     0
	GPIOD->PUPDR = 0x00550000;

	// Now, lock GPIOD configuration
//	GPIOD->LCKR = (LED_LD5_R | LED_LD4_G | LED_LD6_B | LED_LD3_O | LED_LD8_R | LCKK);
//	GPIOD->LCKR = (LED_LD5_R | LED_LD4_G | LED_LD6_B | LED_LD3_O | LED_LD8_R);
//	GPIOD->LCKR = (LED_LD5_R | LED_LD4_G | LED_LD6_B | LED_LD3_O | LED_LD8_R | LCKK);
//	uint32_t lockedD = GPIOD->LCKR;
//	lockedD = GPIOD->LCKR;	// 2nd read needed, as the LCKK bit is updated after the first read

	// PORTE LCD
    // Enable clock to GPIO Port
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;            // enable clock for GPIOE (HD44780)

#ifdef DISPLAY_4_BIT_MODE
	// Discovery STM32F4
	// Use the display in 4-bit mode
	// These bits are written by software to configure the I/O direction mode.
	// 00: Input (reset state)
	// 01: General purpose output mode
	// PORTE display
	// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	// D7 D6 D5 D4 -- -- -- -- RW RS  E
	// 01 01 01 01 00 00 00 00 01 01 01 00 00 00 00 00
	//   5     5     0     0     5     4     0     0
	GPIOE->MODER = 0x55005400;
#else
	// Discovery STM32F4
	// Use the display in 8-bit mode
	// These bits are written by software to configure the I/O direction mode.
	// 00: Input (reset state)
	// 01: General purpose output mode
	// PORTE display
	// 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
	// D7 D6 D5 D4 D5 D6 D7 D8 RW RS  E
	// 01 01 01 01 01 01 01 01 01 01 01 00 00 00 00 00
	//   5     5     5     5     5     4     0     0
	GPIOE->MODER = 0x55555400;
#endif
	GPIOE->OSPEEDR = 0;
	//	These bits are written by software to configure the output type of the I/O port.
	//	0: Output push-pull (reset state)
	//	1: Output open-drain
	GPIOE->OTYPER = 0;
	GPIOE->PUPDR = 0;

	// Now, lock GPIOE configuration
	// 0xffe0 11111111 11100000
//	GPIOE->LCKR = 0xffe0 | LCKK;
//	GPIOE->LCKR = 0xffe0;
//	GPIOE->LCKR = 0xffe0 | LCKK;
//	uint32_t lockedE = GPIOE->LCKR;
//	lockedE = GPIOE->LCKR;	// 2nd read needed, as the LCKK bit is updated after the first read

//	// Indicate GPIOs are locked
//	if ((lockedA & LCKK) && (lockedD & LCKK) && (lockedE & LCKK)) {
//		GPIOD->BSRR = LED_LD3_O;	// Orange LED indicates successful lock
//	}
}

void delay_ms(uint16_t duration) {
	while(duration-- > 0) {
		delay_us(1000);
	}
}

void delay_us(uint32_t duration) {
	duration *= 3;
	while(duration-- > 0) {
	}
}
