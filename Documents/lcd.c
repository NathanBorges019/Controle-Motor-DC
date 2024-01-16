/*******************************************************************************
 * @file lcd.c
 * @brief This module is responsible to control a LCD 16x2 Display.
 * @author Gustavo Adono
 *******************************************************************************
 *
 *     _     ____ ____      _  __ __  ______
 *    | |   / ___|  _ \    / |/ /_\ \/ |___ \
 *    | |  | |   | | | |   | | '_ \\  /  __) |
 *    | |__| |___| |_| |   | | (_) /  \ / __/
 *    |_____\____|____/    |_|\___/_/\_|_____|
 *
 *
 *     _  _   ____ ___ _____ ____
 *    | || | | __ |_ _|_   _/ ___|
 *    | || |_|  _ \| |  | | \___ \
 *    |__   _| |_) | |  | |  ___) |
 *       |_| |____|___| |_| |____/
 *
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "lcd.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


 /*******************************************************************************
 *  @brief This function positions the cursor at a certain position on the display,
 *  and writes a string oh this position.
 *
 *	@param1 Line of the display.
 *	@param2 Column of the display.
 *
 *  @return None
 *
 ******************************************************************************/
void Lcd_Out(uint8_t x, uint8_t y,char *string)
{
	uint8_t temp;

	if (x == 1)
		temp = 0x80;
	else
		temp = 0xC0;

	temp = (temp) + (y-1);

	LCD_Cmd(temp);

	while (*string)
	{
		LCD_Data(*string);
		string++;
	}


}

 /*******************************************************************************
 *  @brief This function positions the cursor at a certain position on the display.
 *
 *	@param1 Line of the display.
 *	@param2 Column of the display.
 *
 *  @return None
 *
 ******************************************************************************/

void LCD_Cursor(uint8_t x, uint8_t y)
{
	uint8_t temp;

	if (x == 0)
		temp = 0x80;
	else
		temp = 0xC0;

	temp = temp + y;

	LCD_Cmd(temp);


}


/*******************************************************************************
*  @brief Send a character to the display.
*
*  @param1 Data to be sended.
*
*  @return  None
*
******************************************************************************/
void LCD_Data(uint8_t data)
{
	RS_GPIO_Port->BSRR = RS_Pin;
	if (data & 0x80)
		D7_GPIO_Port->BSRR = D7_Pin;
	else
		D7_GPIO_Port->BSRR = (uint32_t) D7_Pin << 16u;
	if (data & 0x40)
		D6_GPIO_Port->BSRR = D6_Pin;
	else
		D6_GPIO_Port->BSRR = (uint32_t) D6_Pin << 16u;
	if (data & 0x20)
		D5_GPIO_Port->BSRR = D5_Pin;
	else
		D5_GPIO_Port->BSRR = (uint32_t) D5_Pin << 16u;
	if (data & 0x10)
		D4_GPIO_Port->BSRR = D4_Pin;
	else
		D4_GPIO_Port->BSRR = (uint32_t) D4_Pin << 16u;

	EN_GPIO_Port->BSRR = EN_Pin;
	HAL_Delay(1);
	EN_GPIO_Port->BSRR = (uint32_t) EN_Pin << 16u;
	HAL_Delay(1);

	if (data & 0x08)
		D7_GPIO_Port->BSRR = D7_Pin;
	else
		D7_GPIO_Port->BSRR = (uint32_t) D7_Pin << 16u;
	if (data & 0x04)
		D6_GPIO_Port->BSRR = D6_Pin;
	else
		D6_GPIO_Port->BSRR = (uint32_t) D6_Pin << 16u;
	if (data & 0x02)
		D5_GPIO_Port->BSRR = D5_Pin;
	else
		D5_GPIO_Port->BSRR = (uint32_t) D5_Pin << 16u;
	if (data & 0x01)
		D4_GPIO_Port->BSRR = D4_Pin;
	else
		D4_GPIO_Port->BSRR = (uint32_t) D4_Pin << 16u;

	EN_GPIO_Port->BSRR = EN_Pin;
	HAL_Delay(1);
	EN_GPIO_Port->BSRR = (uint32_t) EN_Pin << 16u;
	HAL_Delay(1);
}

/*******************************************************************************
*  @brief This functions sends a string to the display.
*
*  @param1 String to be sended.
*
******************************************************************************/
void LCD_String(char *string)
{
	while (*string)
	{
		LCD_Data(*string);
		string++;
	}
}

/*******************************************************************************
*  @brief This function sends a command to the display.
*
*  @param1 Command to be sended.
*
*  @return None.
*
******************************************************************************/
void LCD_Cmd(uint8_t cmd)  // Fun��o para enviar um comando para o Display
{
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);

	if (cmd & 0x80)
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_RESET);
	if (cmd & 0x40)
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_RESET);
	if (cmd & 0x20)
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);
	if (cmd & 0x10)
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	if (cmd & 0x08)
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, GPIO_PIN_RESET);
	if (cmd & 0x04)
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, GPIO_PIN_RESET);
	if (cmd & 0x02)
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);
	if (cmd & 0x01)
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

}

/*******************************************************************************
*  @brief This function clear the display screen.
*
*  @return None.
*
******************************************************************************/
void LCD_Clear(void)
{
	LCD_Cmd(0x01);
}

/*******************************************************************************
*  @brief Setup the display to work with 4 bits.
*
*  @return None.
*
******************************************************************************/
void LCD_Init(void)
{
	LCD_Cmd(0x33);
	LCD_Cmd(0x32);
	LCD_Cmd(0x28);
	LCD_Cmd(0x06);
	LCD_Cmd(0x0C);
	LCD_Cmd(0x01);
}
