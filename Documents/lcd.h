/*******************************************************************************
 * @file lcd.h
 * @brief This module is responsible to control a LCD 16x2 Display.
 * @author Gustavo Adono
 *******************************************************************************
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

#ifndef __lcd_H
#define __lcd_H
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void LCD_Init (void);
void LCD_Cursor (uint8_t x, uint8_t y);
void LCD_String (char *string);
void LCD_Data (uint8_t data);
void LCD_Cmd (uint8_t cmd);
void LCD_Clear (void);
void Lcd_Out(uint8_t x, uint8_t y,char *string);

#ifdef __cplusplus
}
#endif
#endif /* __lcd_H */
