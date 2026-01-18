/*
 * Copyright (C) 2026 zhiharev-dev <zhiharev.dev@mail.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/* Includes ---------------------------------------------------------------- */

#include "gpio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void gpio_led_init(void);

static void gpio_sd_card_init(void);

static void gpio_sdmmc1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO
 */
void gpio_init(void)
{
    /* Включить тактирование GPIO */
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN_Msk);
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN_Msk);
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN_Msk);
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN_Msk);

    gpio_led_init();
    gpio_sd_card_init();
    gpio_sdmmc1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO LED
 */
static void gpio_led_init(void)
{
    /* Установить начальное значение = Reset */
    CLEAR_BIT(GPIOB->ODR, GPIO_ODR_OD2_Msk);

    /* Настроить режим работы ввода-вывода = Output */
    MODIFY_REG(GPIOB->MODER,
               GPIO_MODER_MODE2_Msk,
               0x01 << GPIO_MODER_MODE2_Pos);

    /* Настроить тип ввода-вывода = Push-Pull */
    CLEAR_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT2_Msk);

    /* Настроить скорость работы ввода-вывода = Low Speed */
    CLEAR_BIT(GPIOB->OSPEEDR, GPIO_OSPEEDR_OSPEED2_Msk);

    /* Настроить подтягивание/понижение ввода-вывода = Pull-Up */
    MODIFY_REG(GPIOB->PUPDR,
               GPIO_PUPDR_PUPD2_Msk,
               0x01 << GPIO_PUPDR_PUPD2_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO SD card
 */
static void gpio_sd_card_init(void)
{
    /* GPIOA8 SD_DETECT */

    /* Настроить режим работы ввода-вывода = Input */
    CLEAR_BIT(GPIOA->MODER, GPIO_MODER_MODE8_Msk);

    /* Настроить подтягивание/понижение ввода-вывода = No Pull */
    CLEAR_BIT(GPIOA->PUPDR, GPIO_PUPDR_PUPD8_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO SDMMC1
 */
static void gpio_sdmmc1_init(void)
{
    /*
     * GPIOC8  SDMMC1_D0
     * GPIOC9  SDMMC1_D1
     * GPIOC10 SDMMC1_D2
     * GPIOC11 SDMMC1_D3
     * GPIOC12 SDMMC1_CLK
     * GPIOD2  SDMMC1_CMD
     */

    /* GPIOC --------------------------------------------------------------- */

    /* Настроить режим работы ввода-вывода = AF */
    MODIFY_REG(GPIOC->MODER,
               GPIO_MODER_MODE8_Msk
             | GPIO_MODER_MODE9_Msk
             | GPIO_MODER_MODE10_Msk
             | GPIO_MODER_MODE11_Msk
             | GPIO_MODER_MODE12_Msk,
               0x02 << GPIO_MODER_MODE8_Pos
             | 0x02 << GPIO_MODER_MODE9_Pos
             | 0x02 << GPIO_MODER_MODE10_Pos
             | 0x02 << GPIO_MODER_MODE11_Pos
             | 0x02 << GPIO_MODER_MODE12_Pos);

    /* Настроить тип ввода-вывода = Push-Pull */
    CLEAR_BIT(GPIOC->OTYPER,
              GPIO_OTYPER_OT8_Msk
            | GPIO_OTYPER_OT9_Msk
            | GPIO_OTYPER_OT10_Msk
            | GPIO_OTYPER_OT11_Msk
            | GPIO_OTYPER_OT12_Msk);

    /* Настроить скорость работы ввода-вывода = Very High Speed */
    SET_BIT(GPIOC->OSPEEDR,
            GPIO_OSPEEDR_OSPEED8_Msk
          | GPIO_OSPEEDR_OSPEED9_Msk
          | GPIO_OSPEEDR_OSPEED10_Msk
          | GPIO_OSPEEDR_OSPEED11_Msk
          | GPIO_OSPEEDR_OSPEED12_Msk);

    /* Настроить подтягивание/понижение ввода-вывода = No-Pull */
    CLEAR_BIT(GPIOC->PUPDR,
              GPIO_PUPDR_PUPD8_Msk
            | GPIO_PUPDR_PUPD9_Msk
            | GPIO_PUPDR_PUPD10_Msk
            | GPIO_PUPDR_PUPD11_Msk
            | GPIO_PUPDR_PUPD12_Msk);

    /* Настроить номер альтернативной функции ввода-вывода = 11, 12 */
    MODIFY_REG(GPIOC->AFR[1],
               GPIO_AFRH_AFSEL8_Msk
             | GPIO_AFRH_AFSEL9_Msk
             | GPIO_AFRH_AFSEL10_Msk
             | GPIO_AFRH_AFSEL11_Msk
             | GPIO_AFRH_AFSEL12_Msk,
               0x0B << GPIO_AFRH_AFSEL8_Pos
             | 0x0B << GPIO_AFRH_AFSEL9_Pos
             | 0x0C << GPIO_AFRH_AFSEL10_Pos
             | 0x0B << GPIO_AFRH_AFSEL11_Pos
             | 0x0B << GPIO_AFRH_AFSEL12_Pos);
    /* --------------------------------------------------------------------- */


    /* GPIOD --------------------------------------------------------------- */

    /* Настроить режим работы ввода-вывода = AF */
    MODIFY_REG(GPIOD->MODER,
               GPIO_MODER_MODE2_Msk,
               0x02 << GPIO_MODER_MODE2_Pos);

    /* Настроить тип ввода-вывода = Push-Pull */
    CLEAR_BIT(GPIOD->OTYPER, GPIO_OTYPER_OT2_Msk);

    /* Настроить скорость работы ввода-вывода = Very High Speed */
    SET_BIT(GPIOD->OSPEEDR, GPIO_OSPEEDR_OSPEED2_Msk);

    /* Настроить подтягивание/понижение ввода-вывода = No-Pull */
    CLEAR_BIT(GPIOD->PUPDR, GPIO_PUPDR_PUPD2_Msk);

    /* Настроить номер альтернативной функции ввода-вывода = 11 */
    MODIFY_REG(GPIOD->AFR[0],
               GPIO_AFRL_AFSEL2_Msk,
               0x0B << GPIO_AFRL_AFSEL2_Pos);
    /* --------------------------------------------------------------------- */
}
/* ------------------------------------------------------------------------- */
