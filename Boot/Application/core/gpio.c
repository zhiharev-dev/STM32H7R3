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

static void gpio_xspi1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO
 */
void gpio_init(void)
{
    /* Включить тактирование GPIO */
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN_Msk);
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOOEN_Msk);
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOPEN_Msk);

    gpio_xspi1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO XSPI1
 */
static void gpio_xspi1_init(void)
{
    /*
     * GPIOO0 XSPIM_P1_NCS1
     * GPIOO4 XSPIM_P1_CLK
     * GPIOP0 XSPIM_P1_IO0
     * GPIOP1 XSPIM_P1_IO1
     * GPIOP2 XSPIM_P1_IO2
     * GPIOP3 XSPIM_P1_IO3
     */

    /* GPIOO --------------------------------------------------------------- */

    /* Настроить режим работы ввода-вывода = AF */
    MODIFY_REG(GPIOO->MODER,
               GPIO_MODER_MODE0_Msk
             | GPIO_MODER_MODE4_Msk,
               0x02 << GPIO_MODER_MODE0_Pos
             | 0x02 << GPIO_MODER_MODE4_Pos);

    /* Настроить тип ввода-вывода = Push-Pull */
    CLEAR_BIT(GPIOO->OTYPER,
              GPIO_OTYPER_OT0_Msk
            | GPIO_OTYPER_OT4_Msk);

    /* Настроить скорость работы ввода-вывода = Very High Speed */
    SET_BIT(GPIOO->OSPEEDR,
            GPIO_OSPEEDR_OSPEED0_Msk
          | GPIO_OSPEEDR_OSPEED4_Msk);

    /* Настроить подтягивание/понижение ввода-вывода = No-Pull */
    CLEAR_BIT(GPIOO->PUPDR,
              GPIO_PUPDR_PUPD0_Msk
            | GPIO_PUPDR_PUPD4_Msk);

    /* Настроить номер альтернативной функции ввода-вывода = 9 */
    MODIFY_REG(GPIOO->AFR[0],
               GPIO_AFRL_AFSEL0_Msk
             | GPIO_AFRL_AFSEL4_Msk,
               0x09 << GPIO_AFRL_AFSEL0_Pos
             | 0x09 << GPIO_AFRL_AFSEL4_Pos);
    /* --------------------------------------------------------------------- */


    /* GPIOP --------------------------------------------------------------- */

    /* Настроить режим работы ввода-вывода = AF */
    MODIFY_REG(GPIOP->MODER,
               GPIO_MODER_MODE0_Msk
             | GPIO_MODER_MODE1_Msk
             | GPIO_MODER_MODE2_Msk
             | GPIO_MODER_MODE3_Msk,
               0x02 << GPIO_MODER_MODE0_Pos
             | 0x02 << GPIO_MODER_MODE1_Pos
             | 0x02 << GPIO_MODER_MODE2_Pos
             | 0x02 << GPIO_MODER_MODE3_Pos);

    /* Настроить тип ввода-вывода = Push-Pull */
    CLEAR_BIT(GPIOP->OTYPER,
              GPIO_OTYPER_OT0_Msk
            | GPIO_OTYPER_OT1_Msk
            | GPIO_OTYPER_OT2_Msk
            | GPIO_OTYPER_OT3_Msk);

    /* Настроить скорость работы ввода-вывода = Very High Speed */
    SET_BIT(GPIOP->OSPEEDR,
            GPIO_OSPEEDR_OSPEED0_Msk
          | GPIO_OSPEEDR_OSPEED1_Msk
          | GPIO_OSPEEDR_OSPEED2_Msk
          | GPIO_OSPEEDR_OSPEED3_Msk);

    /* Настроить подтягивание/понижение ввода-вывода = No-Pull */
    CLEAR_BIT(GPIOP->PUPDR,
              GPIO_PUPDR_PUPD0_Msk
            | GPIO_PUPDR_PUPD1_Msk
            | GPIO_PUPDR_PUPD2_Msk
            | GPIO_PUPDR_PUPD3_Msk);

    /* Настроить номер альтернативной функции ввода-вывода = 9 */
    MODIFY_REG(GPIOP->AFR[0],
               GPIO_AFRL_AFSEL0_Msk
             | GPIO_AFRL_AFSEL1_Msk
             | GPIO_AFRL_AFSEL2_Msk
             | GPIO_AFRL_AFSEL3_Msk,
               0x09 << GPIO_AFRL_AFSEL0_Pos
             | 0x09 << GPIO_AFRL_AFSEL1_Pos
             | 0x09 << GPIO_AFRL_AFSEL2_Pos
             | 0x09 << GPIO_AFRL_AFSEL3_Pos);
    /* --------------------------------------------------------------------- */
}
/* ------------------------------------------------------------------------- */
