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

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO
 */
void gpio_init(void)
{
    /* Включить тактирование GPIO */
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN_Msk);
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN_Msk);

    gpio_led_init();
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
