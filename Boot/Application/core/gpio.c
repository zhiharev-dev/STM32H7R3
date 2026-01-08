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

gpio_handle_t gpio_led_system = {
    .instance = GPIOB,
    .pin = GPIO_PIN2,
};

/* Private function prototypes --------------------------------------------- */

static void gpio_led_init(void);

static void gpio_xspi1_init(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO
 */
void gpio_init(void)
{
    HAL_GPIOA_ENABLE_CLOCK();
    HAL_GPIOB_ENABLE_CLOCK();
    HAL_GPIOO_ENABLE_CLOCK();
    HAL_GPIOP_ENABLE_CLOCK();

    gpio_led_init();
    gpio_xspi1_init();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO LED
 */
static void gpio_led_init(void)
{
    gpio_init_t gpio_init = {
         .mode = GPIO_OUTPUT,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_LOW_SPEED,
         .pupd = GPIO_PULL_UP,
    };

    hal_gpio_set_state(&gpio_led_system, GPIO_RESET);

    hal_gpio_init(&gpio_led_system, &gpio_init);
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

    gpio_init_t gpio_init = {
         .mode = GPIO_AF,
         .otype = GPIO_PUSH_PULL,
         .ospeed = GPIO_VERY_HIGH_SPEED,
         .pupd = GPIO_NO_PULL,
         .af = 9,
    };

    gpio_handle_t gpio_ncs1 = {
        .instance = GPIOO,
        .pin = GPIO_PIN0,
    };

    gpio_handle_t gpio_clk = {
        .instance = GPIOO,
        .pin = GPIO_PIN4,
    };

    gpio_handle_t gpio_io0 = {
        .instance = GPIOP,
        .pin = GPIO_PIN0,
    };

    gpio_handle_t gpio_io1 = {
        .instance = GPIOP,
        .pin = GPIO_PIN1,
    };

    gpio_handle_t gpio_io2 = {
        .instance = GPIOP,
        .pin = GPIO_PIN2,
    };

    gpio_handle_t gpio_io3 = {
        .instance = GPIOP,
        .pin = GPIO_PIN3,
    };


    hal_gpio_init(&gpio_ncs1, &gpio_init);
    hal_gpio_init(&gpio_clk, &gpio_init);
    hal_gpio_init(&gpio_io0, &gpio_init);
    hal_gpio_init(&gpio_io1, &gpio_init);
    hal_gpio_init(&gpio_io2, &gpio_init);
    hal_gpio_init(&gpio_io3, &gpio_init);
}
/* ------------------------------------------------------------------------- */
