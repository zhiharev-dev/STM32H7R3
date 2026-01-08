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

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO
 */
void gpio_init(void)
{
    HAL_GPIOA_ENABLE_CLOCK();
    HAL_GPIOB_ENABLE_CLOCK();

    gpio_led_init();
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
