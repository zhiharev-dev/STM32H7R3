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

#include "led.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

led_t led_system = {
    .gpio_handle = &gpio_led_system,
};

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Включить светодиод
 *
 * @param self Указатель на структуру данных светодиода
 */
void led_on(led_t * self)
{
    assert(self != NULL);

    hal_gpio_set_state(self->gpio_handle, (gpio_state_t) LED_ON);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Выключить светодиод
 *
 * @param self Указатель на структуру данных светодиода
 */
void led_off(led_t * self)
{
    assert(self != NULL);

    hal_gpio_set_state(self->gpio_handle, (gpio_state_t) LED_OFF);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Переключить состояние светодиода
 *
 * @param self Указатель на структуру данных светодиода
 */
void led_toggle(led_t * self)
{
    assert(self != NULL);

    hal_gpio_toggle(self->gpio_handle);
}
/* ------------------------------------------------------------------------- */
