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

#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "gpio.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение перечисления состояний светодиода
 */
typedef enum led_state {
    LED_OFF = GPIO_RESET,
    LED_ON = GPIO_SET,
} led_state_t;


/**
 * @brief Определение структуры данных светодиода
 */
typedef struct led {
    gpio_handle_t * gpio_handle;                /*!< Указатель на структуру данных обработчика GPIO */
} led_t;

/* Exported variables ------------------------------------------------------ */

extern led_t led_system;

/* Exported function prototypes -------------------------------------------- */

void led_on(led_t * self);

void led_off(led_t * self);

void led_toggle(led_t * self);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LED_H_ */
