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

#ifndef STM32H7RSXX_HAL_SYSTICK_H_
#define STM32H7RSXX_HAL_SYSTICK_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32h7rsxx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных SysTick
 */
typedef SysTick_Type systick_t;


/**
 * @brief Определение перечисления источников тактирования SysTick
 */
typedef enum systick_clock_source {
    SYSTICK_CPU_CLOCK_DIV8,
    SYSTICK_CPU_CLOCK,
} systick_clock_source_t;


/**
 * @brief Определение структуры данных инициализации SysTick
 */
typedef struct systick_init {
    uint32_t frequency;                         /*!< Частота тактирования (Гц) */

    uint32_t clksource;                         /*!< Источник тактирования @ref systick_clock_source_t */
} systick_init_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_systick_init(systick_init_t * systick_init);

void hal_systick_it_handler(void);

void hal_systick_start_it(void);

void hal_systick_stop_it(void);

uint32_t hal_systick_get_ticks(void);

/* Exported callback function prototypes ----------------------------------- */

__WEAK void hal_systick_period_elapsed_callback(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_SYSTICK_H_ */
