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

#ifndef STM32H7RSXX_HAL_FLASH_H_
#define STM32H7RSXX_HAL_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32h7rsxx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных FLASH
 */
typedef FLASH_TypeDef flash_t;


/**
 * @brief Определение перечисления задержeк чтения данных FLASH
 */
typedef enum flash_latency {
    FLASH_LATENCY_0WS,
    FLASH_LATENCY_1WS,
    FLASH_LATENCY_2WS,
    FLASH_LATENCY_3WS,
    FLASH_LATENCY_4WS,
    FLASH_LATENCY_5WS,
    FLASH_LATENCY_6WS,
    FLASH_LATENCY_7WS,
} flash_latency_t;


/**
 * @brief Определение перечисления задержeк записи данных FLASH
 */
typedef enum flash_wrhighfreq {
    FLASH_WRHIGHFREQ0,
    FLASH_WRHIGHFREQ1,
    FLASH_WRHIGHFREQ2,
    FLASH_WRHIGHFREQ3,
} flash_wrhighfreq_t;


/**
 * @brief Определение структуры данных инициализации FLASH
 */
typedef struct flash_init {
    uint32_t latency;                           /*!< Задержка чтения данных @ref flash_latency_t */

    uint32_t wrhighfreq;                        /*!< Задержка записи данных @ref flash_wrhighfreq_t */
} flash_init_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_flash_init(flash_init_t * flash_init);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_FLASH_H_ */
