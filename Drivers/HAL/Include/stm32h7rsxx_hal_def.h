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

#ifndef STM32H7RSXX_HAL_DEF_H_
#define STM32H7RSXX_HAL_DEF_H_

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "stm32h7rsxx.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief Битовая маска
 *
 * @param x Значение
 * @param n Положение
 *
 * @return Значение битовой маски
 */
#define HAL_BITMASK(x, n)       ((x) << (n))

/* Exported constants ------------------------------------------------------ */

/**
 * @brief Максимальное значение задержки
 */
#define HAL_MAX_DELAY       0xFFFFFFFF

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение типа данных статуса
 */
typedef enum hal_status {
    HAL_OK,
    HAL_BUSY,
    HAL_TIMEOUT,
    HAL_ERROR,
} hal_status_t;


/**
 * @brief Определение типа данных статуса флагов
 */
typedef enum hal_flag_status {
    HAL_RESET,
    HAL_SET,
} hal_flag_status_t;


/**
 * @brief Определение типа данных состояния
 */
typedef enum hal_state {
    HAL_DISABLE,
    HAL_ENABLE,
} hal_state_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_DEF_H_ */
