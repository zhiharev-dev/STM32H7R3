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

#ifndef STM32H7RSXX_HAL_XSPIM_H_
#define STM32H7RSXX_HAL_XSPIM_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32h7rsxx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief Включить тактирование XSPIM
 */
#define HAL_XSPIM_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB5ENR, RCC_AHB5ENR_XSPIMEN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных XSPIM
 */
typedef XSPIM_TypeDef xspim_t;


/**
 * @brief Определение перечисления режимов работы XSPIM
 */
typedef enum xspim_mode {
    XSPIM_DIRECT_MODE,
    XSPIM_ARBITRATION_P1_MODE,                  /*!< Если включен мультиплексный режим */
    XSPIM_SWAPPED_MODE = 0x00,
    XSPIM_ARBITRATION_P2_MODE,                  /*!< Если включен мультиплексный режим */
} xspim_mode_t;


/**
 * @brief Определение перечисления переопределения CSS
 */
typedef enum xspim_cssel_ovr {
    XSPIM_CSSEL_OVR_NCS1,
    XSPIM_CSSEL_OVR_NCS2,
} xspim_cssel_ovr_t;


/**
 * @brief Определение структуры данных инициализации XSPIM
 */
typedef struct xspim_init {
    uint32_t mux_enable;                        /*!< Включить мультиплексный режим @ref hal_state_t */

    uint32_t mode;                              /*!< Режим работы @ref xspim_mode_t */

    uint32_t cssel_ovr_enable;                  /*!< Включить переопределение CSS @ref hal_state_t */

    uint32_t cssel_ovr_xspi1;                   /*!< Переопределение CSS для XSPI1 @ref xspim_cssel_ovr_t */

    uint32_t cssel_ovr_xspi2;                   /*!< Переопределение CSS для XSPI2 @ref xspim_cssel_ovr_t */

    uint32_t req2ack_time;                      /*!< Время между двумя транзакциями в мультиплексном режиме */
} xspim_init_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_xspim_init(xspim_init_t * xspim_init);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_XSPIM_H_ */
