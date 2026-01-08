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

#ifndef STM32H7RSXX_HAL_PWR_H_
#define STM32H7RSXX_HAL_PWR_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32h7rsxx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных PWR
 */
typedef PWR_TypeDef pwr_t;


/**
 * @brief Определение перечисления конфигурации источника питания PWR
 */
typedef enum pwr_supply {
    PWR_DEFAULT_SUPPLY = 0x06,
    PWR_LDO = 0x02,
    PWR_DIRECT_SMPS = 0x04,
    PWR_EXTERNAL_SMPS_LDO = 0x1E,
    PWR_EXTERNAL_SMPS_LDO_BYPASS = 0x1D,
    PWR_BYPASS = 0x01,
} pwr_supply_t;


/**
 * @brief Определение перечисления масштабирования напряжения PWR
 */
typedef enum pwr_vos {
    PWR_VOS_LOW,
    PWR_VOS_HIGH,
} pwr_vos_t;


/**
 * @brief Определение структуры данных инициализации PWR
 */
typedef struct pwr_init {
    uint32_t supply;                            /*!< Конфигурация источника питания @ref pwr_supply_t */

    uint32_t vos;                               /*!< Масштабирование напряжения @ref pwr_vos_t */

    uint32_t xspim1_enable;                     /*!< Включить XSPIM1 @ref hal_status_t */
} pwr_init_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_pwr_init(pwr_init_t * pwr_init);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_PWR_H_ */
