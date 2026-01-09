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

#ifndef STM32H7RSXX_HAL_XSPI_H_
#define STM32H7RSXX_HAL_XSPI_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32h7rsxx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief Включить тактирование XSPI1
 */
#define HAL_XSPI1_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB5ENR, RCC_AHB5ENR_XSPI1EN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных XSPI
 */
typedef XSPI_TypeDef xspi_t;


/**
 * @brief Определение перечисления режимов работы XSPI
 */
typedef enum xspi_mode {
    XSPI_MODE0,
    XSPI_MODE3,
} xspi_mode_t;


/**
 * @brief Определение перечисления типов памяти XSPI
 */
typedef enum xspi_memory_type {
    XSPI_MICRON,
    XSPI_MACRONIX,
    XSPI_STANDARD,
} xspi_memory_type_t;


/**
 * @brief Определение перечисления размеров памяти XSPI
 */
typedef enum xspi_memory_size {
    XSPI_SIZE_16B,
    XSPI_SIZE_32B,
    XSPI_SIZE_64B,
    XSPI_SIZE_128B,
    XSPI_SIZE_256B,
    XSPI_SIZE_512B,
    XSPI_SIZE_1KB,
    XSPI_SIZE_2KB,
    XSPI_SIZE_4KB,
    XSPI_SIZE_8KB,
    XSPI_SIZE_16KB,
    XSPI_SIZE_32KB,
    XSPI_SIZE_64KB,
    XSPI_SIZE_128KB,
    XSPI_SIZE_256KB,
    XSPI_SIZE_512KB,
    XSPI_SIZE_1MB,
    XSPI_SIZE_2MB,
    XSPI_SIZE_4MB,
    XSPI_SIZE_8MB,
    XSPI_SIZE_16MB,
    XSPI_SIZE_32MB,
    XSPI_SIZE_64MB,
    XSPI_SIZE_128MB,
    XSPI_SIZE_256MB,
    XSPI_SIZE_512MB,
    XSPI_SIZE_1GB,
    XSPI_SIZE_2GB,
    XSPI_SIZE_4GB,
    XSPI_SIZE_8GB,
    XSPI_SIZE_16GB,
    XSPI_SIZE_32GB,
} xspi_memory_size_t;


/**
 * @brief Определение структуры данных инициализации XSPI
 */
typedef struct xspi_init {
    uint32_t mode;                              /*!< Режим работы @ref xspi_mode_t */

    uint32_t prescaler;                         /*!< Делитель часов [1:256] */

    uint32_t memory_type;                       /*!< Тип памяти @ref xspi_memory_type_t */

    uint32_t memory_size;                       /*!< Размер памяти @ref xspi_memory_size_t */
} xspi_init_t;


/**
 * @brief Определение структуры данных обработчика XSPI
 */
typedef struct xspi_handle {
    xspi_t * instance;                          /*!< Указатель на структуру данных XSPI */
} xspi_handle_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_xspi_init(xspi_handle_t * xspi_handle, xspi_init_t * xspi_init);

void hal_xspi_enable(xspi_handle_t * xspi_handle);

void hal_xspi_disable(xspi_handle_t * xspi_handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_XSPI_H_ */
