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

#include "stm32h7rsxx_hal_xspi.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_xspi_setup_mode(xspi_t * instance, uint32_t mode);

static void hal_xspi_setup_prescaler(xspi_t * instance, uint32_t value);

static void hal_xspi_setup_memory_type(xspi_t * instance, uint32_t memory_type);

static void hal_xspi_setup_memory_size(xspi_t * instance, uint32_t memory_size);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать XSPI
 *
 * @param xspi_handle Указатель на структуру данных обработчика XSPI
 * @param xspi_init Указатель на структуру данных инициализации XSPI
 */
void hal_xspi_init(xspi_handle_t * xspi_handle, xspi_init_t * xspi_init)
{
    assert(xspi_handle != NULL);
    assert(xspi_handle->instance != NULL);
    assert(xspi_init != NULL);

    hal_xspi_setup_mode(xspi_handle->instance, xspi_init->mode);
    hal_xspi_setup_prescaler(xspi_handle->instance, xspi_init->prescaler);
    hal_xspi_setup_memory_type(xspi_handle->instance, xspi_init->memory_type);
    hal_xspi_setup_memory_size(xspi_handle->instance, xspi_init->memory_size);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить режим работы XSPI
 *
 * @param instance Указатель на структуру данных XSPI
 * @param mode Режим работы @ref xspi_mode_t
 */
static void hal_xspi_setup_mode(xspi_t * instance, uint32_t mode)
{
    MODIFY_REG(instance->DCR1,
               XSPI_DCR1_CKMODE_Msk,
               mode << XSPI_DCR1_CKMODE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить делитель часов XSPI
 *
 * @param instance Указатель на структуру данных XSPI
 * @param value Значение делителя часов [1:256]
 */
static void hal_xspi_setup_prescaler(xspi_t * instance, uint32_t value)
{
    if (value == 0 || value > 256) {
        return;
    }

    MODIFY_REG(instance->DCR2,
               XSPI_DCR2_PRESCALER_Msk,
               (value - 1) << XSPI_DCR2_PRESCALER_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить тип памяти XSPI
 *
 * @param instance Указатель на структуру данных XSPI
 * @param memory_type Тип памяти @ref xspi_memory_type_t
 */
static void hal_xspi_setup_memory_type(xspi_t * instance, uint32_t memory_type)
{
    MODIFY_REG(instance->DCR1,
               XSPI_DCR1_MTYP_Msk,
               memory_type << XSPI_DCR1_MTYP_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить размер памяти XSPI
 *
 * @param instance Указатель на структуру данных XSPI
 * @param memory_size Размер памяти @ref xspi_memory_size_t
 */
static void hal_xspi_setup_memory_size(xspi_t * instance, uint32_t memory_size)
{
    MODIFY_REG(instance->DCR1,
               XSPI_DCR1_DEVSIZE_Msk,
               memory_size << XSPI_DCR1_DEVSIZE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Включить XSPI
 *
 * @param xspi_handle Указатель на структуру данных обработчика XSPI
 */
void hal_xspi_enable(xspi_handle_t * xspi_handle)
{
    assert(xspi_handle != NULL);
    assert(xspi_handle->instance != NULL);

    SET_BIT(xspi_handle->instance->CR, XSPI_CR_EN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Выключить XSPI
 *
 * @param xspi_handle Указатель на структуру данных обработчика XSPI
 */
void hal_xspi_disable(xspi_handle_t * xspi_handle)
{
    assert(xspi_handle != NULL);
    assert(xspi_handle->instance != NULL);

    CLEAR_BIT(xspi_handle->instance->CR, XSPI_CR_EN_Msk);
}
/* ------------------------------------------------------------------------- */
