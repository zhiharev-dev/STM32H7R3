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

#include "stm32h7rsxx_hal_flash.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_flash_setup_latency(uint32_t latency);

static void hal_flash_setup_wrhighfreq(uint32_t wrhighfreq);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать FLASH
 *
 * @param flash_init Указатель на структуру данных инициализации FLASH
 */
void hal_flash_init(flash_init_t * flash_init)
{
    assert(flash_init != NULL);

    hal_flash_setup_latency(flash_init->latency);
    hal_flash_setup_wrhighfreq(flash_init->wrhighfreq);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить задержу чтения данных FLASH
 *
 * @param latency Задержка чтения данных FLASH @ref flash_latency_t
 */
static void hal_flash_setup_latency(uint32_t latency)
{
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_LATENCY_Msk,
               latency << FLASH_ACR_LATENCY_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить задержку записи данных FLASH
 *
 * @param wrhighfreq Задержка записи данных FLASH @ref flash_wrhighfreq_t
 */
static void hal_flash_setup_wrhighfreq(uint32_t wrhighfreq)
{
    MODIFY_REG(FLASH->ACR,
               FLASH_ACR_WRHIGHFREQ_Msk,
               wrhighfreq << FLASH_ACR_WRHIGHFREQ_Pos);
}
/* ------------------------------------------------------------------------- */
