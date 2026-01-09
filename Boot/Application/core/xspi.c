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

#include "xspi.h"
#include "rcc.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

xspi_handle_t xspi1 = {
    .instance = XSPI1,
};

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать XSPI
 */
void xspi_init(void)
{
    xspi_init_t xspi_init = {
        .mode = XSPI_MODE0,
        .prescaler = 1,
        .memory_type = XSPI_MACRONIX,
        .memory_size = XSPI_SIZE_8MB,
    };

    HAL_XSPI1_ENABLE_CLOCK();

    HAL_RCC_XSPI_ENABLE_PROTECTION_CLOCK();

    hal_xspi_init(&xspi1, &xspi_init);
    hal_xspi_enable(&xspi1);
}
/* ------------------------------------------------------------------------- */
