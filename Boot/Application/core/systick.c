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

#include "systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать SysTick
 *
 * @param frequency Частота тактирования (Гц)
 */
void systick_init(const uint32_t frequency)
{
    systick_init_t systick_init = {
        .frequency = frequency,
        .clksource = SYSTICK_CPU_CLOCK,
    };

    hal_systick_init(&systick_init);
    hal_systick_start_it();

    NVIC_SetPriority(SysTick_IRQn, 15);
}
/* ------------------------------------------------------------------------- */

void hal_systick_period_elapsed_callback(void)
{

}
/* ------------------------------------------------------------------------- */
