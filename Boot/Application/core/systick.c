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

volatile uint32_t ticks;

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать SysTick
 *
 * @param frequency Частота тактирования (Гц)
 */
void systick_init(const uint32_t frequency)
{
    uint32_t reload = (frequency / 1000) - 1;

    /* Сбросить регистр управления SysTick */
    CLEAR_REG(SysTick->CTRL);

    /* Записать значение перезагрузки счетчика SysTick */
    WRITE_REG(SysTick->LOAD, reload);

    /* Сбросить текущее значение счетчика SysTick */
    CLEAR_REG(SysTick->VAL);

    /* Настроить и запустить SysTick */
    WRITE_REG(SysTick->CTRL,
              SysTick_CTRL_CLKSOURCE_Msk
            | SysTick_CTRL_TICKINT_Msk
            | SysTick_CTRL_ENABLE_Msk);

    /* Настроить NVIC */
    NVIC_SetPriority(SysTick_IRQn, 15);
    NVIC_EnableIRQ(SysTick_IRQn);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Обработать прерывания SysTick
 */
inline void systick_it_handler(void)
{
    ticks++;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Получить значение системного таймера SysTick
 *
 * @return Значение системного таймера (мс)
 */
inline uint32_t systick_get_ticks(void)
{
    return ticks;
}
/* ------------------------------------------------------------------------- */
