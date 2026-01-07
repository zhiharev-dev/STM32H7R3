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

#include "stm32h7rsxx_hal_systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

volatile uint32_t ticks;

/* Private function prototypes --------------------------------------------- */

static void hal_systick_reset(void);

static void hal_systick_setup_clock_source(uint32_t clksource);

static uint32_t hal_systick_compute_reload_value(uint32_t frequency);

static void hal_systick_setup_reload_value(uint32_t value);

static void hal_systick_setup_current_value(uint32_t value);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать SysTick
 *
 * @param systick_init Указатель на структуру данных инициализации SysTick
 */
void hal_systick_init(systick_init_t * systick_init)
{
    uint32_t reload_value;

    assert(systick_init != NULL);

    hal_systick_reset();
    hal_systick_setup_clock_source(systick_init->clksource);
    reload_value = hal_systick_compute_reload_value(systick_init->frequency);
    hal_systick_setup_reload_value(reload_value);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Сбросить SysTick
 */
static void hal_systick_reset(void)
{
    CLEAR_REG(SysTick->CTRL);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить источник тактирования SysTick
 *
 * @param clksource Источник тактирования SysTick @ref systick_clock_source_t
 */
static void hal_systick_setup_clock_source(uint32_t clksource)
{
    MODIFY_REG(SysTick->CTRL,
               SysTick_CTRL_CLKSOURCE_Msk,
               clksource << SysTick_CTRL_CLKSOURCE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Рассчитать значение перезагрузки счетчика SysTick
 *
 * @param frequency Частота тактирования SysTick (Гц)
 *
 * @return Значение перезагрузки счетчика
 */
static uint32_t hal_systick_compute_reload_value(uint32_t frequency)
{
    return (frequency / 1000) - 1;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить значение перезагрузки счетчика SysTick
 *
 * @param value Значение перезагрузки счетчика
 */
static void hal_systick_setup_reload_value(uint32_t value)
{
    WRITE_REG(SysTick->LOAD, value);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить текущее значение счетчика SysTick
 *
 * @param value Текущее значение счетчика
 */
static void hal_systick_setup_current_value(uint32_t value)
{
    WRITE_REG(SysTick->VAL, value);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Обработать прерывания SysTick
 */
inline void hal_systick_it_handler(void)
{
    ticks++;

    hal_systick_period_elapsed_callback();
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Запустить SysTick
 */
void hal_systick_start_it(void)
{
    hal_systick_setup_current_value(0);

    SET_BIT(SysTick->CTRL,
            SysTick_CTRL_TICKINT_Msk
          | SysTick_CTRL_ENABLE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Остановить SysTick
 */
void hal_systick_stop_it(void)
{
    CLEAR_BIT(SysTick->CTRL,
              SysTick_CTRL_TICKINT_Msk
            | SysTick_CTRL_ENABLE_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Получить текущее значение таймера SysTick
 *
 * @return Значение таймера (мс)
 */
inline uint32_t hal_systick_get_ticks(void)
{
    return ticks;
}
/* ------------------------------------------------------------------------- */

__WEAK void hal_systick_period_elapsed_callback(void)
{

}
/* ------------------------------------------------------------------------- */
