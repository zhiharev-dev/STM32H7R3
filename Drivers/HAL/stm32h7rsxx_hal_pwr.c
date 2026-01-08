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

#include "stm32h7rsxx_hal_pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_pwr_disable_bkp_protect(void);

static void hal_pwr_setup_supply(uint32_t supply);

static void hal_pwr_setup_vos(uint32_t vos);

static void hal_pwr_setup_xspim1(uint32_t state);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать PWR
 *
 * @param pwr_init Указатель на структуру данных инициализации PWR
 */
void hal_pwr_init(pwr_init_t * pwr_init)
{
    assert(pwr_init != NULL);

    hal_pwr_disable_bkp_protect();
    hal_pwr_setup_supply(pwr_init->supply);
    hal_pwr_setup_vos(pwr_init->vos);
    hal_pwr_setup_xspim1(pwr_init->xspim1_enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Выключить защиту резервного домена
 */
static void hal_pwr_disable_bkp_protect(void)
{
    SET_BIT(PWR->CR1, PWR_CR1_DBP_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить источник питания PWR
 *
 * @param supply Конфигурация источника питания PWR @ref pwr_supply_t
 */
static void hal_pwr_setup_supply(uint32_t supply)
{
    WRITE_REG(PWR->CSR2, supply);

    while (!READ_BIT(PWR->SR1, PWR_SR1_ACTVOSRDY_Msk)) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить масштабирование напряжения PWR
 *
 * @param vos Масштабирование напряжения PWR @ref pwr_vos_t
 */
static void hal_pwr_setup_vos(uint32_t vos)
{
    MODIFY_REG(PWR->CSR4,
               PWR_CSR4_VOS_Msk,
               vos << PWR_CSR4_VOS_Pos);

    while (!READ_BIT(PWR->CSR4, PWR_CSR4_VOSRDY_Msk)) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние XSPIM1
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_pwr_setup_xspim1(uint32_t state)
{
    MODIFY_REG(PWR->CSR2,
               PWR_CSR2_EN_XSPIM1_Msk,
               state << PWR_CSR2_EN_XSPIM1_Pos);
}
/* ------------------------------------------------------------------------- */
