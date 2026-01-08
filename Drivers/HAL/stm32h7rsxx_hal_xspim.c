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

#include "stm32h7rsxx_hal_xspim.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_xspim_setup_mux(uint32_t state);

static void hal_xspim_setup_mode(uint32_t mode);

static void hal_xspim_setup_cssel_ovr(uint32_t state, uint32_t xspi1, uint32_t xspi2);

static void hal_xspim_setup_req2ack_time(uint32_t value);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать XSPIM
 *
 * @param xspim_init Указатель на структуру данных инициализации XSPIM
 */
void hal_xspim_init(xspim_init_t * xspim_init)
{
    assert(xspim_init != NULL);

    hal_xspim_setup_mux(xspim_init->mux_enable);
    hal_xspim_setup_mode(xspim_init->mode);
    hal_xspim_setup_cssel_ovr(xspim_init->cssel_ovr_enable, xspim_init->cssel_ovr_xspi1, xspim_init->cssel_ovr_xspi2);
    hal_xspim_setup_req2ack_time(xspim_init->req2ack_time);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние мультиплексного режима XSPIM
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_xspim_setup_mux(uint32_t state)
{
    MODIFY_REG(XSPIM->CR,
               XSPIM_CR_MUXEN_Msk,
               state << XSPIM_CR_MUXEN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить режим работы XSPIM
 *
 * @param mode Режим работы @ref xspim_mode_t
 */
static void hal_xspim_setup_mode(uint32_t mode)
{
    MODIFY_REG(XSPIM->CR,
               XSPIM_CR_MODE_Msk,
               mode << XSPIM_CR_MODE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить переопределение CSS XSPIM
 *
 * @param state Состояние @ref hal_state_t
 * @param xspi1 Переопределение CSS для XSPI1 @ref xspim_cssel_ovr_t
 * @param xspi2 Переопределение CSS для XSPI2 @ref xspim_cssel_ovr_t
 */
static void hal_xspim_setup_cssel_ovr(uint32_t state, uint32_t xspi1, uint32_t xspi2)
{
    MODIFY_REG(XSPIM->CR,
               XSPIM_CR_CSSEL_OVR_EN_Msk
             | XSPIM_CR_CSSEL_OVR_O1_Msk
             | XSPIM_CR_CSSEL_OVR_O2_Msk,
               state << XSPIM_CR_CSSEL_OVR_EN_Pos
             | xspi1 << XSPIM_CR_CSSEL_OVR_O1_Pos
             | xspi2 << XSPIM_CR_CSSEL_OVR_O2_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить время между двумя транзакциями в мультиплексном режиме XSPIM
 *
 * @param value Значение времени
 */
static void hal_xspim_setup_req2ack_time(uint32_t value)
{
    MODIFY_REG(XSPIM->CR,
               XSPIM_CR_REQ2ACK_TIME_Msk,
               value << XSPIM_CR_REQ2ACK_TIME_Pos);
}
/* ------------------------------------------------------------------------- */
