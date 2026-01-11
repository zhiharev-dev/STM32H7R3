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

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать XSPI
 */
void xspi_init(void)
{
    /* Включить интерфейс XSPIM1 */
    SET_BIT(PWR->CSR2, PWR_CSR2_EN_XSPIM1_Msk);

    /* Включить тактирование XSPIM */
    SET_BIT(RCC->AHB5ENR, RCC_AHB5ENR_XSPIMEN_Msk);

    /* Включить тактирование XSPI1 */
    SET_BIT(RCC->AHB5ENR, RCC_AHB5ENR_XSPI1EN_Msk);

    /* Настроить источник тактирования XSPI1 = PLL2T */
    MODIFY_REG(RCC->CCIPR1,
               RCC_CCIPR1_XSPI1SEL_Msk,
               0x02 << RCC_CCIPR1_XSPI1SEL_Pos);

    /* Включить защиту тактирования XSPI */
    SET_BIT(RCC->CKPROTR, RCC_CKPROTR_XSPICKP_Msk);


    /* XSPIM --------------------------------------------------------------- */

    CLEAR_REG(XSPIM->CR);
    /* --------------------------------------------------------------------- */


    /* XSPI1 --------------------------------------------------------------- */

    /* Настроить:
     *   - Режим работы = Mode 0
     *   - Тип памяти = Macronix
     *   - Размер памяти = 8MB
     */
    WRITE_REG(XSPI1->DCR1,
              0x00 << XSPI_DCR1_CKMODE_Pos
            | 0x01 << XSPI_DCR1_MTYP_Pos
            | 0x13 << XSPI_DCR1_DEVSIZE_Pos);

    /* Настроить делитель часов = 1 (100MHz / 1 = 100MHz) */
    CLEAR_BIT(XSPI1->DCR2, XSPI_DCR2_PRESCALER_Msk);

    /* Включить XSPI */
    SET_BIT(XSPI1->CR, XSPI_CR_EN_Msk);
    /* --------------------------------------------------------------------- */
}
/* ------------------------------------------------------------------------- */
