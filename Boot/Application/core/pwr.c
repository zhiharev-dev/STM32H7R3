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

#include "pwr.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать PWR
 */
void pwr_init(void)
{
    /* Отключить защиту от записи резервного домена  */
    SET_BIT(PWR->CR1, PWR_CR1_DBP_Msk);

    /* Включить резервный регулятор */
    SET_BIT(PWR->CSR1, PWR_CSR1_BREN_Msk);
    while (!READ_BIT(PWR->CSR1, PWR_CSR1_BRRDY_Msk)) {}

    /* Настроить источник питания = Direct SMPS */
    WRITE_REG(PWR->CSR2, 0x04);
    while (!READ_BIT(PWR->SR1, PWR_SR1_ACTVOSRDY_Msk)) {}

    /* Настроить масштабирование напряжения = High */
    SET_BIT(PWR->CSR4, PWR_CSR4_VOS_Msk);
    while (!READ_BIT(PWR->CSR4, PWR_CSR4_VOSRDY_Msk)) {}
}
/* ------------------------------------------------------------------------- */
