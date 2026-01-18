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

#include "rcc.h"
#include "systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define RCC_HSERDY_TIMEOUT      100
#define RCC_LSERDY_TIMEOUT      5000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать RCC
 */
void rcc_init(void)
{
    uint32_t tickstart = systick_get_ticks();

    /* HSE ----------------------------------------------------------------- */

    /* Включить HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON_Msk);

    while (!READ_BIT(RCC->CR, RCC_CR_HSERDY_Msk)) {
        if (systick_get_ticks() - tickstart > RCC_HSERDY_TIMEOUT) {
            error();
        }
    }

    /* Включить CSS HSE */
    SET_BIT(RCC->CR, RCC_CR_HSECSSON_Msk);
    /* --------------------------------------------------------------------- */


    /* LSE ----------------------------------------------------------------- */

    /* Включить LSE */
    SET_BIT(RCC->BDCR, RCC_BDCR_LSEON_Msk);

    while (!READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY_Msk)) {
        if (systick_get_ticks() - tickstart > RCC_LSERDY_TIMEOUT) {
            error();
        }
    }

    /* Включить CSS LSE */
    SET_BIT(RCC->BDCR, RCC_BDCR_LSECSSON_Msk);
    /* --------------------------------------------------------------------- */


    /* PLLs ---------------------------------------------------------------- */

    /* Выключить PLLs */
    CLEAR_BIT(RCC->CR,
              RCC_CR_PLL1ON_Msk
            | RCC_CR_PLL2ON_Msk
            | RCC_CR_PLL3ON_Msk);

    while (READ_BIT(RCC->CR,
                    RCC_CR_PLL1RDY_Msk
                  | RCC_CR_PLL2RDY_Msk
                  | RCC_CR_PLL3RDY_Msk)) {}

    /*
     * Настроить:
     *   - Источник тактирования = HSE (24MHz)
     *   - DIVM1 = 12 (24MHz / 12 = 2MHz)
     *   - DIVM2 = 12 (24MHz / 12 = 2MHz)
     */
    WRITE_REG(RCC->PLLCKSELR,
              0x02 << RCC_PLLCKSELR_PLLSRC_Pos
            | 0x0C << RCC_PLLCKSELR_DIVM1_Pos
            | 0x0C << RCC_PLLCKSELR_DIVM2_Pos);

    /*
     * Настроить:
     *   - PLL1_RGE = 2MHz..4MHz
     *   - PLL1_VCO = VCOH (384MHz..1672MHz)
     *   - PLL2_RGE = 2MHz..4MHz
     *   - PLL2_VCO = VCOH (384MHz..1672MHz)
     */
    WRITE_REG(RCC->PLLCFGR,
              0x01 << RCC_PLLCFGR_PLL1RGE_Pos
            | RCC_PLLCFGR_PLL1VCOSEL_Msk
            | 0x01 << RCC_PLLCFGR_PLL2RGE_Pos
            | RCC_PLLCFGR_PLL2VCOSEL_Msk);

    /*
     * Настроить:
     *   - PLL1_DIVN = 300 (2MHz * 300 = 600MHz)
     *   - PLL1_DIVP = 1 (600MHz / 1 = 600MHz)
     */
    WRITE_REG(RCC->PLL1DIVR1,
             (0x12C - 1) << RCC_PLL1DIVR1_DIVN_Pos
           | (0x01  - 1) << RCC_PLL1DIVR1_DIVP_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1PEN_Msk);

    /*
     * Настроить:
     *   - PLL2_DIVN = 200 (2MHz * 200 = 400MHz)
     *   - PLL2_DIVS = 8 (400MHz / 8 = 50MHz)
     *   - PLL2_DIVT = 4 (400MHz / 4 = 100MHz)
     */
    WRITE_REG(RCC->PLL2DIVR1, (0xC8 - 1) << RCC_PLL2DIVR1_DIVN_Pos);

    WRITE_REG(RCC->PLL2DIVR2,
             (0x08 - 1) << RCC_PLL2DIVR2_DIVS_Pos
           | (0x04 - 1) << RCC_PLL2DIVR2_DIVT_Pos);

    SET_BIT(RCC->PLLCFGR,
            RCC_PLLCFGR_PLL2SEN_Msk
          | RCC_PLLCFGR_PLL2TEN_Msk);

    /* Включить PLLs */
    SET_BIT(RCC->CR,
            RCC_CR_PLL1ON_Msk
          | RCC_CR_PLL2ON_Msk);

    while (!READ_BIT(RCC->CR,
                     RCC_CR_PLL1RDY_Msk
                   | RCC_CR_PLL2RDY_Msk)) {}
    /* --------------------------------------------------------------------- */


    /* BUS ----------------------------------------------------------------- */

    /* Настроить делитель CPU = 1 (600MHz / 1 = 600MHz) */
    CLEAR_REG(RCC->CDCFGR);

    /* Настроить делитель AHB = 2 (600MHz / 2 = 300MHz) */
    WRITE_REG(RCC->BMCFGR, 0x08);

    /* Настроить делители APB1, APB2, APB4, APB5 = 2 (300MHz / 2 = 150MHz) */
    WRITE_REG(RCC->APBCFGR,
              0x04 << RCC_APBCFGR_PPRE1_Pos
            | 0x04 << RCC_APBCFGR_PPRE2_Pos
            | 0x04 << RCC_APBCFGR_PPRE4_Pos
            | 0x04 << RCC_APBCFGR_PPRE5_Pos);

    /* --------------------------------------------------------------------- */


    /* Установить системный источник тактирования = PLL1P */
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_SW_Msk,
               0x03 << RCC_CFGR_SW_Pos);

    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) !=
            0x03 << RCC_CFGR_SWS_Pos) {}
}
/* ------------------------------------------------------------------------- */
