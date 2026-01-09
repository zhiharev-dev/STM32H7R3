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

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать RCC
 */
void rcc_init(void)
{
    rcc_init_t rcc_init = {
        .hse_enable = HAL_ENABLE,
        .hse_css_enable = HAL_ENABLE,
        .lse_enable = HAL_ENABLE,
        .lse_css_enable = HAL_ENABLE,

        .pll_clksource = RCC_PLL_HSE_CLOCK,             /* HSE = 24MHz */
        .pll_divm1 = 12,                                /* DIVM1 = 24MHz / 12 = 2MHz */
        .pll_divm2 = 12,                                /* DIVM2 = 24MHz / 12 = 2MHz */

        .pll1.enable = HAL_ENABLE,
        .pll1.vco = RCC_PLL_VCOH,
        .pll1.rge = RCC_PLL_RGE_1_2MHZ,
        .pll1.divn = 300,                               /* DIVN = 2MHz * 300 = 600MHz */
        .pll1.divp = 1,                                 /* DIVP = 600MHz / 1 = 600MHz */

        .pll2.enable = HAL_ENABLE,
        .pll2.vco = RCC_PLL_VCOH,
        .pll2.rge = RCC_PLL_RGE_1_2MHZ,
        .pll2.divn = 200,                               /* DIVN = 2MHz * 200 = 400MHz */
        .pll2.divt = 4,                                 /* DIVT = 400MHz / 4 = 100MHz */

        .cpu_div = RCC_CPU_DIV1,
        .ahb_div = RCC_AHB_DIV2,
        .apb1_div = RCC_APB_DIV2,
        .apb2_div = RCC_APB_DIV2,
        .apb4_div = RCC_APB_DIV2,
        .apb5_div = RCC_APB_DIV2,
        .system_clksource = RCC_PLL1P_SYSTEM_CLOCK,

        .xspi1_clksource = RCC_XSPI1_PLL2T_CLOCK,
    };

    hal_rcc_init(&rcc_init);
}
/* ------------------------------------------------------------------------- */

void hal_rcc_hserdy_timeout_callback(void)
{
    error();
}
/* ------------------------------------------------------------------------- */

void hal_rcc_lserdy_timeout_callback(void)
{
    error();
}
/* ------------------------------------------------------------------------- */
