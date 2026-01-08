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

#include "stm32h7rsxx_hal_rcc.h"
#include "stm32h7rsxx_hal_systick.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define HAL_RCC_HSERDY_TIMEOUT      100
#define HAL_RCC_LSERDY_TIMEOUT      5000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_rcc_setup_hse(uint32_t state);

static void hal_rcc_setup_hse_css(uint32_t state);

static void hal_rcc_setup_lse(uint32_t state);

static void hal_rcc_setup_lse_css(uint32_t state);

static void hal_rcc_setup_pll1(uint32_t state);

static void hal_rcc_setup_pll2(uint32_t state);

static void hal_rcc_setup_pll3(uint32_t state);

static void hal_rcc_setup_pll_clock_source(uint32_t clksource);

static void hal_rcc_setup_pll_divm1(uint32_t div);

static void hal_rcc_setup_pll_divm2(uint32_t div);

static void hal_rcc_setup_pll_divm3(uint32_t div);

static void hal_rcc_pll1_init(const rcc_pll_t * pll);

static void hal_rcc_pll1_setup_vco(uint32_t vco);

static void hal_rcc_pll1_setup_rge(uint32_t rge);

static void hal_rcc_pll1_setup_divn(uint32_t div);

static void hal_rcc_pll1_setup_divp(uint32_t div);

static void hal_rcc_pll1_setup_divq(uint32_t div);

static void hal_rcc_pll1_setup_divr(uint32_t div);

static void hal_rcc_pll1_setup_divs(uint32_t div);

static void hal_rcc_pll1_setup_fractional(uint32_t fracn);

static void hal_rcc_pll2_init(const rcc_pll_t * pll);

static void hal_rcc_pll2_setup_vco(uint32_t vco);

static void hal_rcc_pll2_setup_rge(uint32_t rge);

static void hal_rcc_pll2_setup_divn(uint32_t div);

static void hal_rcc_pll2_setup_divp(uint32_t div);

static void hal_rcc_pll2_setup_divq(uint32_t div);

static void hal_rcc_pll2_setup_divr(uint32_t div);

static void hal_rcc_pll2_setup_divs(uint32_t div);

static void hal_rcc_pll2_setup_divt(uint32_t div);

static void hal_rcc_pll2_setup_fractional(uint32_t fracn);

static void hal_rcc_pll3_init(const rcc_pll_t * pll);

static void hal_rcc_pll3_setup_vco(uint32_t vco);

static void hal_rcc_pll3_setup_rge(uint32_t rge);

static void hal_rcc_pll3_setup_divn(uint32_t div);

static void hal_rcc_pll3_setup_divp(uint32_t div);

static void hal_rcc_pll3_setup_divq(uint32_t div);

static void hal_rcc_pll3_setup_divr(uint32_t div);

static void hal_rcc_pll3_setup_divs(uint32_t div);

static void hal_rcc_pll3_setup_fractional(uint32_t fracn);

static void hal_rcc_setup_cpu_div(uint32_t div);

static void hal_rcc_setup_ahb_div(uint32_t div);

static void hal_rcc_setup_apb1_div(uint32_t div);

static void hal_rcc_setup_apb2_div(uint32_t div);

static void hal_rcc_setup_apb4_div(uint32_t div);

static void hal_rcc_setup_apb5_div(uint32_t div);

static void hal_rcc_setup_system_clock_source(uint32_t clksource);

static void hal_rcc_setup_xspi1_clock_source(uint32_t clksource);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать RCC
 *
 * @param rcc_init Указатель на структуру данных инициализации RCC
 */
void hal_rcc_init(rcc_init_t * rcc_init)
{
    assert(rcc_init != NULL);

    /* Настроить HSE */
    hal_rcc_setup_hse(rcc_init->hse_enable);
    hal_rcc_setup_hse_css(rcc_init->hse_css_enable);

    /* Настроить LSE */
    hal_rcc_setup_lse(rcc_init->lse_enable);
    hal_rcc_setup_lse_css(rcc_init->lse_css_enable);

    /* Настроить PLL */
    hal_rcc_setup_pll1(HAL_DISABLE);
    hal_rcc_setup_pll2(HAL_DISABLE);
    hal_rcc_setup_pll3(HAL_DISABLE);
    hal_rcc_setup_pll_clock_source(rcc_init->pll_clksource);
    hal_rcc_setup_pll_divm1(rcc_init->pll_divm1);
    hal_rcc_setup_pll_divm2(rcc_init->pll_divm2);
    hal_rcc_setup_pll_divm3(rcc_init->pll_divm3);
    hal_rcc_pll1_init((rcc_pll_t *) &rcc_init->pll1);
    hal_rcc_pll2_init((rcc_pll_t *) &rcc_init->pll2);
    hal_rcc_pll3_init((rcc_pll_t *) &rcc_init->pll3);

    /* Настроить тактирование CPU, BUS и системный источник тактирования */
    hal_rcc_setup_cpu_div(rcc_init->cpu_div);
    hal_rcc_setup_ahb_div(rcc_init->ahb_div);
    hal_rcc_setup_apb1_div(rcc_init->apb1_div);
    hal_rcc_setup_apb2_div(rcc_init->apb2_div);
    hal_rcc_setup_apb4_div(rcc_init->apb4_div);
    hal_rcc_setup_apb5_div(rcc_init->apb5_div);
    hal_rcc_setup_system_clock_source(rcc_init->system_clksource);

    /* Настроить источник тактирования периферии */
    hal_rcc_setup_xspi1_clock_source(rcc_init->xspi1_clksource);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние HSE
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_rcc_setup_hse(uint32_t state)
{
    MODIFY_REG(RCC->CR,
               RCC_CR_HSEON_Msk,
               state << RCC_CR_HSEON_Pos);

    if (state == HAL_ENABLE) {
        uint32_t tickstart = hal_systick_get_ticks();
        while (!READ_BIT(RCC->CR, RCC_CR_HSERDY_Msk)) {
            if (hal_systick_get_ticks() - tickstart > HAL_RCC_HSERDY_TIMEOUT) {
                hal_rcc_hserdy_timeout_callback();
            }
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние HSE CSS
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_rcc_setup_hse_css(uint32_t state)
{
    MODIFY_REG(RCC->CR,
               RCC_CR_HSECSSON_Msk,
               state << RCC_CR_HSECSSON_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние LSE
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_rcc_setup_lse(uint32_t state)
{
    MODIFY_REG(RCC->BDCR,
               RCC_BDCR_LSEON_Msk,
               state << RCC_BDCR_LSEON_Pos);

    if (state == HAL_ENABLE) {
        uint32_t tickstart = hal_systick_get_ticks();
        while (!READ_BIT(RCC->BDCR, RCC_BDCR_LSERDY_Msk)) {
            if (hal_systick_get_ticks() - tickstart > HAL_RCC_LSERDY_TIMEOUT) {
                hal_rcc_lserdy_timeout_callback();
            }
        }
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние LSE CSS
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_rcc_setup_lse_css(uint32_t state)
{
    MODIFY_REG(RCC->BDCR,
               RCC_BDCR_LSECSSON_Msk,
               state << RCC_BDCR_LSECSSON_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние PLL1
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_rcc_setup_pll1(uint32_t state)
{
    MODIFY_REG(RCC->CR,
               RCC_CR_PLL1ON_Msk,
               state << RCC_CR_PLL1ON_Pos);

    while (READ_BIT(RCC->CR, RCC_CR_PLL1RDY_Msk) !=
            state << RCC_CR_PLL1RDY_Pos) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние PLL2
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_rcc_setup_pll2(uint32_t state)
{
    MODIFY_REG(RCC->CR,
               RCC_CR_PLL2ON_Msk,
               state << RCC_CR_PLL2ON_Pos);

    while (READ_BIT(RCC->CR, RCC_CR_PLL2RDY_Msk) !=
            state << RCC_CR_PLL2RDY_Pos) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить состояние PLL3
 *
 * @param state Состояние @ref hal_state_t
 */
static void hal_rcc_setup_pll3(uint32_t state)
{
    MODIFY_REG(RCC->CR,
               RCC_CR_PLL3ON_Msk,
               state << RCC_CR_PLL3ON_Pos);

    while (READ_BIT(RCC->CR, RCC_CR_PLL3RDY_Msk) !=
            state << RCC_CR_PLL3RDY_Pos) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить источник тактирования PLL
 *
 * @param clksource Источник тактирования PLL @ref rcc_pll_clock_source_t
 */
static void hal_rcc_setup_pll_clock_source(uint32_t clksource)
{
    MODIFY_REG(RCC->PLLCKSELR,
               RCC_PLLCKSELR_PLLSRC_Msk,
               clksource << RCC_PLLCKSELR_PLLSRC_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить значение делителя DIVM1
 *
 * @param div Делитель:
 *              0 - Выключен
 *              1 - Bypass
 *              2 - /2
 *              3 - /3
 *              ...
 *              63 - /63
 */
static void hal_rcc_setup_pll_divm1(uint32_t div)
{
    MODIFY_REG(RCC->PLLCKSELR,
               RCC_PLLCKSELR_DIVM1_Msk,
               div << RCC_PLLCKSELR_DIVM1_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить значение делителя DIVM2
 *
 * @param div Делитель:
 *              0 - Выключен
 *              1 - Bypass
 *              2 - /2
 *              3 - /3
 *              ...
 *              63 - /63
 */
static void hal_rcc_setup_pll_divm2(uint32_t div)
{
    MODIFY_REG(RCC->PLLCKSELR,
               RCC_PLLCKSELR_DIVM2_Msk,
               div << RCC_PLLCKSELR_DIVM2_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить значение делителя DIVM3
 *
 * @param div Делитель:
 *              0 - Выключен
 *              1 - Bypass
 *              2 - /2
 *              3 - /3
 *              ...
 *              63 - /63
 */
static void hal_rcc_setup_pll_divm3(uint32_t div)
{
    MODIFY_REG(RCC->PLLCKSELR,
               RCC_PLLCKSELR_DIVM3_Msk,
               div << RCC_PLLCKSELR_DIVM3_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать PLL1
 *
 * @param pll Указатель на структуру данных PLL
 */
static void hal_rcc_pll1_init(const rcc_pll_t * pll)
{
    assert(pll != NULL);

    hal_rcc_pll1_setup_vco(pll->vco);
    hal_rcc_pll1_setup_rge(pll->rge);
    hal_rcc_pll1_setup_divn(pll->divn);
    hal_rcc_pll1_setup_divp(pll->divp);
    hal_rcc_pll1_setup_divq(pll->divq);
    hal_rcc_pll1_setup_divr(pll->divr);
    hal_rcc_pll1_setup_divs(pll->divs);
    hal_rcc_pll1_setup_fractional(pll->fracn);

    hal_rcc_setup_pll1(pll->enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 VCO
 *
 * @param vco Диапазон VCO @ref enum rcc_pll_vco
 */
static void hal_rcc_pll1_setup_vco(uint32_t vco)
{
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLL1VCOSEL_Msk,
               vco << RCC_PLLCFGR_PLL1VCOSEL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 RGE
 *
 * @param rge Диапазон RGE @ref enum rcc_pll_rge
 */
static void hal_rcc_pll1_setup_rge(uint32_t rge)
{
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLL1RGE_Msk,
               rge << RCC_PLLCFGR_PLL1RGE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 DIVN
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll1_setup_divn(uint32_t div)
{
    if (div < 8 || div > 420) {
        return;
    }

    MODIFY_REG(RCC->PLL1DIVR1,
               RCC_PLL1DIVR1_DIVN_Msk,
               (div - 1) << RCC_PLL1DIVR1_DIVN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 DIVP
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll1_setup_divp( uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1PEN_Msk);

    if (div == 0 || div == 2 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL1DIVR1,
               RCC_PLL1DIVR1_DIVP_Msk,
               (div - 1) << RCC_PLL1DIVR1_DIVP_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1PEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 DIVQ
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll1_setup_divq(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1QEN_Msk);

    if (div == 0 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL1DIVR1,
               RCC_PLL1DIVR1_DIVQ_Msk,
               (div - 1) << RCC_PLL1DIVR1_DIVQ_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1QEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 DIVR
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll1_setup_divr(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1REN_Msk);

    if (div == 0 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL1DIVR1,
               RCC_PLL1DIVR1_DIVR_Msk,
               (div - 1) << RCC_PLL1DIVR1_DIVR_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1REN_Msk);

}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 DIVS
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll1_setup_divs(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1SEN_Msk);

    if (div == 0 || div > 8) {
        return;
    }

    MODIFY_REG(RCC->PLL1DIVR2,
               RCC_PLL1DIVR2_DIVS_Msk,
               (div - 1) << RCC_PLL1DIVR2_DIVS_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1SEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL1 Fractional
 *
 * @param fracn Значение Fractional
 */
static void hal_rcc_pll1_setup_fractional(uint32_t fracn)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN_Msk);

    MODIFY_REG(RCC->PLL1FRACR,
               RCC_PLL1FRACR_FRACN_Msk,
               fracn << RCC_PLL1FRACR_FRACN_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL1FRACEN_Msk);

    uint32_t tickstart = hal_systick_get_ticks();
    while (hal_systick_get_ticks() - tickstart < 1) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать PLL2
 *
 * @param pll Указатель на структуру данных PLL
 */
static void hal_rcc_pll2_init(const rcc_pll_t * pll)
{
    assert(pll != NULL);

    hal_rcc_pll2_setup_vco(pll->vco);
    hal_rcc_pll2_setup_rge(pll->rge);
    hal_rcc_pll2_setup_divn(pll->divn);
    hal_rcc_pll2_setup_divp(pll->divp);
    hal_rcc_pll2_setup_divq(pll->divq);
    hal_rcc_pll2_setup_divr(pll->divr);
    hal_rcc_pll2_setup_divs(pll->divs);
    hal_rcc_pll2_setup_divt(pll->divt);
    hal_rcc_pll2_setup_fractional(pll->fracn);

    hal_rcc_setup_pll2(pll->enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 VCO
 *
 * @param vco Диапазон VCO @ref rcc_pll_vco_t
 */
static void hal_rcc_pll2_setup_vco(uint32_t vco)
{
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLL2VCOSEL_Msk,
               vco << RCC_PLLCFGR_PLL2VCOSEL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 RGE
 *
 * @param rge Диапазон RGE @ref rcc_pll_rge_t
 */
static void hal_rcc_pll2_setup_rge(uint32_t rge)
{
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLL2RGE_Msk,
               rge << RCC_PLLCFGR_PLL2RGE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 DIVN
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll2_setup_divn(uint32_t div)
{
    if (div < 8 || div > 420) {
        return;
    }

    MODIFY_REG(RCC->PLL2DIVR1,
               RCC_PLL2DIVR1_DIVN_Msk,
               (div - 1) << RCC_PLL2DIVR1_DIVN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 DIVP
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll2_setup_divp(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2PEN_Msk);

    if (div == 0 || div == 2 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL2DIVR1,
               RCC_PLL2DIVR1_DIVP_Msk,
               (div - 1) << RCC_PLL2DIVR1_DIVP_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2PEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 DIVQ
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll2_setup_divq(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2QEN_Msk);

    if (div == 0 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL2DIVR1,
               RCC_PLL2DIVR1_DIVQ_Msk,
               (div - 1) << RCC_PLL2DIVR1_DIVQ_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2QEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 DIVR
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll2_setup_divr(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2REN_Msk);

    if (div == 0 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL2DIVR1,
               RCC_PLL2DIVR1_DIVR_Msk,
               (div - 1) << RCC_PLL2DIVR1_DIVR_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2REN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 DIVS
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll2_setup_divs(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2SEN_Msk);

    if (div == 0 || div > 8) {
        return;
    }

    MODIFY_REG(RCC->PLL2DIVR2,
               RCC_PLL2DIVR2_DIVS_Msk,
               (div - 1) << RCC_PLL2DIVR2_DIVS_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2SEN_Msk);

}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 DIVT
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll2_setup_divt(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2TEN_Msk);

    if (div == 0 || div > 8) {
        return;
    }

    MODIFY_REG(RCC->PLL2DIVR2,
               RCC_PLL2DIVR2_DIVT_Msk,
               (div - 1) << RCC_PLL2DIVR2_DIVT_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2TEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL2 Fractional
 *
 * @param fracn Значение Fractional
 */
static void hal_rcc_pll2_setup_fractional(uint32_t fracn)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN_Msk);

    MODIFY_REG(RCC->PLL2FRACR,
               RCC_PLL2FRACR_FRACN_Msk,
               fracn << RCC_PLL2FRACR_FRACN_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL2FRACEN_Msk);

    uint32_t tickstart = hal_systick_get_ticks();
    while (hal_systick_get_ticks() - tickstart < 1) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Инициализировать PLL3
 *
 * @param pll Указатель на структуру данных PLL
 */
static void hal_rcc_pll3_init(const rcc_pll_t * pll)
{
    assert(pll != NULL);

    hal_rcc_pll3_setup_vco(pll->vco);
    hal_rcc_pll3_setup_rge(pll->rge);
    hal_rcc_pll3_setup_divn(pll->divn);
    hal_rcc_pll3_setup_divp(pll->divp);
    hal_rcc_pll3_setup_divq(pll->divq);
    hal_rcc_pll3_setup_divr(pll->divr);
    hal_rcc_pll3_setup_divs(pll->divs);
    hal_rcc_pll3_setup_fractional(pll->fracn);

    hal_rcc_setup_pll3(pll->enable);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 VCO
 *
 * @param vco Диапазон VCO @ref rcc_pll_vco_t
 */
static void hal_rcc_pll3_setup_vco(uint32_t vco)
{
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLL3VCOSEL_Msk,
               vco << RCC_PLLCFGR_PLL3VCOSEL_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 RGE
 *
 * @param rge Диапазон RGE @ref rcc_pll_rge_t
 */
static void hal_rcc_pll3_setup_rge(uint32_t rge)
{
    MODIFY_REG(RCC->PLLCFGR,
               RCC_PLLCFGR_PLL3RGE_Msk,
               rge << RCC_PLLCFGR_PLL3RGE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 DIVN
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll3_setup_divn(uint32_t div)
{
    if (div < 8 || div > 420) {
        return;
    }

    MODIFY_REG(RCC->PLL3DIVR1,
               RCC_PLL3DIVR1_DIVN_Msk,
               (div - 1) << RCC_PLL3DIVR1_DIVN_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 DIVP
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll3_setup_divp(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3PEN_Msk);

    if (div == 0 || div == 2 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL3DIVR1,
               RCC_PLL3DIVR1_DIVP_Msk,
               (div - 1) << RCC_PLL3DIVR1_DIVP_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3PEN_Msk);

}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 DIVQ
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll3_setup_divq(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3QEN_Msk);

    if (div == 0 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL3DIVR1,
               RCC_PLL3DIVR1_DIVQ_Msk,
               (div - 1) << RCC_PLL3DIVR1_DIVQ_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3QEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 DIVR
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll3_setup_divr(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3REN_Msk);

    if (div == 0 || div > 128) {
        return;
    }

    MODIFY_REG(RCC->PLL3DIVR1,
               RCC_PLL3DIVR1_DIVR_Msk,
               (div - 1) << RCC_PLL3DIVR1_DIVR_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3REN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 DIVS
 *
 * @param div Значение делителя
 */
static void hal_rcc_pll3_setup_divs(uint32_t div)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3SEN_Msk);

    if (div == 0 || div > 8) {
        return;
    }

    MODIFY_REG(RCC->PLL3DIVR2,
               RCC_PLL3DIVR2_DIVS_Msk,
               (div - 1) << RCC_PLL3DIVR2_DIVS_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3SEN_Msk);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить PLL3 Fractional
 *
 * @param[in] fracn Значение Fractional
 */
static void hal_rcc_pll3_setup_fractional(uint32_t fracn)
{
    CLEAR_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3FRACEN_Msk);

    MODIFY_REG(RCC->PLL3FRACR,
               RCC_PLL3FRACR_FRACN_Msk,
               fracn << RCC_PLL3FRACR_FRACN_Pos);

    SET_BIT(RCC->PLLCFGR, RCC_PLLCFGR_PLL3FRACEN_Msk);

    uint32_t tickstart = hal_systick_get_ticks();
    while (hal_systick_get_ticks() - tickstart < 1);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить делитель CPU
 *
 * @param div Значение делителя @ref rcc_cpu_div_t
 */
static void hal_rcc_setup_cpu_div(uint32_t div)
{
    MODIFY_REG(RCC->CDCFGR,
               RCC_CDCFGR_CPRE_Msk,
               div << RCC_CDCFGR_CPRE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить делитель AHB
 *
 * @param div Значение делителя @ref rcc_ahb_div_t
 */
static void hal_rcc_setup_ahb_div(uint32_t div)
{
    MODIFY_REG(RCC->BMCFGR,
               RCC_BMCFGR_BMPRE_Msk,
               div << RCC_BMCFGR_BMPRE_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить делитель APB1
 *
 * @param div Значение делителя @ref rcc_apb_div_t
 */
static void hal_rcc_setup_apb1_div(uint32_t div)
{
    MODIFY_REG(RCC->APBCFGR,
               RCC_APBCFGR_PPRE1_Msk,
               div << RCC_APBCFGR_PPRE1_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить делитель APB2
 *
 * @param div Значение делителя @ref rcc_apb_div_t
 */
static void hal_rcc_setup_apb2_div(uint32_t div)
{
    MODIFY_REG(RCC->APBCFGR,
               RCC_APBCFGR_PPRE2_Msk,
               div << RCC_APBCFGR_PPRE2_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить делитель APB4
 *
 * @param div Значение делителя @ref rcc_apb_div_t
 */
static void hal_rcc_setup_apb4_div(uint32_t div)
{
    MODIFY_REG(RCC->APBCFGR,
               RCC_APBCFGR_PPRE4_Msk,
               div << RCC_APBCFGR_PPRE4_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить делитель APB5
 *
 * @param div Значение делителя @ref rcc_apb_div_t
 */
static void hal_rcc_setup_apb5_div(uint32_t div)
{
    MODIFY_REG(RCC->APBCFGR,
               RCC_APBCFGR_PPRE5_Msk,
               div << RCC_APBCFGR_PPRE5_Pos);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить системный источник тактирования
 *
 * @param clksource Источник тактирования @ref rcc_system_clock_source_t
 */
static void hal_rcc_setup_system_clock_source(uint32_t clksource)
{
    MODIFY_REG(RCC->CFGR,
               RCC_CFGR_SW_Msk,
               clksource << RCC_CFGR_SW_Pos);

    while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS_Msk) !=
            clksource << RCC_CFGR_SWS_Pos) {}
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить источник тактирования XSPI1
 *
 * @param clksource Источник тактирования @ref rcc_xspi1_clock_source_t
 */
static void hal_rcc_setup_xspi1_clock_source(uint32_t clksource)
{
    MODIFY_REG(RCC->CCIPR1,
               RCC_CCIPR1_XSPI1SEL_Msk,
               clksource << RCC_CCIPR1_XSPI1SEL_Pos);
}
/* ------------------------------------------------------------------------- */

__WEAK void hal_rcc_hserdy_timeout_callback(void)
{

}
/* ------------------------------------------------------------------------- */

__WEAK void hal_rcc_lserdy_timeout_callback(void)
{

}
/* ------------------------------------------------------------------------- */
