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

#ifndef STM32H7RSXX_HAL_RCC_H_
#define STM32H7RSXX_HAL_RCC_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32h7rsxx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных RCC
 */
typedef RCC_TypeDef rcc_t;


/**
 * @brief Определение перечисления источника тактирования PLL
 */
typedef enum rcc_pll_clock_source {
    RCC_PLL_HSI_CLOCK,
    RCC_PLL_CSI_CLOCK,
    RCC_PLL_HSE_CLOCK,
    RCC_PLL_NOT_CLOCK,
} rcc_pll_clock_source_t;


/**
 * @brief Определение перечисления диапазонов PLL VCO
 */
typedef enum rcc_pll_vco {
    RCC_PLL_VCOH,                               /*!< 384MHz - 1672MHz (Fref1_ck = 2MHz - 16MHz) */
    RCC_PLL_VCOL,                               /*!< 150MHz - 420MHz (Fref1_ck = 1MHz - 2MHz) */
} rcc_pll_vco_t;


/**
 * @brief Определение перечисления диапазонов PLL RGE
 */
typedef enum rcc_pll_rge {
    RCC_PLL_RGE_1_2MHZ,
    RCC_PLL_RGE_2_4MHZ,
    RCC_PLL_RGE_4_8MHZ,
    RCC_PLL_RGE_8_16MHZ,
} rcc_pll_rge_t;


/**
 * @brief Определение перечисления делителей CPU
 */
typedef enum rcc_cpu_div {
    RCC_CPU_DIV1,
    RCC_CPU_DIV2 = 0x08,
    RCC_CPU_DIV4,
    RCC_CPU_DIV8,
    RCC_CPU_DIV16,
    RCC_CPU_DIV64,
    RCC_CPU_DIV128,
    RCC_CPU_DIV256,
    RCC_CPU_DIV512,
} rcc_cpu_div_t;


/**
 * @brief Определение перечисления делителей AHB
 */
typedef enum rcc_ahb_div {
    RCC_AHB_DIV1,
    RCC_AHB_DIV2 = 0x08,
    RCC_AHB_DIV4,
    RCC_AHB_DIV8,
    RCC_AHB_DIV16,
    RCC_AHB_DIV64,
    RCC_AHB_DIV128,
    RCC_AHB_DIV256,
    RCC_AHB_DIV512,
} rcc_ahb_div_t;


/**
 * @brief Определение перечисления делителей APB
 */
typedef enum rcc_apb_div {
    RCC_APB_DIV1,
    RCC_APB_DIV2 = 0x04,
    RCC_APB_DIV4,
    RCC_APB_DIV8,
    RCC_APB_DIV16,
} rcc_apb_div_t;


/**
 * @brief Определение перечисления системных источников тактирования
 */
typedef enum rcc_system_clock_source {
    RCC_HSI_SYSTEM_CLOCK,
    RCC_CSI_SYSTEM_CLOCK,
    RCC_HSE_SYSTEM_CLOCK,
    RCC_PLL1P_SYSTEM_CLOCK,
} rcc_system_clock_source_t;


/**
 * @brief Определение структуры данных PLL
 */
typedef struct rcc_pll {
    uint32_t enable;                            /*!< Состояние PLL @ref hal_state_t  */

    uint32_t vco;                               /*!< Диапазон PLL VCO @ref rcc_pll_vco_t */

    uint32_t rge;                               /*!< Диапазон PLL RGE @ref rcc_pll_rge_t */

    uint32_t divn;                              /*!< Множитель PLL DIVN: 8 <= DIVN <= 420 */

    uint32_t divp;                              /*!< Делитель PLL DIVP: 1 <= DIVP 128 */

    uint32_t divq;                              /*!< Делитель PLL DIVQ: 1 <= DIVQ 128 */

    uint32_t divr;                              /*!< Делитель PLL DIVR: 1 <= DIVR 128 */

    uint32_t divs;                              /*!< Делитель PLL DIVS: 1 <= DIVS 8 */

    uint32_t divt;                              /*!< Делитель PLL DIVR: 1 <= DIVT 8 */

    uint32_t fracn;                             /*!< Дробная часть коэффициента умножения для PLL VCO */
} rcc_pll_t;


/**
 * @brief Определение структуры данных инициализации RCC
 */
typedef struct rcc_init {
    uint32_t hse_enable;                        /*!< Состояние HSE @ref hal_state_t */

    uint32_t hse_css_enable;                    /*!< Состояние HSE CSS @ref hal_state_t */

    uint32_t lse_enable;                        /*!< Состояние LSE @ref hal_state_t */

    uint32_t lse_css_enable;                    /*!< Состояние LSE CSS @ref hal_state_t */

    uint32_t pll_clksource;                     /*!< Источник тактирования PLL @ref rcc_pll_clock_source_t */

    uint32_t pll_divm1;                         /*!< Делитель PLL DIVM1 */

    uint32_t pll_divm2;                         /*!< Делитель PLL DIVM2 */

    uint32_t pll_divm3;                         /*!< Делитель PLL DIVM3 */

    rcc_pll_t pll1;                             /*!< Данные PLL1 */

    rcc_pll_t pll2;                             /*!< Данные PLL2 */

    rcc_pll_t pll3;                             /*!< Данные PLL3 */

    uint32_t cpu_div;                           /*!< Делитель CPU @ref rcc_cpu_div_t */

    uint32_t ahb_div;                           /*!< Делитель BUS Matrix @ref rcc_ahb_div_t */

    uint32_t apb1_div;                          /*!< Делитель APB1 @ref rcc_apb_div_t */

    uint32_t apb2_div;                          /*!< Делитель APB2 @ref rcc_apb_div_t */

    uint32_t apb4_div;                          /*!< Делитель APB4 @ref rcc_apb_div_t */

    uint32_t apb5_div;                          /*!< Делитель APB5 @ref rcc_apb_div_t */

    uint32_t system_clksource;                  /*!< Системный источник тактирования @ref rcc_system_clock_source_t */
} rcc_init_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_rcc_init(rcc_init_t * rcc_init);

/* Exported callback function prototypes ----------------------------------- */

__WEAK void hal_rcc_hserdy_timeout_callback(void);

__WEAK void hal_rcc_lserdy_timeout_callback(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_RCC_H_ */
