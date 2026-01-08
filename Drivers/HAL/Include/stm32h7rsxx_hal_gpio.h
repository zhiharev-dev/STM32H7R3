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

#ifndef STM32H7RSXX_HAL_GPIO_H_
#define STM32H7RSXX_HAL_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "stm32h7rsxx_hal_def.h"

/* Exported macros --------------------------------------------------------- */

/**
 * @brief Включить тактирование GPIOA
 */
#define HAL_GPIOA_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOAEN_Msk)

/**
 * @brief Включить тактирование GPIOB
 */
#define HAL_GPIOB_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOBEN_Msk)

/**
 * @brief Включить тактирование GPIOC
 */
#define HAL_GPIOC_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOCEN_Msk)

/**
 * @brief Включить тактирование GPIOD
 */
#define HAL_GPIOD_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIODEN_Msk)

/**
 * @brief Включить тактирование GPIOE
 */
#define HAL_GPIOE_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOEEN_Msk)

/**
 * @brief Включить тактирование GPIOF
 */
#define HAL_GPIOF_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOFEN_Msk)

/**
 * @brief Включить тактирование GPIOG
 */
#define HAL_GPIOG_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOGEN_Msk)

/**
 * @brief Включить тактирование GPIOH
 */
#define HAL_GPIOH_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOHEN_Msk)

/**
 * @brief Включить тактирование GPIOM
 */
#define HAL_GPIOM_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOMEN_Msk)

/**
 * @brief Включить тактирование GPION
 */
#define HAL_GPION_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIONEN_Msk)

/**
 * @brief Включить тактирование GPIOO
 */
#define HAL_GPIOO_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOOEN_Msk)

/**
 * @brief Включить тактирование GPIOP
 */
#define HAL_GPIOP_ENABLE_CLOCK() \
    SET_BIT(RCC->AHB4ENR, RCC_AHB4ENR_GPIOPEN_Msk)

/* Exported constants ------------------------------------------------------ */

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных GPIO
 */
typedef GPIO_TypeDef gpio_t;


/**
 * @brief Определение перечисления портов ввода-вывода GPIO
 */
typedef enum gpio_pin {
    GPIO_PIN0,
    GPIO_PIN1,
    GPIO_PIN2,
    GPIO_PIN3,
    GPIO_PIN4,
    GPIO_PIN5,
    GPIO_PIN6,
    GPIO_PIN7,
    GPIO_PIN8,
    GPIO_PIN9,
    GPIO_PIN10,
    GPIO_PIN11,
    GPIO_PIN12,
    GPIO_PIN13,
    GPIO_PIN14,
    GPIO_PIN15,
} gpio_pin_t;


/**
 * @brief Определение перечисления состояний порта ввода-вывода GPIO
 */
typedef enum gpio_state {
    GPIO_RESET,
    GPIO_SET,
} gpio_state_t;


/**
 * @brief Определение перечисления режмов работы порта ввода-вывода GPIO
 */
typedef enum gpio_mode {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_AF,
    GPIO_ANALOG,
} gpio_mode_t;


/**
 * @brief Определение перечисления типов порта вывода GPIO
 */
typedef enum gpio_output_type {
    GPIO_PUSH_PULL,
    GPIO_OPEN_DRAIN,
} gpio_output_type_t;


/**
 * @brief Определение перечисления скорости работы порта вывода GPIO
 */
typedef enum gpio_output_speed {
    GPIO_LOW_SPEED,
    GPIO_MEDIUM_SPEED,
    GPIO_HIGH_SPEED,
    GPIO_VERY_HIGH_SPEED,
} gpio_output_speed_t;


/**
 * @brief Определение перечисления подтяжки сигнала порта ввода-вывода GPIO
 */
typedef enum gpio_pupd {
    GPIO_NO_PULL,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
} gpio_pupd_t;


/**
 * @brief Определение структуры данных настройки GPIO
 */
typedef struct gpio_init {
    uint32_t mode;                              /*!< Режим работы @ref gpio_mode_t */

    uint32_t otype;                             /*!< Тип вывода @ref gpio_output_type_t */

    uint32_t ospeed;                            /*!< Скорость работы вывода @ref gpio_output_speed_t */

    uint32_t pupd;                              /*!< Подтяжка сигнала @ref gpio_pupd_t */

    uint32_t af;                                /*!< Альтернативная функция [0, 15] */
} gpio_init_t;


/**
 * @brief Определение структуры данных обработчика GPIO
 */
typedef struct gpio_handle {
    gpio_t * instance;                          /*!< Указатель на структуру данных GPIO */

    uint32_t pin;                               /*!< Номер порта ввода-вывода GPIO @ref gpio_pin_t */
} gpio_handle_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

void hal_gpio_init(gpio_handle_t * gpio_handle, gpio_init_t * gpio_init);

void hal_gpio_set_state(gpio_handle_t * gpio_handle, uint32_t state);

uint32_t hal_gpio_state(gpio_handle_t * gpio_handle);

void hal_gpio_toggle(gpio_handle_t * gpio_handle);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* STM32H7RSXX_HAL_GPIO_H_ */
