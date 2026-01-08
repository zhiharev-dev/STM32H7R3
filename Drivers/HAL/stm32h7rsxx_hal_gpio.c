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

#include "stm32h7rsxx_hal_gpio.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

/* Private function prototypes --------------------------------------------- */

static void hal_gpio_setup_mode(gpio_t * instance, uint32_t pin, uint32_t mode);

static void hal_gpio_setup_output_type(gpio_t * instance, uint32_t pin, uint32_t otype);

static void hal_gpio_setup_output_speed(gpio_t * instance, uint32_t pin, uint32_t ospeed);

static void hal_gpio_setup_pupd(gpio_t * instance, uint32_t pin, uint32_t pupd);

static void hal_gpio_setup_af(gpio_t * instance, uint32_t pin, uint32_t af);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать GPIO
 *
 * @param gpio_handle Указатель на структуру данных обработчика GPIO
 * @param gpio_init Указатель на структуру данных инициализации GPIO
 */
void hal_gpio_init(gpio_handle_t * gpio_handle, gpio_init_t * gpio_init)
{
    assert(gpio_handle != NULL);
    assert(gpio_handle->instance != NULL);
    assert(gpio_init != NULL);

    hal_gpio_setup_mode(gpio_handle->instance, gpio_handle->pin, gpio_init->mode);
    hal_gpio_setup_output_type(gpio_handle->instance, gpio_handle->pin, gpio_init->otype);
    hal_gpio_setup_output_speed(gpio_handle->instance, gpio_handle->pin, gpio_init->ospeed);
    hal_gpio_setup_pupd(gpio_handle->instance, gpio_handle->pin, gpio_init->pupd);
    hal_gpio_setup_af(gpio_handle->instance, gpio_handle->pin, gpio_init->af);
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить режим работы GPIO
 *
 * @param instance Указатель на структуру данных GPIO
 * @param pin Номер порта ввода-вывода GPIO @ref gpio_pin_t
 * @param mode Режим работы GPIO @ref gpio_mode_t
 */
static void hal_gpio_setup_mode(gpio_t * instance, uint32_t pin, uint32_t mode)
{
    MODIFY_REG(instance->MODER,
               HAL_BITMASK(0x03, pin * 2),
               HAL_BITMASK(mode, pin * 2));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить тип вывода GPIO
 *
 * @param instance Указатель на структуру данных GPIO
 * @param pin Номер порта ввода-вывода GPIO @ref gpio_pin_t
 * @param otype Тип вывода GPIO @ref gpio_output_type_t
 */
static void hal_gpio_setup_output_type(gpio_t * instance, uint32_t pin, uint32_t otype)
{
    MODIFY_REG(instance->OTYPER,
               HAL_BITMASK(0x01, pin * 1),
               HAL_BITMASK(otype, pin * 1));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить скорость работы вывода GPIO
 *
 * @param instance Указатель на структуру данных GPIO
 * @param pin Номер порта ввода-вывода GPIO @ref gpio_pin_t
 * @param ospeed Скорость работы вывода GPIO @ref gpio_otuput_speed_t
 */
static void hal_gpio_setup_output_speed(gpio_t * instance, uint32_t pin, uint32_t ospeed)
{
    MODIFY_REG(instance->OSPEEDR,
               HAL_BITMASK(0x03, pin * 2),
               HAL_BITMASK(ospeed, pin * 2));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить подтяжку сигнала GPIO
 *
 * @param instance Указатель на структуру данных GPIO
 * @param pin Номер порта ввода-вывода GPIO @ref gpio_pin_t
 * @param pupd Подтяжка сигнала GPIO @ref gpio_pupd_t
 */
static void hal_gpio_setup_pupd(gpio_t * instance, uint32_t pin, uint32_t pupd)
{
    MODIFY_REG(instance->PUPDR,
               HAL_BITMASK(0x03, pin * 2),
               HAL_BITMASK(pupd, pin * 2));
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить альтернативную функцию GPIO
 *
 * @param instance Указатель на структуру данных GPIO
 * @param pin Номер порта ввода-вывода GPIO @ref gpio_pin_t
 * @param af Номер альтернативной функции GPIO
 */
static void hal_gpio_setup_af(gpio_t * instance, uint32_t pin, uint32_t af)
{
    if (pin < GPIO_PIN8) {
        MODIFY_REG(instance->AFR[0],
                   HAL_BITMASK(0x0F, pin * 4),
                   HAL_BITMASK(af, pin * 4));
    } else {
        MODIFY_REG(instance->AFR[1],
                   HAL_BITMASK(0x0F, (pin - 8) * 4),
                   HAL_BITMASK(af, (pin - 8) * 4));
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Установить состояние порта ввода-вывода GPIO
 *
 * @param gpio_handle Указатель на структуру данных обработчика GPIO
 * @param state Состояние порта ввода-вывода GPIO @ref gpio_state_t
 */
void hal_gpio_set_state(gpio_handle_t * gpio_handle, uint32_t state)
{
    assert(gpio_handle != NULL);
    assert(gpio_handle->instance != NULL);

    if (state == GPIO_SET) {
        SET_BIT(gpio_handle->instance->BSRR, HAL_BITMASK(0x01, gpio_handle->pin));
    } else {
        SET_BIT(gpio_handle->instance->BSRR, HAL_BITMASK(0x10000, gpio_handle->pin));
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Получить состояние порта ввода-вывода GPIO
 *
 * @param gpio_handle Указатель на структуру данных обработчика GPIO
 *
 * @return Состояние порта ввода-вывода GPIO @ref gpio_state_t
 */
uint32_t hal_gpio_state(gpio_handle_t * gpio_handle)
{
    assert(gpio_handle != NULL);
    assert(gpio_handle->instance != NULL);

    return READ_BIT(gpio_handle->instance->IDR, HAL_BITMASK(0x01, gpio_handle->pin)) ?
            GPIO_SET : GPIO_RESET;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Изменить состояние порта ввода-вывода GPIO
 *
 * @param gpio_handle Указатель на структуру данных обработчика GPIO
 */
void hal_gpio_toggle(gpio_handle_t * gpio_handle)
{
    assert(gpio_handle != NULL);
    assert(gpio_handle->instance != NULL);

    XOR_BIT(gpio_handle->instance->ODR, HAL_BITMASK(0x01, gpio_handle->pin));
}
/* ------------------------------------------------------------------------- */
