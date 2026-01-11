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

#include "w25q.h"

/* Private macros ---------------------------------------------------------- */

/* Private constants ------------------------------------------------------- */

#define W25Q_XSPI_TIMEOUT       5000

/* Private types ----------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */

w25q_t w25q = {
    .xspi = XSPI1,
};

/* Private function prototypes --------------------------------------------- */

static int32_t w25q_read_jedec_id(void * dest);

static uint32_t w25q_ticks(void);

/* Private user code ------------------------------------------------------- */

/**
 * @brief Инициализировать W25Q
 *
 * @return Статус:
 *           - W25Q_ERROR
 *           - W25Q_OK
 */
int32_t w25q_init(void)
{
    assert(w25q.xspi != NULL);

    /* Прочитать и проверить идентификатор памяти */
    uint8_t id[3];

    if (w25q_read_jedec_id(id) != W25Q_OK) {
        return W25Q_ERROR;
    } else if (id[0] != W25Q_MANUFACTURER_ID) {
        return W25Q_ERROR;
    } else {
        return W25Q_OK;
    }
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Прочитать JEDEC ID
 *
 * @param dest Указатель на данные
 *
 * @return Статус:
 *           - W25Q_ERROR
 *           - W25Q_OK
 */
static int32_t w25q_read_jedec_id(void * dest)
{
    assert(dest != NULL);

    uint32_t tickstart = w25q_ticks();

    /* Размер данных */
    uint32_t data_size = 3;
    /* Указатель на данные */
    uint8_t * pdata = (uint8_t *) dest;
    /* Указатель на данные XSPI */
    volatile uint8_t * DR = (uint8_t *) &w25q.xspi->DR;

    /* Ожидание готовности XSPI */
    while (READ_BIT(w25q.xspi->SR, XSPI_SR_BUSY_Msk)) {
        if (w25q_ticks() - tickstart > W25Q_XSPI_TIMEOUT) {
            return W25Q_ERROR;
        }
    }

    /* Настроить Functional Mode */
    MODIFY_REG(w25q.xspi->CR,
               XSPI_CR_FMODE_Msk,
               0x01 << XSPI_CR_FMODE_Pos);

    /* Настроить DLR */
    WRITE_REG(w25q.xspi->DLR, data_size - 1);

    /* Настроить TCR */
    CLEAR_REG(w25q.xspi->TCR);

    /* Настроить CCR */
    WRITE_REG(w25q.xspi->CCR,
              0x01 << XSPI_CCR_IMODE_Pos
            | 0x01 << XSPI_CCR_DMODE_Pos);

    /* Настроить IR */
    WRITE_REG(w25q.xspi->IR, W25Q_JEDEC_ID);

    /* Принять данные */
    while (data_size-- > 0) {
        /* Ожидание возможности приема данных */
        while (!READ_BIT(w25q.xspi->SR,
                         XSPI_SR_FTF_Msk
                       | XSPI_SR_TCF_Msk)) {
            if (w25q_ticks() - tickstart > W25Q_XSPI_TIMEOUT) {
                return W25Q_ERROR;
            }
        }

        /* Записать принятые данные */
        *pdata++ = *DR;
    }

    /* Ожидание завершения операции */
    while (!READ_BIT(w25q.xspi->SR, XSPI_SR_TCF_Msk)) {
        if (w25q_ticks() - tickstart > W25Q_XSPI_TIMEOUT)
            return W25Q_ERROR;
    }

    /* Очистить статус завершения операции */
    SET_BIT(w25q.xspi->FCR, XSPI_FCR_CTCF_Msk);

    return W25Q_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Настроить Memory Mapped Mode
 *
 * @return Статус:
 *           - W25Q_ERROR
 *           - W25Q_OK
 */
int32_t w25q_setup_memory_mapped_mode(void)
{
    assert(w25q.xspi != NULL);

    uint32_t tickstart = w25q_ticks();

    /* Ожидание готовности XSPI */
    while (READ_BIT(w25q.xspi->SR, XSPI_SR_BUSY_Msk)) {
        if (w25q_ticks() - tickstart > W25Q_XSPI_TIMEOUT) {
            return W25Q_ERROR;
        }
    }

    /* Настроить Functional Mode */
    MODIFY_REG(w25q.xspi->CR,
               XSPI_CR_FMODE_Msk,
               0x03 << XSPI_CR_FMODE_Pos);

    /* Настоить TCR */
    WRITE_REG(w25q.xspi->TCR, 0x04 << XSPI_TCR_DCYC_Pos);

    /* Настроить CCR */
    WRITE_REG(w25q.xspi->CCR,
              0x01 << XSPI_CCR_IMODE_Pos
            | 0x03 << XSPI_CCR_ADMODE_Pos
            | 0x02 << XSPI_CCR_ADSIZE_Pos
            | 0x03 << XSPI_CCR_ABMODE_Pos
            | 0x03 << XSPI_CCR_DMODE_Pos);

    /* Настроить IR */
    WRITE_REG(w25q.xspi->IR, W25Q_FAST_READ_QIO);

    /* Настроить ABR */
    WRITE_REG(w25q.xspi->ABR, 0xFF);

    /* Настроить WTCR */
    CLEAR_REG(w25q.xspi->WTCR);

    /* Настроить WCCR */
    WRITE_REG(w25q.xspi->WCCR,
              0x01 << XSPI_CCR_IMODE_Pos
            | 0x01 << XSPI_CCR_ADMODE_Pos
            | 0x02 << XSPI_CCR_ADSIZE_Pos
            | 0x03 << XSPI_CCR_DMODE_Pos);

    /* Настроить WIR */
    WRITE_REG(w25q.xspi->WIR, W25Q_QUAD_PAGE_PROGRAM);

    /* Ожидание готовности XSPI */
    while (READ_BIT(w25q.xspi->SR, XSPI_SR_BUSY_Msk)) {
        if (systick_get_ticks() - tickstart > W25Q_XSPI_TIMEOUT) {
            return W25Q_ERROR;
        }
    }

    return W25Q_OK;
}
/* ------------------------------------------------------------------------- */

/**
 * @brief Получить значение системного таймера
 *
 * @return Значение системного таймера (мс)
 */
static inline uint32_t w25q_ticks(void)
{
    return systick_get_ticks();
}
/* ------------------------------------------------------------------------- */
