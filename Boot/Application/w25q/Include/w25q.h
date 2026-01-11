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

#ifndef W25Q_H_
#define W25Q_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ---------------------------------------------------------------- */

#include "main.h"
#include "systick.h"
#include "xspi.h"

/* Exported macros --------------------------------------------------------- */

/* Exported constants ------------------------------------------------------ */

#define W25Q_MANUFACTURER_ID                    0xEF

#define W25Q16_DEVICE_ID                        0x14
#define W25Q32_DEVICE_ID                        0x15
#define W25Q64_DEVICE_ID                        0x16
#define W25Q128_DEVICE_ID                       0x17
#define W25Q256_DEVICE_ID                       0x18

#define W25Q16_JEDEC_ID                         0x4015
#define W25Q32_JEDEC_ID                         0x4016
#define W25Q64_JEDEC_ID                         0x4017
#define W25Q128_JEDEC_ID                        0x4018
#define W25Q256_JEDEC_ID                        0x4019

#define W25Q_WRITE_ENABLE                       0x06
#define W25Q_WRITE_DISABLE                      0x04
#define W25Q_READ_SR1                           0x05
#define W25Q_READ_SR2                           0x35
#define W25Q_WRITE_SR1                          0x01
#define W25Q_WRITE_SR2                          0x31
#define W25Q_PAGE_PROGRAM                       0x02
#define W25Q_QUAD_PAGE_PROGRAM                  0x32
#define W25Q_BLOCK_64KB_ERASE                   0xD8
#define W25Q_BLOCK_32KB_ERASE                   0x52
#define W25Q_SECTOR_4KB_ERASE                   0x20
#define W25Q_CHIP_ERASE                         0xC7
#define W25Q_SUSPEND_ERASE                      0x75
#define W25Q_RESUME_ERASE                       0x7A
#define W25Q_POWER_DOWN                         0xB9
#define W25Q_HIGH_PERFOMANCE_MODE               0xA3
#define W25Q_CONT_READ_MODE_RESET               0xFF
#define W25Q_RELEASE_POWER_DOWN_HPM_DEVICE_ID   0xAB
#define W25Q_MFR_DEVICE_ID                      0x90
#define W25Q_READ_UNIQUE_ID                     0x4B
#define W25Q_JEDEC_ID                           0x9F
#define W25Q_READ_DATA                          0x03
#define W25Q_FAST_READ                          0x0B
#define W25Q_FAST_READ_DOUT                     0x3B
#define W25Q_FAST_READ_DIO                      0xBB
#define W25Q_FAST_READ_QOUT                     0x6B
#define W25Q_FAST_READ_QIO                      0xEB
#define W25Q_OCTAL_WORD_READ_QIO                0xE3

#define W25Q_PAGE_SIZE                          0x100
#define W25Q_SECTOR_SIZE                        0x1000
#define W25Q_BLOCK_SIZE                         0x10000

#define W25Q_OK                                 0
#define W25Q_ERROR                             -1

/* Exported types ---------------------------------------------------------- */

/**
 * @brief Определение структуры данных W25Q
 */
typedef struct w25q {
    XSPI_TypeDef * xspi;                        /*!< Указатель на структуру данных XSPI */
} w25q_t;

/* Exported variables ------------------------------------------------------ */

/* Exported function prototypes -------------------------------------------- */

int32_t w25q_init(void);

int32_t w25q_setup_memory_mapped_mode(void);

/* Exported callback function prototypes ----------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* W25Q_H_ */
