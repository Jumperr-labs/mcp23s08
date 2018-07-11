#pragma once

#define SPI_MODE_0 (1U)
#define SPI_MODE_1 (1U << 1)
#define SPI_MODE_2 (1U << 2)
#define SPI_MODE_3 (1U << 3)

#define SPI_MODE_CPOL0_CPHA0 SPI_MODE_0
#define SPI_MODE_CPOL0_CPHA1 SPI_MODE_1
#define SPI_MODE_CPOL1_CPHA0 SPI_MODE_2
#define SPI_MODE_CPOL1_CPHA1 SPI_MODE_3

enum BitOrder {
    BIT_ORDER_UNDEFINED = -1,
    MSB_FIRST,
    LSB_FIRST
};
