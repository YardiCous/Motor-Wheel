#ifndef BOARD_IO_MOCK_H
#define BOARD_IO_MOCK_H
#include <stdint.h>
uint32_t board_io_mock_read(uint32_t address);
void board_io_mock_write(uint32_t address, uint32_t value);
uint32_t board_io_mock_lock(void);
void board_io_mock_unlock(uint32_t mask);
#endif
