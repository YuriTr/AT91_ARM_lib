
#ifndef AT45_H
#define AT45_H

#include "board.h"
#include <stdint.h>

#ifndef AT45_SPI_BAUD
//! Serial Clock Baud Rate for AT45
#define AT45_SPI_BAUD           2000000
#endif

//! WRITE ENABLE (WREN) command
#define AT25_WRITE_ENABLE   0x6
//! WRITE DISABLE (WRDI) command
#define AT25_WRITE_DISABLE  0x4
//! READ STATUS REGISTER (RDSR) command
#define AT25_STATUS_READ    0x5
//! WRITE STATUS REGISTER (WRSR) command
#define AT25_STATUS_WRITE   0x1


typedef struct {
    /// Size of EEPROM 
    uint16_t  EepromSize;
    /// The block write protection levels and
    uint16_t  BlockWPaddr[4];
    /// Identifier.
	const char *name;
} At25Desc;

#define AT25_CMD_BUFFER_SZ  8

typedef struct {

    /// Pointer to Spi Structure (SPI low level driver).
	Spid *pSpid;
    /// Current SPI command sent to the SPI low level driver.
	SpidCmd command;
    /// Pointer to the dataflash description.
	const At25Desc *pDesc;
    /// Buffer to store the current command (opcode + dataflash address.
	SpidData_t pCmdBuffer[AT25_CMD_BUFFER_SZ];

} At25;


#endif
