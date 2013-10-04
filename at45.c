/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include "at45.h"
#include <board.h>
#include "assert.h"

#include <string.h>

//------------------------------------------------------------------------------
//         Internal definitions
//------------------------------------------------------------------------------

/// Number of dataflash which can be recognized.
#define NUMDATAFLASH    (sizeof(at45Devices) / sizeof(At45Desc))

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------

/// indicate if the device is configured as binary page or not.
static unsigned char configuredBinaryPage;

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

static const At45Desc at45Devices[] = {
    {  512,  1, 264,   9, 0x0C, "AT45DB011D"},
    { 1024,  1, 264,   9, 0x14, "AT45DB021D"},
    { 2048,  1, 264,   9, 0x1C, "AT45DB041D"},
    { 4096,  1, 264,   9, 0x24, "AT45DB081D"},
    { 4096,  1, 528,  10, 0x2C, "AT45DB161D"},
    { 8192,  1, 528,  10, 0x34, "AT45DB321D"},
    { 8192,  1, 1056, 11, 0x3C, "AT45DB642D"},
    {16384,  1, 1056, 11, 0x10, "AT45DB1282"},
    {16384,  1, 2112, 12, 0x18, "AT45DB2562"},
    {32768,  1, 2112, 12, 0x20, "AT45DB5122"}
};

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

/**
* Initializes an AT45 instance and configures SPI chip select register.
* @return Always returns 0.
* @param pAt45  Pointer to the At45 instance to initialize.
* @param pSpid  Pointer to the underlying SPI driver.
* @param spiCs  Chip select value to connect to the At45.
*/
unsigned char AT45_Configure(At45 *pAt45, Spid *pSpid, unsigned char spiCs)
{
    SpidCmd *pCommand;

    // Sanity checks
    ASSERT(pSpid, "AT45_Configure: pSpid is 0.\n\r");
    ASSERT(pAt45, "AT45_Configure: pAt45 is 0.\n\r");

    // Initialize the At45 instance
    pAt45->pSpid = pSpid;
    pAt45->pDesc = 0;
    memset(pAt45->pCmdBuffer, 0, sizeof(SpidData_t)*8 );
    
 #if SPID_PCS_MODE == SPID_PCS_VAR
    SPID_ConfigureCS(pSpid, (spiCs>>2),AT91C_SPI_NCPHA | SPID_CSR_SCBR(BOARD_MCK,AT45_SPI_BAUD));//SPI_SCBR = 2, baudrate = MCK/SPI_SCBR
//    SPID_ConfigureCS(pSpid, (spiCs>>2),AT45_CSR(BOARD_MCK,AT45_SPI_BAUD));//SPI_SCBR = 2, baudrate = MCK/SPI_SCBR
 #else
    //SPI_SCBR = 2, baudrate = MCK/SPI_SCBR
    //NCPHA: Clock Phase 1 = Data is captured on the leading edge of SPCK and changed on the following edge of SPCK. 
    //CPOL: Clock Polarity 0 = The inactive state value of SPCK is logic level zero. 
    SPID_ConfigureCS(pSpid, spiCs,AT91C_SPI_NCPHA | SPID_CSR_SCBR(BOARD_MCK,AT45_SPI_BAUD));//SPI_SCBR = 2, baudrate = MCK/SPI_SCBR
//    SPID_ConfigureCS(pSpid, (spiCs),AT45_CSR(BOARD_MCK,AT45_SPI_BAUD));//SPI_SCBR = 2, baudrate = MCK/SPI_SCBR
 #endif
    // Initialize the spidCmd structure
    pCommand = &(pAt45->command);
    pCommand->pCmd = pAt45->pCmdBuffer;
    pCommand->callback = 0;
    pCommand->pArgument = 0;
    pCommand->spiCs = spiCs;

    return 0;
}

/**------------------------------------------------------------------------------
* This function returns 1 if the At45 driver is busy (executing any command - SPID is busy)
* otherwise it returns 0.
* @param pAt45  Pointer to an At45 instance.
 ------------------------------------------------------------------------------*/
unsigned char AT45_IsBusy(At45 *pAt45)
{
    return SPID_IsBusy(pAt45->pSpid);
}

/**------------------------------------------------------------------------------
*  Reads status register of DataFlash and check Ready/Busy bit. 
*  @return This function returns 1 if the At45 DataFlash is busy (executing any command)
* otherwise it returns 0.
* @param pAt45  Pointer to an At45 instance.
 ------------------------------------------------------------------------------*/
unsigned char AT45_IsDFBusy(At45 *pAt45)
{
    unsigned char b = SPID_IsBusy(pAt45->pSpid);
    SpidData_t status;
    //if spid busy -> AT45 driver busy too
    if (b) {
        return b;
    }
    //Send STATUS REGISTER READ command
    b = AT45_SendCommand(pAt45,AT45_STATUS_READ,1,&status,1,0,0,0);
    //wait the end of executing command
    while (SPID_IsBusy(pAt45->pSpid)) ;
    if ( AT45_STATUS_READY(status) ) 
        return 0;
    else 
        return 1;
}

/**
* Sends a command to the dataflash through the SPI. The command 
* is identified by its command code and the number of bytes to
* transfer (1 + number of address bytes + number of dummy
* bytes). If data needs to be received, then  a data buffer must
* be provided. This function does not block; its optional 
* callback will be invoked when the transfer completes.
* @param pAt45  Pointer to an At45 driver instance. 
* @param cmd Command code.
* @param cmdSize  Size of command code + address bytes + dummy 
bytes. 
* @param pData  Data buffer.
* @param dataSize  Number of data bytes to send/receive.
* @param address  Address at which the command is performed if
meaningful. 
* @param callback  Optional callback to invoke at end of
transfer. 
* @param pArgument  Optional parameter to the callback
function. 
*/ 
unsigned char AT45_SendCommand(
    At45 *pAt45,
    unsigned char cmd,
    unsigned char cmdSize,
    SpidData_t *pData,
    unsigned int dataSize,
    unsigned int address,
    SpidCallback callback,
    void *pArgument)
{
    SpidCmd *pCommand;
    const At45Desc *pDesc = pAt45->pDesc;
    unsigned int dfAddress = 0;

    // Sanity checks
    ASSERT(pAt45, "AT45_Command: pAt45 is 0.\n\r");
    ASSERT(pDesc || (cmd == AT45_STATUS_READ),
           "AT45_Command: Device has no descriptor, only STATUS_READ command allowed\n\r");

    // Check if the SPI driver is available
    if (AT45_IsBusy(pAt45)) {

        return AT45_ERROR_LOCK;
    }

    // Compute command pattern
    pAt45->pCmdBuffer[0] = cmd;

    // Add address bytes if necessary
    if (cmdSize > 1) {

        ASSERT(pDesc, "AT45_Command: No descriptor for dataflash.\n\r");
        if (!configuredBinaryPage) {
            dfAddress =
                ((address / (pDesc->pageSize)) << pDesc->pageOffset)
                 + (address % (pDesc->pageSize));
        }
        else {
            dfAddress = address;
        }
        // Write address bytes
        if (pDesc->pageNumber >= 16384) {

            pAt45->pCmdBuffer[1] = ((dfAddress & 0x0F000000) >> 24);
            pAt45->pCmdBuffer[2] = ((dfAddress & 0x00FF0000) >> 16);
            pAt45->pCmdBuffer[3] = ((dfAddress & 0x0000FF00) >> 8);
            pAt45->pCmdBuffer[4] = ((dfAddress & 0x000000FF) >> 0);
    
            if ((cmd != AT45_CONTINUOUS_READ) && (cmd != AT45_PAGE_READ)) {
    
                cmdSize++;
            }
        }
        else {

            pAt45->pCmdBuffer[1] = ((dfAddress & 0x00FF0000) >> 16);
            pAt45->pCmdBuffer[2] = ((dfAddress & 0x0000FF00) >> 8);
            pAt45->pCmdBuffer[3] = ((dfAddress & 0x000000FF) >> 0);
        }
    }

    // Update the SPI Transfer descriptors
    pCommand = &(pAt45->command);
    pCommand->cmdSize = cmdSize;
    pCommand->pData = pData;
    pCommand->dataSize = dataSize;
    pCommand->callback = callback;
    pCommand->pArgument = pArgument;

    // Send Command and data through the SPI
    if (SPID_SendCommand(pAt45->pSpid, pCommand)) {

        return AT45_ERROR_SPI;
    }

    return 0;
}

/**
  This function returns the At45Desc structure corresponding to
  the device connected It automatically initializes pAt45->pDesc
  field structure. This function shall be called by the
  application before AT45_SendCommand.
 
* @return Returns 0 if successful; Otherwise,
returns AT45_ERROR_LOCK if the At45 driver is in use or 
AT45_ERROR_SPI if there was an error with the SPI driver. 
 
* @param pAt45  Pointer to an AT45 driver instance. 
* @param status  Device status register value. 
*/
const At45Desc * AT45_FindDevice(At45 *pAt45, unsigned char status)
{
    unsigned int i;
    unsigned char id = AT45_STATUS_ID(status);

    // Check if status is all one; in which case, it is assumed that no device
    // is connected
    if (status == 0xFF) {

        return 0;
    }

    // Look in device array
    i = 0;
    pAt45->pDesc = 0;
    while ((i < NUMDATAFLASH) && !(pAt45->pDesc)) {

        if (at45Devices[i].id == id) {

            pAt45->pDesc = &(at45Devices[i]);
        }
        i++;
    }
    configuredBinaryPage = AT45_STATUS_BINARY(status);
    return pAt45->pDesc;
}

/**
 This function returns the pagesize corresponding to the device
connected 
* @param pAt45  Pointer to an AT45 driver instance. 
*/
unsigned int  AT45_PageSize(At45 *pAt45)
{
    unsigned int pagesize = pAt45->pDesc->pageSize;
    if(((pAt45->pDesc->hasBinaryPage) == 0) || !configuredBinaryPage){
        return pagesize;
    }
    return ((pagesize >> 8) << 8);
}
