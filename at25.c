
#include "at45.h"

/// Number of dataflash which can be recognized.
#define NUMEEPROM    (sizeof(at25Devices) / sizeof(At25Desc))


static const At25Desc at25Devices[] = {
    {1024, { 0x3ff, 0x300, 0x200,0}, "AT25080"},
    {2048, { 0x7ff, 0x600, 0x400,0}, "AT25160"},
    {4096, { 0xfff, 0xc00, 0x800,0}, "AT25320"},
    {8192, {0x1fff,0x1800,0x1000,0}, "AT25640"},
};

unsigned char AT25_Configure(At25 *pAt25, Spid *pSpid, unsigned char spiCs)
{
    SpidCmd *pCommand;

    // Sanity checks
    ASSERT(pSpid, "AT25_Configure: pSpid is 0.\n\r");
    ASSERT(pAt25, "AT25_Configure: pAt25 is 0.\n\r");

    // Initialize the At25 instance
    pAt25->pSpid = pSpid;
    pAt25->pDesc = 0;
    memset(pAt25->pCmdBuffer, 0, sizeof(SpidData_t)*AT25_CMD_BUFFER_SZ );

 #if SPID_PCS_MODE == SPID_PCS_VAR
    SPID_ConfigureCS(pSpid, (spiCs>>2),AT91C_SPI_NCPHA | SPID_CSR_SCBR(BOARD_MCK,AT25_SPI_BAUD));//SPI_SCBR = 2, baudrate = MCK/SPI_SCBR
 #else
    //SPI_SCBR = 2, baudrate = MCK/SPI_SCBR
    //NCPHA: Clock Phase 1 = Data is captured on the leading edge of SPCK and changed on the following edge of SPCK. 
    //CPOL: Clock Polarity 0 = The inactive state value of SPCK is logic level zero. 
    SPID_ConfigureCS(pSpid, spiCs,AT91C_SPI_NCPHA | SPID_CSR_SCBR(BOARD_MCK,AT25_SPI_BAUD));
 #endif
    // Initialize the spidCmd structure
    pCommand = &(pAt25->command);
    pCommand->pCmd = pAt25->pCmdBuffer;
    pCommand->callback = 0;
    pCommand->pArgument = 0;
    pCommand->spiCs = spiCs;

    return 0;
}

/**------------------------------------------------------------------------------
* This function returns 1 if the At25 driver is busy (executing any command - SPID is busy)
* otherwise it returns 0.
* @param pAt25  Pointer to an At25 instance.
 ------------------------------------------------------------------------------*/
unsigned char AT25_IsBusy(At25 *pAt25)
{
    return SPID_IsBusy(pAt25->pSpid);
}

/**------------------------------------------------------------------------------
*  Reads status register of EEPROM and check Ready/Busy bit. 
*  @return This function returns 1 if the At25 EEPROM is busy (executing any command)
* otherwise it returns 0.
* @param pAt25  Pointer to an At25 instance.
 ------------------------------------------------------------------------------*/
unsigned char AT25_IsEepromBusy(At25 *pAt25)
{
    unsigned char b = SPID_IsBusy(pAt25->pSpid);
    SpidData_t status;
    //if spid busy -> AT25 driver busy too
    if (b) {
        return b;
    }
    //Send STATUS REGISTER READ command
    b = AT25_SendCommand(pAt25,AT25_STATUS_READ,1,&status,1,0,0,0);
    //wait the end of executing command
    while (SPID_IsBusy(pAt25->pSpid)) ;
    if ( AT25_STATUS_READY(status) ) 
        return 0;
    else 
        return 1;
}

