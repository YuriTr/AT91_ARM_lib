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

#include "spid.h"
#include <board.h>
#include "assert.h"

//------------------------------------------------------------------------------
//         Macros
//------------------------------------------------------------------------------

/// Write PMC register
#define WRITE_PMC(pPmc, regName, value) pPmc->regName = (value)

/// Write SPI register
#define WRITE_SPI(pSpi, regName, value) pSpi->regName = (value)

/// Read SPI registers
#define READ_SPI(pSpi, regName) (pSpi->regName)

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

/**------------------------------------------------------------------------------
* Initializes the Spid structure and the corresponding SPI hardware.
* @return Always returns 0.
* @param pSpid  Pointer to a Spid instance.
* @param pSpiHw  Associated SPI peripheral.
* @param spiId  SPI peripheral identifier.
*/
unsigned char SPID_Configure(Spid *pSpid, AT91S_SPI *pSpiHw, unsigned char spiId)
{
    // Initialize the SPI structure
    pSpid->pSpiHw = pSpiHw;
    pSpid->spiId  = spiId;
    pSpid->pCurrentCommand = 0;

    #ifdef __RTX
    os_mut_init(  pSpid->SpidMutex);
    #else
    pSpid->semaphore = 1;
    #endif
    // Enable the SPI clock
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCER, (1 << pSpid->spiId));
    
    // Execute a software reset of the SPI twice
    WRITE_SPI(pSpiHw, SPI_CR, AT91C_SPI_SWRST);
    WRITE_SPI(pSpiHw, SPI_CR, AT91C_SPI_SWRST);

#if SPID_PCS_MODE == SPID_PCS_VAR
    // Configure SPI in Master Mode with No CS selected !!!
    WRITE_SPI(pSpiHw, SPI_MR, AT91C_SPI_MSTR | AT91C_SPI_MODFDIS | AT91C_SPI_PCSDEC | AT91C_SPI_PCS | AT91C_SPI_PS_VARIABLE);
#else
    // Configure SPI in Master Mode with No CS selected !!!
    WRITE_SPI(pSpiHw, SPI_MR, AT91C_SPI_MSTR | AT91C_SPI_MODFDIS | AT91C_SPI_PCS);
#endif
    
    // Set SCBR to 0xff. 
    // Programming the SCBR field at 0 is forbidden. Triggering a transfer while SCBR is at 0 can lead to unpredictable results.
    SPID_ConfigureCS(pSpid,0,AT91C_SPI_SCBR);
    SPID_ConfigureCS(pSpid,1,AT91C_SPI_SCBR);
    SPID_ConfigureCS(pSpid,2,AT91C_SPI_SCBR);
    SPID_ConfigureCS(pSpid,3,AT91C_SPI_SCBR);

    // Disable the PDC transfer    
    WRITE_SPI(pSpiHw, SPI_PTCR, AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);

    // Enable the SPI
    WRITE_SPI(pSpiHw, SPI_CR, AT91C_SPI_SPIEN);

    // Disable the SPI clock
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCDR, (1 << pSpid->spiId));
    
    return 0;
}

/**------------------------------------------------------------------------------
* Configures the parameters for the device corresponding to the cs.
* @param pSpid  Pointer to a Spid instance.
* @param cs  number corresponding to the SPI chip select.
* @param csr  SPI_CSR value to setup.
*/
void SPID_ConfigureCS(Spid *pSpid, unsigned char cs, unsigned int csr)
{
    AT91S_SPI *pSpiHw = pSpid->pSpiHw;
    ASSERT(cs<4, "SPID_ConfigureCS: Wrong CS value: %d",cs);
    WRITE_SPI(pSpiHw, SPI_CSR[cs], csr);
}
    
/**------------------------------------------------------------------------------
* Starts a SPI master transfer. This is a non blocking function. It will
* return as soon as the transfer is started.
* Returns 0 if the transfer has been started successfully; otherwise returns
* SPID_ERROR_LOCK is the driver is in use, or SPID_ERROR if the command is not
* valid.
* @param pSpid  Pointer to a Spid instance.
* @param pCommand Pointer to the SPI command to execute.
*/
unsigned char SPID_SendCommand(Spid *pSpid, SpidCmd *pCommand)
{
    AT91S_SPI *pSpiHw = pSpid->pSpiHw;
    #if SPID_PCS_MODE == SPID_PCS_VAR
        uint16_t n;
    #else
        unsigned int spiMr;
    #endif
         
    #ifdef __RTX
    os_mut_wait(pSpid->SpidMutex,0xffff); //позже можно добавить поле timeout в структуру SpidCmd. Пока с бесконечным ожиданием.
    #else
    // Try to get the dataflash semaphore
     if (pSpid->semaphore == 0) {
    
         return SPID_ERROR_LOCK;
    }
     pSpid->semaphore--;
    #endif

    // Enable the SPI clock
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCER, (1 << pSpid->spiId));
    
    // Disable transmitter and receiver
    WRITE_SPI(pSpiHw, SPI_PTCR, AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);

    #if SPID_PCS_MODE == SPID_PCS_VAR
        // для режима переменного PCS нужно вставить значения во все слова команды и данных.
        for (n=0;n<pCommand->cmdSize;n++) {
            pCommand->pCmd[n] = (pCommand->pCmd[n] & 0xffff) | (pCommand->spiCs<<16);
        }
        if (pCommand->dataSize) {
            for (n=0;n<pCommand->dataSize;n++) {
                pCommand->pData[n] = (pCommand->pData[n] & 0xffff) | (pCommand->spiCs<<16);
            }
        }
    #else
        // Write to the MR register
        spiMr = READ_SPI(pSpiHw, SPI_MR);
        spiMr |= AT91C_SPI_PCS;
        spiMr &= ~((1 << pCommand->spiCs) << 16);
        WRITE_SPI(pSpiHw, SPI_MR, spiMr);
    #endif    
    // Initialize the two SPI PDC buffer
    WRITE_SPI(pSpiHw, SPI_RPR, (int) pCommand->pCmd);
    WRITE_SPI(pSpiHw, SPI_RCR, pCommand->cmdSize);
    WRITE_SPI(pSpiHw, SPI_TPR, (int) pCommand->pCmd);
    WRITE_SPI(pSpiHw, SPI_TCR, pCommand->cmdSize);
    
    WRITE_SPI(pSpiHw, SPI_RNPR, (int) pCommand->pData);
    WRITE_SPI(pSpiHw, SPI_RNCR, pCommand->dataSize);
    WRITE_SPI(pSpiHw, SPI_TNPR, (int) pCommand->pData);
    WRITE_SPI(pSpiHw, SPI_TNCR, pCommand->dataSize);

    // Initialize the callback
    pSpid->pCurrentCommand = pCommand;
    
    // Enable transmitter and receiver
    WRITE_SPI(pSpiHw, SPI_PTCR, AT91C_PDC_RXTEN | AT91C_PDC_TXTEN);

    // Enable buffer complete interrupt
    WRITE_SPI(pSpiHw, SPI_IER, AT91C_SPI_RXBUFF);
    
    return 0;    
}

#if SPID_PCS_MODE == SPID_PCS_VAR
/**
 * Запускает передачу по SPI аналогично SPID_SendCommand() 
 * но не устанавливает биты PCS (NPCS[3:0]) 
 * Это позволяет посылать посылать массив данных с 
 * разными чипселектами. Биты PCS[19:16] в массивах 
 * команды и данных должны быть установлены перед вызовом
 * функции
 * @author Yuri (21.10.2013)
 * 
 * @param pSpid - Указатель на экземпляр Spid
 * @param pCommand - указатель на команду для выполнения
 * 
 * @return unsigned char 0-успешно, SPID_ERROR_LOCK - SPI занят.
 */
unsigned char SPID_SendVarCS(Spid *pSpid, SpidCmd *pCommand)
{
    AT91S_SPI *pSpiHw = pSpid->pSpiHw;

    uint16_t n;
         
    #ifdef __RTX
    os_mut_wait(pSpid->SpidMutex,0xffff); //позже можно добавить поле timeout в структуру SpidCmd. Пока с бесконечным ожиданием.
    #else
    // Try to get the dataflash semaphore
     if (pSpid->semaphore == 0) {
    
         return SPID_ERROR_LOCK;
    }
     pSpid->semaphore--;
    #endif

    // Enable the SPI clock
    WRITE_PMC(AT91C_BASE_PMC, PMC_PCER, (1 << pSpid->spiId));
    
    // Disable transmitter and receiver
    WRITE_SPI(pSpiHw, SPI_PTCR, AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);

    // Initialize the two SPI PDC buffer
    WRITE_SPI(pSpiHw, SPI_RPR, (int) pCommand->pCmd);
    WRITE_SPI(pSpiHw, SPI_RCR, pCommand->cmdSize);
    WRITE_SPI(pSpiHw, SPI_TPR, (int) pCommand->pCmd);
    WRITE_SPI(pSpiHw, SPI_TCR, pCommand->cmdSize);
    
    WRITE_SPI(pSpiHw, SPI_RNPR, (int) pCommand->pData);
    WRITE_SPI(pSpiHw, SPI_RNCR, pCommand->dataSize);
    WRITE_SPI(pSpiHw, SPI_TNPR, (int) pCommand->pData);
    WRITE_SPI(pSpiHw, SPI_TNCR, pCommand->dataSize);

    // Initialize the callback
    pSpid->pCurrentCommand = pCommand;
    
    // Enable transmitter and receiver
    WRITE_SPI(pSpiHw, SPI_PTCR, AT91C_PDC_RXTEN | AT91C_PDC_TXTEN);

    // Enable buffer complete interrupt
    WRITE_SPI(pSpiHw, SPI_IER, AT91C_SPI_RXBUFF);
    
    return 0;    
}
#endif //#if SPID_PCS_MODE == SPID_PCS_VAR

/**------------------------------------------------------------------------------
* The SPI_Handler must be called by the SPI Interrupt Service Routine with the
* corresponding Spi instance.
* The SPI_Handler will unlock the Spi semaphore and invoke the upper application 
* callback.
* @param pSpid  Pointer to a Spid instance.
*/
void SPID_Handler(Spid *pSpid)
{
    SpidCmd *pSpidCmd = pSpid->pCurrentCommand;
    AT91S_SPI *pSpiHw = pSpid->pSpiHw;
    volatile unsigned int spiSr;
    
    // Read the status register
    spiSr = READ_SPI(pSpiHw, SPI_SR);    
    if (spiSr & AT91C_SPI_RXBUFF) {

        // Disable transmitter and receiver
        WRITE_SPI(pSpiHw, SPI_PTCR, AT91C_PDC_RXTDIS | AT91C_PDC_TXTDIS);

        // Disable the SPI clock
        WRITE_PMC(AT91C_BASE_PMC, PMC_PCDR, (1 << pSpid->spiId));

        // Disable buffer complete interrupt
        WRITE_SPI(pSpiHw, SPI_IDR, AT91C_SPI_RXBUFF);

        #ifdef __RTX
        os_mut_release(pSpid->SpidMutex);
        #else
        // Release the dataflash semaphore
        pSpid->semaphore++;
        #endif
            
        // Invoke the callback associated with the current command
        if (pSpidCmd && pSpidCmd->callback) {
        
            pSpidCmd->callback(0, pSpidCmd->pArgument);
        }
            
        // Nothing must be done after. A new DF operation may have been started
        // in the callback function.
    }
}

/**------------------------------------------------------------------------------
* Returns 1 if the SPI driver is currently busy executing a command; otherwise
* returns 0.
* @param pSpid  Pointer to a SPI driver instance.
*/
unsigned char SPID_IsBusy(const Spid *pSpid)
{
    #ifdef __RTX
    OS_RESULT res;
    res = os_mut_wait(pSpid->SpidMutex,0);
    if (res==OS_R_TMO) {
        return 1;
    }
    else {
        os_mut_release(pSpid->SpidMutex);
        return 0;
    }
    #else
    if (pSpid->semaphore == 0) {

        return 1;
    }
    else {

        return 0;
    }
    #endif
}

