#include "cmt2300a.h"

static u8 g_nInterrutFlags = 0;

//spi 100k
void cmt_spi3_delay(void)
{
    udelay(5);
}

void cmt_spi3_init(void)
{
    cmt_spi3_csb_1();
    cmt_spi3_sclk_0();
    cmt_spi3_sdio_1();
    cmt_spi3_fcsb_1();
    cmt_spi3_delay();
}

void cmt_spi3_send(u8 data8)
{
    u8 i;

    for(i=0; i<8; i++)
    {
        cmt_spi3_sclk_0();

        /* Send byte on the rising edge of SCLK */
        if(data8 & 0x80)
            cmt_spi3_sdio_1();
        else
            cmt_spi3_sdio_0();

        cmt_spi3_delay();

        data8 <<= 1;
        cmt_spi3_sclk_1();
        cmt_spi3_delay();
    }
}

u8 cmt_spi3_recv(void)
{
    u8 i;
    u8 data8 = 0xFF;

    for(i=0; i<8; i++)
    {
        cmt_spi3_sclk_0();
        cmt_spi3_delay();
        data8 <<= 1;

        cmt_spi3_sclk_1();

        /* Read byte on the rising edge of SCLK */
        if(cmt_spi3_sdio_read())
            data8 |= 0x01;
        else
            data8 &= ~0x01;

        cmt_spi3_delay();
    }

    return data8;
}

void cmt_spi3_write(u8 addr, u8 dat)
{
    cmt_spi3_sdio_1();
    cmt_spi3_sclk_0();
    cmt_spi3_fcsb_1();
    cmt_spi3_csb_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    /* r/w = 0 */
    cmt_spi3_send(addr&0x7F);

    cmt_spi3_send(dat);

    cmt_spi3_sclk_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    cmt_spi3_csb_1();

    cmt_spi3_sdio_1();
    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

void cmt_spi3_read(u8 addr, u8* p_dat)
{
    cmt_spi3_sdio_1();
    cmt_spi3_sclk_0();
    cmt_spi3_fcsb_1();
    cmt_spi3_csb_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    /* r/w = 1 */
    cmt_spi3_send(addr|0x80);

    /* Must set SDIO to input before the falling edge of SCLK */
    cmt_spi3_sdio_in();

    *p_dat = cmt_spi3_recv();

    cmt_spi3_sclk_0();

    /* > 0.5 SCLK cycle */
    cmt_spi3_delay();
    cmt_spi3_delay();

    cmt_spi3_csb_1();

    cmt_spi3_sdio_1();
    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

void cmt_spi3_write_fifo(const u8* p_buf, u16 len)
{
    u16 i;

    cmt_spi3_fcsb_1();
    cmt_spi3_csb_1();
    cmt_spi3_sclk_0();
    cmt_spi3_sdio_1();

    for(i=0; i<len; i++)
    {
        cmt_spi3_fcsb_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay();
        cmt_spi3_delay();

        cmt_spi3_send(p_buf[i]);

        cmt_spi3_sclk_0();

        /* > 2 us */
        mdelay(4);

        cmt_spi3_fcsb_1();

        /* > 4 us */
        mdelay(6);
    }

    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

void cmt_spi3_read_fifo(u8* p_buf, u16 len)
{
    u16 i;

    cmt_spi3_fcsb_1();
    cmt_spi3_csb_1();
    cmt_spi3_sclk_0();
    cmt_spi3_sdio_in();

    for(i=0; i<len; i++)
    {
        cmt_spi3_fcsb_0();

        /* > 1 SCLK cycle */
        cmt_spi3_delay();
        cmt_spi3_delay();

        p_buf[i] = cmt_spi3_recv();

        cmt_spi3_sclk_0();

        /* > 2 us */
        mdelay(4);

        cmt_spi3_fcsb_1();

        /* > 4 us */
        mdelay(6);
    }

    cmt_spi3_sdio_in();

    cmt_spi3_fcsb_1();
}

/*! ********************************************************
* @name    CMT2300A_InitGpio
* @desc    Initializes the CMT2300A interface GPIOs.
* *********************************************************/
void CMT2300A_InitGpio(void)
{
    CMT2300A_SetGpio1In();
    CMT2300A_SetGpio2In();
    CMT2300A_SetGpio3In();

    cmt_spi3_init();
}

/*! ********************************************************
* @name    CMT2300A_ReadReg
* @desc    Read the CMT2300A register at the specified address.
* @param   addr: register address
* @return  Register value
* *********************************************************/
u8 CMT2300A_ReadReg(u8 addr)
{
    u8 dat = 0xFF;
    cmt_spi3_read(addr, &dat);

    return dat;
}

/*! ********************************************************
* @name    CMT2300A_WriteReg
* @desc    Write the CMT2300A register at the specified address.
* @param   addr: register address
*          dat: register value
* *********************************************************/
void CMT2300A_WriteReg(u8 addr, u8 dat)
{
    cmt_spi3_write(addr, dat);
}

/*! ********************************************************
* @name    CMT2300A_ReadFifo
* @desc    Reads the contents of the CMT2300A FIFO.
* @param   buf: buffer where to copy the FIFO read data
*          len: number of bytes to be read from the FIFO
* *********************************************************/
void CMT2300A_ReadFifo(u8 buf[], u16 len)
{
    cmt_spi3_read_fifo(buf, len);
}

/*! ********************************************************
* @name    CMT2300A_WriteFifo
* @desc    Writes the buffer contents to the CMT2300A FIFO.
* @param   buf: buffer containing data to be put on the FIFO
*          len: number of bytes to be written to the FIFO
* *********************************************************/
void CMT2300A_WriteFifo(const u8 buf[], u16 len)
{
    cmt_spi3_write_fifo(buf, len);
}

/*! ********************************************************
* @name    CMT2300A_SoftReset
* @desc    Soft reset.
* *********************************************************/
void CMT2300A_SoftReset(void)
{
    CMT2300A_WriteReg(0x7F, 0xFF);
}

/*! ********************************************************
* @name    CMT2300A_GetChipStatus
* @desc    Get the chip status.
* @return
*          CMT2300A_STA_PUP
*          CMT2300A_STA_SLEEP
*          CMT2300A_STA_STBY
*          CMT2300A_STA_RFS
*          CMT2300A_STA_TFS
*          CMT2300A_STA_RX
*          CMT2300A_STA_TX
*          CMT2300A_STA_EEPROM
*          CMT2300A_STA_ERROR
*          CMT2300A_STA_CAL
* *********************************************************/
u8 CMT2300A_GetChipStatus(void)
{
    return  CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
}

/*! ********************************************************
* @name    CMT2300A_AutoSwitchStatus
* @desc    Auto switch the chip status, and 10 ms as timeout.
* @param   nGoCmd: the chip next status
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_AutoSwitchStatus(u8 nGoCmd)
{
#ifdef ENABLE_AUTO_SWITCH_CHIP_STATUS
    u32 nBegTick = CMT2300A_GetTickCount();
    u8  nWaitStatus;

    switch(nGoCmd)
    {
    case CMT2300A_GO_SLEEP:
        nWaitStatus = CMT2300A_STA_SLEEP;
        break;
    case CMT2300A_GO_STBY :
        nWaitStatus = CMT2300A_STA_STBY ;
        break;
    case CMT2300A_GO_TFS  :
        nWaitStatus = CMT2300A_STA_TFS  ;
        break;
    case CMT2300A_GO_TX   :
        nWaitStatus = CMT2300A_STA_TX   ;
        break;
    case CMT2300A_GO_RFS  :
        nWaitStatus = CMT2300A_STA_RFS  ;
        break;
    case CMT2300A_GO_RX   :
        nWaitStatus = CMT2300A_STA_RX   ;
        break;
    }

    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, nGoCmd);

    while(CMT2300A_GetTickCount()-nBegTick < 10)
    {
        CMT2300A_DelayUs(100);

        if(nWaitStatus==CMT2300A_GetChipStatus())
            return TRUE;

        if(CMT2300A_GO_TX==nGoCmd) {
            CMT2300A_DelayUs(100);

            if(CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_CLR1))
                return TRUE;
        }

        if(CMT2300A_GO_RX==nGoCmd) {
            CMT2300A_DelayUs(100);

            if(CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG))
                return TRUE;
        }
    }

    return FALSE;

#else
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_CTL, nGoCmd);
    return TRUE;
#endif
}

/*! ********************************************************
* @name    CMT2300A_GoSleep
* @desc    Entry SLEEP mode.
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_GoSleep(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_SLEEP);
}

/*! ********************************************************
* @name    CMT2300A_GoStby
* @desc    Entry Sleep mode.
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_GoStby(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_STBY);
}

/*! ********************************************************
* @name    CMT2300A_GoTFS
* @desc    Entry TFS mode.
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_GoTFS(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_TFS);
}

/*! ********************************************************
* @name    CMT2300A_GoRFS
* @desc    Entry RFS mode.
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_GoRFS(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_RFS);
}

/*! ********************************************************
* @name    CMT2300A_GoTx
* @desc    Entry Tx mode.
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_GoTx(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_TX);
}

/*! ********************************************************
* @name    CMT2300A_GoRx
* @desc    Entry Rx mode.
* @return  TRUE or FALSE
* *********************************************************/
BOOL CMT2300A_GoRx(void)
{
    return CMT2300A_AutoSwitchStatus(CMT2300A_GO_RX);
}

/*! ********************************************************
* @name    CMT2300A_ConfigGpio
* @desc    Config GPIO pins mode.
* @param   nGpioSel: GPIO1_SEL | GPIO2_SEL | GPIO3_SEL | GPIO4_SEL
*          GPIO1_SEL:
*            CMT2300A_GPIO1_SEL_DOUT/DIN
*            CMT2300A_GPIO1_SEL_INT1
*            CMT2300A_GPIO1_SEL_INT2
*            CMT2300A_GPIO1_SEL_DCLK
*
*          GPIO2_SEL:
*            CMT2300A_GPIO2_SEL_INT1
*            CMT2300A_GPIO2_SEL_INT2
*            CMT2300A_GPIO2_SEL_DOUT/DIN
*            CMT2300A_GPIO2_SEL_DCLK
*
*          GPIO3_SEL:
*            CMT2300A_GPIO3_SEL_CLKO
*            CMT2300A_GPIO3_SEL_DOUT/DIN
*            CMT2300A_GPIO3_SEL_INT2
*            CMT2300A_GPIO3_SEL_DCLK
*
*          GPIO4_SEL:
*            CMT2300A_GPIO4_SEL_RSTIN
*            CMT2300A_GPIO4_SEL_INT1
*            CMT2300A_GPIO4_SEL_DOUT
*            CMT2300A_GPIO4_SEL_DCLK
* *********************************************************/
void CMT2300A_ConfigGpio(u8 nGpioSel)
{
    CMT2300A_WriteReg(CMT2300A_CUS_IO_SEL, nGpioSel);
}

/*! ********************************************************
* @name    CMT2300A_ConfigInterrupt
* @desc    Config interrupt on INT1 and INT2.
* @param   nInt1Sel, nInt2Sel
*            CMT2300A_INT_SEL_RX_ACTIVE
*            CMT2300A_INT_SEL_TX_ACTIVE
*            CMT2300A_INT_SEL_RSSI_VLD
*            CMT2300A_INT_SEL_PREAM_OK
*            CMT2300A_INT_SEL_SYNC_OK
*            CMT2300A_INT_SEL_NODE_OK
*            CMT2300A_INT_SEL_CRC_OK
*            CMT2300A_INT_SEL_PKT_OK
*            CMT2300A_INT_SEL_SL_TMO
*            CMT2300A_INT_SEL_RX_TMO
*            CMT2300A_INT_SEL_TX_DONE
*            CMT2300A_INT_SEL_RX_FIFO_NMTY
*            CMT2300A_INT_SEL_RX_FIFO_TH
*            CMT2300A_INT_SEL_RX_FIFO_FULL
*            CMT2300A_INT_SEL_RX_FIFO_WBYTE
*            CMT2300A_INT_SEL_RX_FIFO_OVF
*            CMT2300A_INT_SEL_TX_FIFO_NMTY
*            CMT2300A_INT_SEL_TX_FIFO_TH
*            CMT2300A_INT_SEL_TX_FIFO_FULL
*            CMT2300A_INT_SEL_STATE_IS_STBY
*            CMT2300A_INT_SEL_STATE_IS_FS
*            CMT2300A_INT_SEL_STATE_IS_RX
*            CMT2300A_INT_SEL_STATE_IS_TX
*            CMT2300A_INT_SEL_LED
*            CMT2300A_INT_SEL_TRX_ACTIVE
*            CMT2300A_INT_SEL_PKT_DONE
* *********************************************************/
void CMT2300A_ConfigInterrupt(u8 nInt1Sel, u8 nInt2Sel)
{
    nInt1Sel &= CMT2300A_MASK_INT1_SEL;
    nInt1Sel |= (~CMT2300A_MASK_INT1_SEL) & CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);
    CMT2300A_WriteReg(CMT2300A_CUS_INT1_CTL, nInt1Sel);

    nInt2Sel &= CMT2300A_MASK_INT2_SEL;
    nInt2Sel |= (~CMT2300A_MASK_INT2_SEL) & CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);
    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, nInt2Sel);
}

/*! ********************************************************
* @name    CMT2300A_SetInterruptPolar
* @desc    Set the polarity of the interrupt.
* @param   bEnable(TRUE): active-high (default)
*          bEnable(FALSE): active-low
* *********************************************************/
void CMT2300A_SetInterruptPolar(BOOL bActiveHigh)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);

    if(bActiveHigh)
        tmp &= ~CMT2300A_MASK_INT_POLAR;
    else
        tmp |= CMT2300A_MASK_INT_POLAR;

    CMT2300A_WriteReg(CMT2300A_CUS_INT1_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_SetFifoThreshold
* @desc    Set FIFO threshold.
* @param   nFifoThreshold
* *********************************************************/
void CMT2300A_SetFifoThreshold(u8 nFifoThreshold)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_PKT29);

    tmp &= ~CMT2300A_MASK_FIFO_TH;
    tmp |= nFifoThreshold & CMT2300A_MASK_FIFO_TH;

    CMT2300A_WriteReg(CMT2300A_CUS_PKT29, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableAntennaSwitch
* @desc    Enable antenna switch, output TX_ACTIVE/RX_ACTIVE
*          via GPIO1/GPIO2.
* @param   nMode
*            0: RF_SWT1_EN=1, RF_SWT2_EN=0
*               GPIO1: RX_ACTIVE, GPIO2: TX_ACTIVE
*            1: RF_SWT1_EN=0, RF_SWT2_EN=1
*               GPIO1: RX_ACTIVE, GPIO2: ~RX_ACTIVE
* *********************************************************/
void CMT2300A_EnableAntennaSwitch(u8 nMode)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);

    if(0 == nMode) {
        tmp |= CMT2300A_MASK_RF_SWT1_EN;
        tmp &= ~CMT2300A_MASK_RF_SWT2_EN;
    }
    else if(1 == nMode) {
        tmp &= ~CMT2300A_MASK_RF_SWT1_EN;
        tmp |= CMT2300A_MASK_RF_SWT2_EN;
    }

    CMT2300A_WriteReg(CMT2300A_CUS_INT1_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableInterrupt
* @desc    Enable interrupt.
* @param   nEnable
*            CMT2300A_MASK_SL_TMO_EN   |
*            CMT2300A_MASK_RX_TMO_EN   |
*            CMT2300A_MASK_TX_DONE_EN  |
*            CMT2300A_MASK_PREAM_OK_EN |
*            CMT2300A_MASK_SYNC_OK_EN  |
*            CMT2300A_MASK_NODE_OK_EN  |
*            CMT2300A_MASK_CRC_OK_EN   |
*            CMT2300A_MASK_PKT_DONE_EN
* *********************************************************/
void CMT2300A_EnableInterrupt(u8 nEnable)
{
    CMT2300A_WriteReg(CMT2300A_CUS_INT_EN, nEnable);
}

/*! ********************************************************
* @name    CMT2300A_EnableRxFifoAutoClear
* @desc    Auto clear Rx FIFO before entry Rx mode.
* @param   bEnable(TRUE): Enable it(default)
*          bEnable(FALSE): Disable it
* *********************************************************/
void CMT2300A_EnableRxFifoAutoClear(BOOL bEnable)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);

    if(bEnable)
        tmp &= ~CMT2300A_MASK_FIFO_AUTO_CLR_DIS;
    else
        tmp |= CMT2300A_MASK_FIFO_AUTO_CLR_DIS;

    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableFifoMerge
* @desc    Enable FIFO merge.
* @param   bEnable(TRUE): use a single 64-byte FIFO for either Tx or Rx
*          bEnable(FALSE): use a 32-byte FIFO for Tx and another 32-byte FIFO for Rx(default)
* *********************************************************/
void CMT2300A_EnableFifoMerge(BOOL bEnable)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);

    if(bEnable)
        tmp |= CMT2300A_MASK_FIFO_MERGE_EN;
    else
        tmp &= ~CMT2300A_MASK_FIFO_MERGE_EN;

    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableReadFifo
* @desc    Enable SPI to read the FIFO.
* *********************************************************/
void CMT2300A_EnableReadFifo(void)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    tmp &= ~CMT2300A_MASK_SPI_FIFO_RD_WR_SEL;
    tmp &= ~CMT2300A_MASK_FIFO_RX_TX_SEL;
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableWriteFifo
* @desc    Enable SPI to write the FIFO.
* *********************************************************/
void CMT2300A_EnableWriteFifo(void)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    tmp |= CMT2300A_MASK_SPI_FIFO_RD_WR_SEL;
    tmp |= CMT2300A_MASK_FIFO_RX_TX_SEL;
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_RestoreFifo
* @desc    Restore the FIFO.
* *********************************************************/
void CMT2300A_RestoreFifo(void)
{
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_RESTORE);
}

/*! ********************************************************
* @name    CMT2300A_ClearFifo
* @desc    Clear the Tx FIFO.
* @return  FIFO flags
*            CMT2300A_MASK_RX_FIFO_FULL_FLG |
*            CMT2300A_MASK_RX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_RX_FIFO_TH_FLG   |
*            CMT2300A_MASK_RX_FIFO_OVF_FLG  |
*            CMT2300A_MASK_TX_FIFO_FULL_FLG |
*            CMT2300A_MASK_TX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_TX_FIFO_TH_FLG
* *********************************************************/
u8 CMT2300A_ClearTxFifo(void)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_FLAG);
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_TX);
    return tmp;
}

/*! ********************************************************
* @name    CMT2300A_ClearFifo
* @desc    Clear the Rx FIFO.
* @return  FIFO flags
*            CMT2300A_MASK_RX_FIFO_FULL_FLG |
*            CMT2300A_MASK_RX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_RX_FIFO_TH_FLG   |
*            CMT2300A_MASK_RX_FIFO_OVF_FLG  |
*            CMT2300A_MASK_TX_FIFO_FULL_FLG |
*            CMT2300A_MASK_TX_FIFO_NMTY_FLG |
*            CMT2300A_MASK_TX_FIFO_TH_FLG
* *********************************************************/
u8 CMT2300A_ClearRxFifo(void)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_FLAG);
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_RX);
    return tmp;
}

/*! ********************************************************
* @name    CMT2300A_ClearInterruptFlags
* @desc    Clear all interrupt flags.
* @return  Some interrupt flags
*            CMT2300A_MASK_SL_TMO_EN    |
*            CMT2300A_MASK_RX_TMO_EN    |
*            CMT2300A_MASK_TX_DONE_EN   |
*            CMT2300A_MASK_PREAM_OK_FLG |
*            CMT2300A_MASK_SYNC_OK_FLG  |
*            CMT2300A_MASK_NODE_OK_FLG  |
*            CMT2300A_MASK_CRC_OK_FLG   |
*            CMT2300A_MASK_PKT_OK_FLG
* *********************************************************/
u8 CMT2300A_ClearInterruptFlags(void)
{
    u8 nFlag1, nFlag2;
    u8 nClr1 = 0;
    u8 nClr2 = 0;
    u8 nRet  = 0;
    u8 nIntPolar;

    nIntPolar = CMT2300A_ReadReg(CMT2300A_CUS_INT1_CTL);
    nIntPolar = (nIntPolar & CMT2300A_MASK_INT_POLAR) ?1 :0;

    nFlag1 = CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG);
    nFlag2 = CMT2300A_ReadReg(CMT2300A_CUS_INT_CLR1);

    if(nIntPolar) {
        /* Interrupt flag active-low */
        nFlag1 = ~nFlag1;
        nFlag2 = ~nFlag2;
    }

    if(CMT2300A_MASK_LBD_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_LBD_CLR;         /* Clear LBD_FLG */
    }

    if(CMT2300A_MASK_COL_ERR_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR;    /* Clear COL_ERR_FLG by PKT_DONE_CLR */
    }

    if(CMT2300A_MASK_PKT_ERR_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR;    /* Clear PKT_ERR_FLG by PKT_DONE_CLR */
    }

    if(CMT2300A_MASK_PREAM_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PREAM_OK_CLR;    /* Clear PREAM_OK_FLG */
        nRet  |= CMT2300A_MASK_PREAM_OK_FLG;    /* Return PREAM_OK_FLG */
    }

    if(CMT2300A_MASK_SYNC_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_SYNC_OK_CLR;    /* Clear SYNC_OK_FLG */
        nRet  |= CMT2300A_MASK_SYNC_OK_FLG;    /* Return SYNC_OK_FLG */
    }

    if(CMT2300A_MASK_NODE_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_NODE_OK_CLR;    /* Clear NODE_OK_FLG */
        nRet  |= CMT2300A_MASK_NODE_OK_FLG;    /* Return NODE_OK_FLG */
    }

    if(CMT2300A_MASK_CRC_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_CRC_OK_CLR;    /* Clear CRC_OK_FLG */
        nRet  |= CMT2300A_MASK_CRC_OK_FLG;    /* Return CRC_OK_FLG */
    }

    if(CMT2300A_MASK_PKT_OK_FLG & nFlag1) {
        nClr2 |= CMT2300A_MASK_PKT_DONE_CLR;  /* Clear PKT_OK_FLG */
        nRet  |= CMT2300A_MASK_PKT_OK_FLG;    /* Return PKT_OK_FLG */
    }

    if(CMT2300A_MASK_SL_TMO_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_SL_TMO_CLR;    /* Clear SL_TMO_FLG */
        nRet  |= CMT2300A_MASK_SL_TMO_EN;     /* Return SL_TMO_FLG by SL_TMO_EN */
    }

    if(CMT2300A_MASK_RX_TMO_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_RX_TMO_CLR;    /* Clear RX_TMO_FLG */
        nRet  |= CMT2300A_MASK_RX_TMO_EN;     /* Return RX_TMO_FLG by RX_TMO_EN */
    }

    if(CMT2300A_MASK_TX_DONE_FLG & nFlag2) {
        nClr1 |= CMT2300A_MASK_TX_DONE_CLR;   /* Clear TX_DONE_FLG */
        nRet  |= CMT2300A_MASK_TX_DONE_EN;    /* Return TX_DONE_FLG by TX_DONE_EN */
    }

    CMT2300A_WriteReg(CMT2300A_CUS_INT_CLR1, nClr1);
    CMT2300A_WriteReg(CMT2300A_CUS_INT_CLR2, nClr2);

    if(nIntPolar) {
        /* Interrupt flag active-low */
        nRet = ~nRet;
    }

    return nRet;
}

/*! ********************************************************
* @name    CMT2300A_ConfigTxDin
* @desc    Used to select whether to use GPIO1 or GPIO2 or GPIO3
*          as DIN in the direct mode. It only takes effect when
*          call CMT2300A_EnableTxDin(TRUE) in the direct mode.
* @param   nDinSel
*            CMT2300A_TX_DIN_SEL_GPIO1
*            CMT2300A_TX_DIN_SEL_GPIO2
*            CMT2300A_TX_DIN_SEL_GPIO3
* *********************************************************/
void CMT2300A_ConfigTxDin(u8 nDinSel)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);
    tmp &= ~CMT2300A_MASK_TX_DIN_SEL;
    tmp |= nDinSel;
    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableTxDin
* @desc    Used to change GPIO1/GPIO2/GPIO3 between DOUT and DIN.
* @param   bEnable(TRUE): used as DIN
*          bEnable(FALSE): used as DOUT(default)
* *********************************************************/
void CMT2300A_EnableTxDin(BOOL bEnable)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FIFO_CTL);

    if(bEnable)
        tmp |= CMT2300A_MASK_TX_DIN_EN;
    else
        tmp &= ~CMT2300A_MASK_TX_DIN_EN;

    CMT2300A_WriteReg(CMT2300A_CUS_FIFO_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableTxDinInvert
* @desc    Used to invert DIN data in direct mode.
* @param   bEnable(TRUE): invert DIN
*          bEnable(FALSE): not invert DIN(default)
* *********************************************************/
void CMT2300A_EnableTxDinInvert(BOOL bEnable)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);

    if(bEnable)
        tmp |= CMT2300A_MASK_TX_DIN_INV;
    else
        tmp &= ~CMT2300A_MASK_TX_DIN_INV;

    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_IsExist
* @desc    Chip indentify.
* @return  TRUE: chip is exist, FALSE: chip not found
* *********************************************************/
BOOL CMT2300A_IsExist(void)
{
    u8 back, dat;

    back = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, 0xAA);

    dat = CMT2300A_ReadReg(CMT2300A_CUS_PKT17);
    CMT2300A_WriteReg(CMT2300A_CUS_PKT17, back);
    pr_info("CMT2300A_IsExist=%x\n",dat);
    if(0xAA==dat)
        return TRUE;
    else
        return FALSE;
}

/*! ********************************************************
* @name    CMT2300A_GetRssiCode
* @desc    Get RSSI code.
* @return  RSSI code
* *********************************************************/
u8 CMT2300A_GetRssiCode(void)
{
    return CMT2300A_ReadReg(CMT2300A_CUS_RSSI_CODE);
}

/*! ********************************************************
* @name    CMT2300A_GetRssiDBm
* @desc    Get RSSI dBm.
* @return  dBm
* *********************************************************/
int CMT2300A_GetRssiDBm(void)
{
    return (int)CMT2300A_ReadReg(CMT2300A_CUS_RSSI_DBM) - 128;
}

/*! ********************************************************
* @name    CMT2300A_SetFrequencyChannel
* @desc    This defines up to 255 frequency channel
*          for fast frequency hopping operation.
* @param   nChann: the frequency channel
* *********************************************************/
void CMT2300A_SetFrequencyChannel(u8 nChann)
{
    CMT2300A_WriteReg(CMT2300A_CUS_FREQ_CHNL, nChann);
}

/*! ********************************************************
* @name    CMT2300A_SetFrequencyStep
* @desc    This defines the frequency channel step size
*          for fast frequency hopping operation.
*          One step size is 2.5 kHz.
* @param   nOffset: the frequency step
* *********************************************************/
void CMT2300A_SetFrequencyStep(u8 nOffset)
{
    CMT2300A_WriteReg(CMT2300A_CUS_FREQ_OFS, nOffset);
}

/*! ********************************************************
* @name    CMT2300A_SetPayloadLength
* @desc    Set payload length.
* @param   nLength
* *********************************************************/
void CMT2300A_SetPayloadLength(u16 nLength)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_PKT14);

    tmp &= ~CMT2300A_MASK_PAYLOAD_LENG_10_8;
    tmp |= (nLength >> 4) & CMT2300A_MASK_PAYLOAD_LENG_10_8;
    CMT2300A_WriteReg(CMT2300A_CUS_PKT14, tmp);

    tmp = nLength & CMT2300A_MASK_PAYLOAD_LENG_7_0;
    CMT2300A_WriteReg(CMT2300A_CUS_PKT15, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableLfosc
* @desc    If you need use sleep timer, you should enable LFOSC.
* @param   bEnable(TRUE): Enable it(default)
*          bEnable(FALSE): Disable it
* *********************************************************/
void CMT2300A_EnableLfosc(BOOL bEnable)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_SYS2);

    if(bEnable) {
        tmp |= CMT2300A_MASK_LFOSC_RECAL_EN;
        tmp |= CMT2300A_MASK_LFOSC_CAL1_EN;
        tmp |= CMT2300A_MASK_LFOSC_CAL2_EN;
    }
    else {
        tmp &= ~CMT2300A_MASK_LFOSC_RECAL_EN;
        tmp &= ~CMT2300A_MASK_LFOSC_CAL1_EN;
        tmp &= ~CMT2300A_MASK_LFOSC_CAL2_EN;
    }

    CMT2300A_WriteReg(CMT2300A_CUS_SYS2, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableLfoscOutput
* @desc    LFOSC clock is output via GPIO3.
* @param   bEnable(TRUE): Enable it
*          bEnable(FALSE): Disable it(default)
* *********************************************************/
void CMT2300A_EnableLfoscOutput(BOOL bEnable)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_INT2_CTL);

    if(bEnable)
        tmp |= CMT2300A_MASK_LFOSC_OUT_EN;
    else
        tmp &= ~CMT2300A_MASK_LFOSC_OUT_EN;

    CMT2300A_WriteReg(CMT2300A_CUS_INT2_CTL, tmp);
}

/*! ********************************************************
* @name    CMT2300A_EnableAfc
* @desc    AFC enable or disanble.
* @param   bEnable(TRUE): Enable it
*          bEnable(FALSE): Disable it(default)
* *********************************************************/
void CMT2300A_EnableAfc(BOOL bEnable)
{
    u8 tmp = CMT2300A_ReadReg(CMT2300A_CUS_FSK5);

    if(bEnable)
        tmp |= 0x10;
    else
        tmp &= ~0x10;

    CMT2300A_WriteReg(CMT2300A_CUS_FSK5, tmp);
}

/*! ********************************************************
* @name    CMT2300A_SetAfcOvfTh
* @desc    This is optional, only needed when using Rx fast frequency hopping.
* @param   afcOvfTh: AFC_OVF_TH see AN142 and AN197 for details.
* *********************************************************/
void CMT2300A_SetAfcOvfTh(u8 afcOvfTh)
{
    CMT2300A_WriteReg(CMT2300A_CUS_FSK4, afcOvfTh);
}

/*! ********************************************************
* @name    CMT2300A_Init
* @desc    Initialize chip status.
* *********************************************************/
void CMT2300A_Init(void)
{
    u8 tmp;

    CMT2300A_SoftReset();
    CMT2300A_DelayMs(20);

    CMT2300A_GoStby();

    tmp  = CMT2300A_ReadReg(CMT2300A_CUS_MODE_STA);
    tmp |= CMT2300A_MASK_CFG_RETAIN;         /* Enable CFG_RETAIN */
    tmp &= ~CMT2300A_MASK_RSTN_IN_EN;        /* Disable RSTN_IN */
    CMT2300A_WriteReg(CMT2300A_CUS_MODE_STA, tmp);

    tmp  = CMT2300A_ReadReg(CMT2300A_CUS_EN_CTL);
    tmp |= CMT2300A_MASK_LOCKING_EN;         /* Enable LOCKING_EN */
    CMT2300A_WriteReg(CMT2300A_CUS_EN_CTL, tmp);

    CMT2300A_EnableLfosc(FALSE);             /* Diable LFOSC */

    CMT2300A_ClearInterruptFlags();
}

/*! ********************************************************
* @name    CMT2300A_ConfigRegBank
* @desc    Config one register bank.
* *********************************************************/
BOOL CMT2300A_ConfigRegBank(u8 base_addr, const u8 bank[], u8 len)
{
    u8 i;
    for(i=0; i<len; i++)
        CMT2300A_WriteReg(i+base_addr, bank[i]);

    return TRUE;
}

void RF_Config(void)
{
#ifdef ENABLE_ANTENNA_SWITCH
    /* If you enable antenna switch, GPIO1/GPIO2 will output RX_ACTIVE/TX_ACTIVE,
       and it can't output INT1/INT2 via GPIO1/GPIO2 */
    CMT2300A_EnableAntennaSwitch(0);

#else
    /* Config GPIOs */
    CMT2300A_ConfigGpio(
        CMT2300A_GPIO1_SEL_INT1 | /* INT1 > GPIO1 */
        CMT2300A_GPIO2_SEL_INT2 | /* INT2 > GPIO2 */
        CMT2300A_GPIO3_SEL_DOUT
    );

    /* Config interrupt */
    CMT2300A_ConfigInterrupt(
        CMT2300A_INT_SEL_TX_DONE, /* Config INT1 */
        CMT2300A_INT_SEL_PKT_OK   /* Config INT2 */
    );
#endif

    /* Enable interrupt */
    CMT2300A_EnableInterrupt(
        CMT2300A_MASK_TX_DONE_EN  |
        CMT2300A_MASK_PREAM_OK_EN |
        CMT2300A_MASK_SYNC_OK_EN  |
        CMT2300A_MASK_NODE_OK_EN  |
        CMT2300A_MASK_CRC_OK_EN   |
        CMT2300A_MASK_PKT_DONE_EN
    );

    /* Disable low frequency OSC calibration */
    CMT2300A_EnableLfosc(FALSE);

    /* Use a single 64-byte FIFO for either Tx or Rx */
    //CMT2300A_EnableFifoMerge(TRUE);

    //CMT2300A_SetFifoThreshold(16);

    /* This is optional, only needed when using Rx fast frequency hopping */
    /* See AN142 and AN197 for details */
    //CMT2300A_SetAfcOvfTh(0x27);

    /* Go to sleep for configuration to take effect */
    CMT2300A_GoSleep();
}

void RF_Init(void)
{
    u8 tmp;

    CMT2300A_InitGpio();
    CMT2300A_Init();

    /* Config registers */
    CMT2300A_ConfigRegBank(CMT2300A_CMT_BANK_ADDR       , g_cmt2300aCmtBank       , CMT2300A_CMT_BANK_SIZE       );
    CMT2300A_ConfigRegBank(CMT2300A_SYSTEM_BANK_ADDR    , g_cmt2300aSystemBank    , CMT2300A_SYSTEM_BANK_SIZE    );
    CMT2300A_ConfigRegBank(CMT2300A_FREQUENCY_BANK_ADDR , g_cmt2300aFrequencyBank , CMT2300A_FREQUENCY_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_DATA_RATE_BANK_ADDR , g_cmt2300aDataRateBank  , CMT2300A_DATA_RATE_BANK_SIZE );
    CMT2300A_ConfigRegBank(CMT2300A_BASEBAND_BANK_ADDR  , g_cmt2300aBasebandBank  , CMT2300A_BASEBAND_BANK_SIZE  );
    CMT2300A_ConfigRegBank(CMT2300A_TX_BANK_ADDR        , g_cmt2300aTxBank        , CMT2300A_TX_BANK_SIZE        );

    // xosc_aac_code[2:0] = 2
    tmp = (~0x07) & CMT2300A_ReadReg(CMT2300A_CUS_CMT10);
    CMT2300A_WriteReg(CMT2300A_CUS_CMT10, tmp|0x02);

    RF_Config();
}



void RF_SetStatus(EnumRFStatus nStatus)
{
    g_nNextRFState = nStatus;
}

EnumRFStatus RF_GetStatus(void)
{
    return g_nNextRFState;
}

u8 RF_GetInterruptFlags(void)
{
    return g_nInterrutFlags;
}

void RF_StartRx(u8 buf[], u16 len, u32 timeout)
{
    g_pRxBuffer = buf;
    g_nRxLength = len;
    g_nRxTimeout = timeout;

    memset(g_pRxBuffer, 0, g_nRxLength);
    g_nNextRFState = RF_STATE_RX_START;
}

void RF_StartTx(u8 buf[], u16 len, u32 timeout)
{
    g_pTxBuffer = buf;
    g_nTxLength = len;
    g_nTxTimeout = timeout;

    g_nNextRFState = RF_STATE_TX_START;
}

EnumRFResult RF_Process(void)
{
    EnumRFResult nRes = RF_BUSY;
    int ret;

    switch(g_nNextRFState)
    {
    case RF_STATE_IDLE:
    {
        nRes = RF_IDLE;
        break;
    }

    case RF_STATE_RX_START:
    {
        CMT2300A_GoStby();
        CMT2300A_ClearInterruptFlags();

        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableReadFifo();
        CMT2300A_ClearRxFifo();

        if(FALSE==CMT2300A_GoRx())
            g_nNextRFState = RF_STATE_ERROR;
        else
            g_nNextRFState = RF_STATE_RX_WAIT;

        g_nRxTimeCount = CMT2300A_GetTickCount();

        break;
    }

    case RF_STATE_RX_WAIT:
    {
#ifdef ENABLE_ANTENNA_SWITCH
        if(CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG))  /* Read PKT_OK flag */
#else
        if(CMT2300A_ReadGpio2())  /* Read INT2, PKT_OK */
#endif
        {
            g_nNextRFState = RF_STATE_RX_DONE;
        }
        ret=ktime_to_ms(ktime_sub(g_nSysTickCount,g_nRxTimeCount));
        if(ret>g_nRxTimeout)
            g_nNextRFState = RF_STATE_RX_TIMEOUT;

        break;
    }

    case RF_STATE_RX_DONE:
    {
        CMT2300A_GoStby();

        /* The length need be smaller than 32 */
        CMT2300A_ReadFifo(g_pRxBuffer, g_nRxLength);

        g_nInterrutFlags = CMT2300A_ClearInterruptFlags();

        CMT2300A_GoSleep();

        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_RX_DONE;
        break;
    }

    case RF_STATE_RX_TIMEOUT:
    {
        CMT2300A_GoSleep();

        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_RX_TIMEOUT;
        break;
    }

    case RF_STATE_TX_START:
    {
        CMT2300A_GoStby();
        CMT2300A_ClearInterruptFlags();

        /* Must clear FIFO after enable SPI to read or write the FIFO */
        CMT2300A_EnableWriteFifo();
        CMT2300A_ClearTxFifo();

        /* The length need be smaller than 32 */
        CMT2300A_WriteFifo(g_pTxBuffer, g_nTxLength);

        if( 0==(CMT2300A_MASK_TX_FIFO_NMTY_FLG & CMT2300A_ReadReg(CMT2300A_CUS_FIFO_FLAG)) )
            g_nNextRFState = RF_STATE_ERROR;

        if(FALSE==CMT2300A_GoTx())
            g_nNextRFState = RF_STATE_ERROR;
        else
            g_nNextRFState = RF_STATE_TX_WAIT;

        g_nTxTimeCount = CMT2300A_GetTickCount();

        break;
    }

    case RF_STATE_TX_WAIT:
    {
#ifdef ENABLE_ANTENNA_SWITCH
        if(CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_CLR1))  /* Read TX_DONE flag */
#else
        if(CMT2300A_ReadGpio1())  /* Read INT1, TX_DONE */
#endif
        {
            g_nNextRFState = RF_STATE_TX_DONE;
        }

        ret=ktime_to_ms(ktime_sub(g_nSysTickCount,g_nTxTimeCount));
        if(ret>g_nTxTimeout) {
            g_nNextRFState = RF_STATE_TX_TIMEOUT;
            pr_info("RF_STATE_TX_WAIT g_nNextRFState=%d ret=%d %d\n",g_nNextRFState,ret,g_nTxTimeout);
        }

        break;
    }

    case RF_STATE_TX_DONE:
    {
        CMT2300A_ClearInterruptFlags();
        CMT2300A_GoSleep();

        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_TX_DONE;
        break;
    }

    case RF_STATE_TX_TIMEOUT:
    {
        CMT2300A_GoSleep();

        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_TX_TIMEOUT;
        break;
    }

    case RF_STATE_ERROR:
    {
        CMT2300A_SoftReset();
        CMT2300A_DelayMs(20);

        CMT2300A_GoStby();
        RF_Config();

        g_nNextRFState = RF_STATE_IDLE;
        nRes = RF_ERROR;
        break;
    }

    default:
        break;
    }

    return nRes;
}

void OnMaster(void) {
    char str[32];
    switch(RF_Process()) {
    case RF_IDLE:
        g_nSendCount++;
        g_txBuffer[0] = (u8)g_nSendCount;
        g_txBuffer[1] = g_nSendCount >> 8;
        RF_StartTx(g_txBuffer, RF_PACKET_SIZE, 1000);
        break;

    case RF_TX_DONE:
        sprintf(str, "send: %d", g_nSendCount);
        pr_info("%s\n",str);
        RF_StartRx(g_rxBuffer, RF_PACKET_SIZE, 1000);
        break;

    case RF_RX_DONE:
        g_nRecvCount++;
        sprintf(str, "recv: %d", g_nRecvCount);
        pr_info("%s\n",str);
        break;

    case RF_RX_TIMEOUT:
        sprintf(str, "recv:timeout");
        pr_info("%s\n",str);
        break;

    case RF_ERROR:
        sprintf(str, "error: %d", ++g_nErrCount);
        pr_info("%s\n",str);
        break;

    default:
        break;
    }
}

int cmt2300a_send_signal(void) {
    int ret;
    RF_Init();

    RF_StartTx(g_txBuffer, RF_PACKET_SIZE, 1000);

//RF_STATE_TX_START
    CMT2300A_GoStby();
    CMT2300A_ClearInterruptFlags();

    /* Must clear FIFO after enable SPI to read or write the FIFO */
    CMT2300A_EnableWriteFifo();
    CMT2300A_ClearTxFifo();

    /* The length need be smaller than 32 */
    CMT2300A_WriteFifo(g_pTxBuffer, g_nTxLength);

    if( 0==(CMT2300A_MASK_TX_FIFO_NMTY_FLG & CMT2300A_ReadReg(CMT2300A_CUS_FIFO_FLAG)) ) {
        g_nNextRFState = RF_STATE_ERROR;
        goto err;
    }

    if(FALSE==CMT2300A_GoTx()) {
        g_nNextRFState = RF_STATE_ERROR;
        goto err;
    }
    else {
        g_nNextRFState = RF_STATE_TX_WAIT;
    }

//RF_STATE_TX_WAIT
#ifdef ENABLE_ANTENNA_SWITCH
    ret=CMT2300A_MASK_TX_DONE_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_CLR1);  /* Read TX_DONE flag */
    //need redesign
#else
    ret=wait_for_completion_timeout(&tx_done_complete,msecs_to_jiffies(g_nTxTimeout));
#endif
    if(ret>0) {
        g_nNextRFState = RF_STATE_TX_DONE;
    }
    if(ret==0) {
        g_nNextRFState = RF_STATE_TX_TIMEOUT;
    }
err:
    pr_info("g_nNextRFState=%d\n",g_nNextRFState);
    return g_nNextRFState;
}

int cmt2300a_receive_signal(void) {
    int ret;
    RF_StartRx(g_rxBuffer, RF_PACKET_SIZE, 1000);

    CMT2300A_GoStby();
    CMT2300A_ClearInterruptFlags();

    /* Must clear FIFO after enable SPI to read or write the FIFO */
    CMT2300A_EnableReadFifo();
    CMT2300A_ClearRxFifo();

    if(FALSE==CMT2300A_GoRx()) {
        g_nNextRFState = RF_STATE_ERROR;
        goto err;
    }

#ifdef ENABLE_ANTENNA_SWITCH
    if(CMT2300A_MASK_PKT_OK_FLG & CMT2300A_ReadReg(CMT2300A_CUS_INT_FLAG))  /* Read PKT_OK flag */
        //need redesign
#else
    ret=wait_for_completion_timeout(&rx_done_complete,msecs_to_jiffies(g_nRxTimeout));
#endif
        if(ret==0) {
            g_nNextRFState = RF_STATE_RX_TIMEOUT;
            goto err;
        } else {
            CMT2300A_GoStby();

            /* The length need be smaller than 32 */
            CMT2300A_ReadFifo(g_pRxBuffer, g_nRxLength);

            g_nInterrutFlags = CMT2300A_ClearInterruptFlags();

            CMT2300A_GoSleep();
            g_nNextRFState = RF_STATE_RX_DONE;
        }
err:
    pr_info("g_nNextRFState=%d\n",g_nNextRFState);
    return g_nNextRFState;
}

static ssize_t cmt2300a_send_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count) {
    int ret=0;
    int i;
    ret=simple_strtoul(buf,NULL,10);
    gpio_direction_output(cmt2300a_en,1);
    for(i=0; i<RF_PACKET_SIZE; i++)
        g_txBuffer[i] = i+1;
    ret=cmt2300a_send_signal();
    gpio_direction_output(cmt2300a_en,0);
    return count;
}

static ssize_t cmt2300a_send_show(struct device *dev,struct device_attribute *attr, char *buf) {
    return sprintf(buf,"g_nNextRFState=%d\n",g_nNextRFState);
}

static ssize_t cmt2300a_send_test_show(struct device *dev,struct device_attribute *attr, char *buf) {

    if(cmt2300a_send_thread!=NULL)
        kthread_stop(cmt2300a_send_thread);
    __pm_relax(&tx_wake_lock);
    gpio_direction_output(cmt2300a_en,0);
    return sprintf(buf,"send cmt2300a count %lld\n",cmt2300a_send_count);
}

static int cmt2300a_send(void *data) {
    int i;
    unsigned char msg=0;
    gpio_direction_output(cmt2300a_en,1);
    do {
        for(i=0; i<RF_PACKET_SIZE; i++)
            g_txBuffer[i] = msg;
        msg++;
        printk(KERN_INFO "cmt2300a_send: %lld times\n", cmt2300a_send_count);
        cmt2300a_send_signal();
        cmt2300a_send_count++;
        msleep(100);
    } while(!kthread_should_stop());
    cmt2300a_send_thread=NULL;
    return 0;
}
static ssize_t cmt2300a_send_test_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count) {

    cmt2300a_send_thread = kthread_run(cmt2300a_send, NULL, "cmt2300a_send");
    if (IS_ERR(cmt2300a_send_thread)) {
        printk(KERN_ERR "failed to create thread,err %ld\n",PTR_ERR(cmt2300a_send_thread));
        cmt2300a_send_thread = NULL;
    }
    cmt2300a_send_count=0;

    __pm_stay_awake(&tx_wake_lock);
    return count;
}

static ssize_t cmt2300a_receive_show(struct device *dev,struct device_attribute *attr, char *buf) {
    pr_info("receive data length %d\n",g_nRxLength);
    print_hex_dump(KERN_INFO, "RX:", DUMP_PREFIX_NONE, 16, 1 , g_rxBuffer,g_nRxLength, true);
    return sprintf(buf,"receive data length is %d\n",g_nRxLength);
}

static ssize_t cmt2300a_receive_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count) {
    int ret=0;
    gpio_direction_output(cmt2300a_en,1);
    ret=simple_strtoul(buf,NULL,10);
    RF_Init();
    cmt2300a_receive_signal();
    gpio_direction_output(cmt2300a_en,0);
    return count;
}

static ssize_t cmt2300a_receive_test_show(struct device *dev,struct device_attribute *attr, char *buf) {

    if(cmt2300a_receive_thread!=NULL)
        kthread_stop(cmt2300a_receive_thread);
    __pm_relax(&rx_wake_lock);
    return sprintf(buf,"receive cmt2300a count %lld\n",cmt2300a_receive_count);
}

static int cmt2300a_receive(void *data) {
    int ret;
    gpio_direction_output(cmt2300a_en,1);
    RF_Init();
    do {
        ret=cmt2300a_receive_signal();
        if(ret==RF_STATE_RX_DONE) {
            print_hex_dump(KERN_INFO, "RX:", DUMP_PREFIX_NONE, 16, 1 , g_rxBuffer,g_nRxLength, true);
        }
        printk(KERN_INFO "cmt2300a_receive: %lld times\n", cmt2300a_receive_count);
        cmt2300a_receive_count++;
    } while(!kthread_should_stop());
    cmt2300a_send_thread=NULL;
    return 0;
}
static ssize_t cmt2300a_receive_test_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count) {

    cmt2300a_receive_thread = kthread_run(cmt2300a_receive, NULL, "cmt2300a_receive");
    if (IS_ERR(cmt2300a_receive_thread)) {
        printk(KERN_ERR "failed to create thread,err %ld\n",PTR_ERR(cmt2300a_receive_thread));
        cmt2300a_receive_thread = NULL;
    }
    cmt2300a_receive_count=0;
    __pm_stay_awake(&rx_wake_lock);
    return count;
}

static ssize_t cmt2300a_exist_show(struct device *dev,struct device_attribute *attr, char *buf) {
    BOOL ret;
    gpio_direction_output(cmt2300a_en,1);
    ret=CMT2300A_IsExist();
    gpio_direction_output(cmt2300a_en,0);
    return sprintf(buf,"connect to cmt2300a %d\n",ret);
}

static DEVICE_ATTR(exist,0444,cmt2300a_exist_show,NULL);
static DEVICE_ATTR(send,0600,cmt2300a_send_show,cmt2300a_send_store);
static DEVICE_ATTR(send_test,0600,cmt2300a_send_test_show,cmt2300a_send_test_store);
static DEVICE_ATTR(receive,0600,cmt2300a_receive_show,cmt2300a_receive_store);
static DEVICE_ATTR(receive_test,0600,cmt2300a_receive_test_show,cmt2300a_receive_test_store);

static struct attribute *cmt2300a_attributes[] = {
    &dev_attr_exist.attr,
    &dev_attr_send.attr,
    &dev_attr_send_test.attr,
    &dev_attr_receive.attr,
    &dev_attr_receive_test.attr,
    NULL
};

static const struct attribute_group cmt2300a_attributes_group = {
    .attrs = cmt2300a_attributes,
};

static irqreturn_t send_handle(int irq, void *dev_id)
{
    if(CMT2300A_ReadGpio1()) {
        complete(&tx_done_complete);
    }
    return IRQ_HANDLED;
}

static irqreturn_t receive_handle(int irq, void *dev_id)
{
    if(CMT2300A_ReadGpio2()) {
        complete(&rx_done_complete);
    }
    return IRQ_HANDLED;
}

void get_dts_info(struct platform_device *pdev)
{
    int ret;
    int virq;
    cmt2300a_en=of_get_named_gpio_flags(pdev->dev.of_node, "en-gpios", 0 , 0);
    if(cmt2300a_en < 0)
        pr_err("Unable to get cmt2300a_en gpio\n");
    else {
        ret=gpio_request(cmt2300a_en, "cmt2300a_en");
        if(ret<0)
            pr_err("request cmt2300a_en err!");
    }

    cmt2300a_clk=of_get_named_gpio_flags(pdev->dev.of_node, "clk-gpios", 0 , 0);
    if(cmt2300a_clk < 0)
        pr_err("Unable to get cmt2300a_clk\n");
    else {
        ret=gpio_request(cmt2300a_clk, "cmt2300a_clk");
        if(ret<0)
            pr_err("request cmt2300a_clk err!");
    }

    cmt2300a_sdio=of_get_named_gpio_flags(pdev->dev.of_node, "sdio-gpios", 0 , 0);
    if(cmt2300a_sdio < 0)
        pr_err("Unable to get cmt2300a_sdio\n");
    else {
        ret=gpio_request(cmt2300a_sdio, "cmt2300a-sdio");
        if(ret<0)
            pr_err("request cmt2300a_sdio err!");
    }

    cmt2300a_fcsb=of_get_named_gpio_flags(pdev->dev.of_node, "fcsb-gpios", 0 , 0);
    if(cmt2300a_fcsb < 0)
        pr_err("Unable to get cmt2300a_fcsb\n");
    else {
        ret=gpio_request(cmt2300a_fcsb, "cmt2300a-fcsb");
        if(ret<0)
            pr_err("request cmt2300a_fcsb err!");
    }

    cmt2300a_csb=of_get_named_gpio_flags(pdev->dev.of_node, "csb-gpios",0 , 0);
    if(cmt2300a_csb < 0)
        pr_err("Unable to get cmt2300a_csb\n");
    else {
        ret=gpio_request(cmt2300a_csb, "cmt2300a-csb");
        if(ret<0)
            pr_err("request cmt2300a_csb err!");
    }

    cmt2300a_gpio1=of_get_named_gpio_flags(pdev->dev.of_node, "gpio1-gpios", 0 , 0);
    if(cmt2300a_gpio1 < 0)
        pr_err("Unable to get cmt2300a_gpio1\n");
    else {
        ret=gpio_request(cmt2300a_gpio1, "cmt2300a-gpio1");
        if(ret<0)
            pr_err("request cmt2300a_gpio1 err!");
        else {
            virq=gpio_to_irq(cmt2300a_gpio1);
            ret = request_irq(virq,send_handle,IRQF_TRIGGER_RISING,"cmt2300a_send_complete", NULL);
            if(ret<0) {
                pr_err("request cmt2300a_gpio1 to irq err!");
            }
        }
    }

    cmt2300a_gpio2=of_get_named_gpio_flags(pdev->dev.of_node, "gpio2-gpios", 0 , 0);
    if(cmt2300a_gpio2 < 0) {
        pr_err("Unable to get cmt2300a_gpio2\n");
    } else {
        ret=gpio_request(cmt2300a_gpio2, "cmt2300a-gpio2");
        if(ret<0)
            pr_err("request cmt2300a_gpio2 err!");
        else {
            virq=gpio_to_irq(cmt2300a_gpio2);
            ret = request_irq(virq,receive_handle,IRQF_TRIGGER_RISING,"cmt2300a_receive_complete", NULL);
            if(ret<0) {
                pr_err("request cmt2300a_gpio2 to irq err!");
            }
        }
    }
}

static int cmt2300a_probe(struct platform_device *pdev)
{
    int ret;
    pr_info("cmt2300a_probe\n");
    get_dts_info(pdev);
    init_completion(&tx_done_complete);
    init_completion(&rx_done_complete);
    gpio_direction_output(cmt2300a_en,0);
    wakeup_source_init(&tx_wake_lock, "tx_wake_lock");
    wakeup_source_init(&rx_wake_lock, "rx_wake_lock");
    ret = sysfs_create_group(&pdev->dev.kobj,&cmt2300a_attributes_group);
    if (ret) {
        pr_err("failed to register cmt2300a sysfs\n");
        return ret;
    }
    return 0;
}

static int cmt2300a_suspend(struct device *dev)
{
    return 0;
}

static int cmt2300a_resume(struct device *dev)
{
    return 0;
}

static const struct dev_pm_ops cmt2300a_ops = {
    .suspend = cmt2300a_suspend,
    .resume  = cmt2300a_resume,
};

static int cmt2300a_remove(struct platform_device *pdev)
{
    return 0;
}

static const struct of_device_id cmt2300a_match[] = {
    { .compatible = "cmostek,cmt2300a", },
    {},
};

static struct platform_driver cmt2300a_module_driver = {
    .probe		= cmt2300a_probe,
    .remove		= cmt2300a_remove,
    .driver		= {
        .name	= "cmt2300a",
        .of_match_table = cmt2300a_match,
        .pm = &cmt2300a_ops,
    },
};

module_platform_driver(cmt2300a_module_driver);
MODULE_LICENSE("GPL");