/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "build_config.h"

#include "common/link_driver.h"
#include "common/utils.h"
#include "gpio.h"
#include "inverter.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

static BOOL actCommSetBaudRate(PifComm* p_comm, uint32_t baudrate)
{
    uartPort_t *uartPort = getUartPort(p_comm->_id);
    if (uartPort) {
        uartSetBaudRate(&uartPort->port, baudrate);
        return TRUE;
    }
    return FALSE;
}

static BOOL actUartStartTransfer(PifComm* p_comm)
{
    uartPort_t *uartPort = getUartPort(p_comm->_id);
    if (uartPort) {
        USART_ITConfig(uartPort->USARTx, USART_IT_TXE, ENABLE);
        return TRUE;
    }
    return FALSE;
}

static void usartConfigurePinInversion(uartPort_t *uartPort) {
#if !defined(INVERTER)
    UNUSED(uartPort);
#else
    bool inverted = uartPort->port.options & SERIAL_INVERTED;

#ifdef INVERTER
    if (inverted && uartPort->USARTx == INVERTER_USART) {
        // Enable hardware inverter if available.
        INVERTER_ON;
    }
#endif
#endif
}

static void uartReconfigure(uartPort_t *uartPort)
{
    USART_InitTypeDef USART_InitStructure;
    USART_Cmd(uartPort->USARTx, DISABLE);

    USART_InitStructure.USART_BaudRate = uartPort->port.baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;

    USART_InitStructure.USART_StopBits = (uartPort->port.options & SERIAL_STOPBITS_2) ? USART_StopBits_2 : USART_StopBits_1;
    USART_InitStructure.USART_Parity   = (uartPort->port.options & SERIAL_PARITY_EVEN) ? USART_Parity_Even : USART_Parity_No;

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (uartPort->port.mode & MODE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (uartPort->port.mode & MODE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;

    USART_Init(uartPort->USARTx, &USART_InitStructure);

    usartConfigurePinInversion(uartPort);

    if(uartPort->port.options & SERIAL_BIDIR)
        USART_HalfDuplexCmd(uartPort->USARTx, ENABLE);
    else
        USART_HalfDuplexCmd(uartPort->USARTx, DISABLE);

    USART_Cmd(uartPort->USARTx, ENABLE);
}

serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s = NULL;
    int id;

    (void)callback;

    if (USARTx == USART1) {
        s = serialUSART1(baudRate, mode, options);
        id = 0;
#ifdef USE_USART2
    } else if (USARTx == USART2) {
        s = serialUSART2(baudRate, mode, options);
        id = 1;
#endif
#ifdef USE_USART3
    } else if (USARTx == USART3) {
        s = serialUSART3(baudRate, mode, options);
        id = 2;
#endif
    } else {
        return (serialPort_t *)s;
    }
    s->txDMAEmpty = true;

    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    uartReconfigure(s);

    if (!pifComm_Init(&s->port.comm, PIF_ID_UART(id))) return NULL;
    s->port.comm.act_set_baudrate = actCommSetBaudRate;

    // Receive DMA or IRQ
    DMA_InitTypeDef DMA_InitStructure;
    if (mode & MODE_RX) {
        if (s->rxDMAChannel) {
            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_PeripheralBaseAddr = s->rxDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

            DMA_InitStructure.DMA_BufferSize = s->port.comm._p_rx_buffer->_size;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
            DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pifRingBuffer_GetTailPointer(s->port.comm._p_rx_buffer, 0);
            DMA_DeInit(s->rxDMAChannel);
            DMA_Init(s->rxDMAChannel, &DMA_InitStructure);
            DMA_Cmd(s->rxDMAChannel, ENABLE);
            USART_DMACmd(s->USARTx, USART_DMAReq_Rx, ENABLE);
            s->rxDMAPos = DMA_GetCurrDataCounter(s->rxDMAChannel);
        } else {
            USART_ClearITPendingBit(s->USARTx, USART_IT_RXNE);
            USART_ITConfig(s->USARTx, USART_IT_RXNE, ENABLE);
        }
    }

    // Transmit DMA or IRQ
    if (mode & MODE_TX) {
        s->port.comm.act_start_transfer = actUartStartTransfer;

        if (s->txDMAChannel) {
            DMA_StructInit(&DMA_InitStructure);
            DMA_InitStructure.DMA_PeripheralBaseAddr = s->txDMAPeripheralBaseAddr;
            DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
            DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

            DMA_InitStructure.DMA_BufferSize = s->port.comm._p_tx_buffer->_size;
            DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
            DMA_DeInit(s->txDMAChannel);
            DMA_Init(s->txDMAChannel, &DMA_InitStructure);
            DMA_ITConfig(s->txDMAChannel, DMA_IT_TC, ENABLE);
            DMA_SetCurrDataCounter(s->txDMAChannel, 0);
            s->txDMAChannel->CNDTR = 0;
            USART_DMACmd(s->USARTx, USART_DMAReq_Tx, ENABLE);
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
        }
    }

    USART_Cmd(s->USARTx, ENABLE);

    return (serialPort_t *)s;
}

void uartSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.baudRate = baudRate;
    uartReconfigure(uartPort);
}

void uartSetMode(serialPort_t *instance, portMode_t mode)
{
    uartPort_t *uartPort = (uartPort_t *)instance;
    uartPort->port.mode = mode;
    uartReconfigure(uartPort);
}

void uartStartTxDMA(uartPort_t *s)
{
    uint16_t size;

    s->txDMAChannel->CMAR = (uint32_t)pifRingBuffer_GetTailPointer(s->port.comm._p_tx_buffer, 0);
    size = pifRingBuffer_GetLinerSize(s->port.comm._p_tx_buffer, 0);
    s->txDMAChannel->CNDTR = size;
    pifRingBuffer_Remove(s->port.comm._p_tx_buffer, size);
    s->txDMAEmpty = false;
    DMA_Cmd(s->txDMAChannel, ENABLE);
}
