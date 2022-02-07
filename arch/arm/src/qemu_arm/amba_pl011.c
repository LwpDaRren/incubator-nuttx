/*
 * Copyright (c) 2013-2019 Huawei Technologies Co., Ltd. All rights reserved.
 * Copyright (c) 2020-2021 Huawei Device Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 *    of conditions and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "amba_pl011.h"

#define UART_REG_BASE 	0x09000000

int32_t g_inputIdx = 0;
uint32_t g_uart_fputc_en = 1;

#define REG32(addr) ((volatile uint32_t *)(uintptr_t)(addr))
#define UARTREG(base, reg)  (*REG32((base) + (reg)))
#define UART_FR_TXFF (0x1U << 5)

static void UartPutcReg(uintptr_t base, char c)
{
    /* Spin while fifo is full */
    while (UARTREG(base, UART_FR) & UART_FR_TXFF) {}
    UARTREG(base, UART_DR) = c;
}

static inline uintptr_t uart_to_ptr(uintptr_t n)
{
    (void)n;
    return UART_REG_BASE;
}

int32_t uart_putc(int32_t port, char c)
{
    uintptr_t base = uart_to_ptr((uint32_t)port);
    UartPutcReg(base, c);
    return 1;
}

char uart_fputc(char c, void *f)
{
    (void)f;
    if (g_uart_fputc_en == 1) {
        if (c == '\n') {
            (void)uart_putc(0, '\r');
        }
        return (uart_putc(0, (c)));
    } else {
        return 0;
    }
}

int up_putc(int ch)
{
	uart_fputc((char)ch, NULL);
}

static void UartPutStr(uintptr_t base, const char *s, uint32_t len)
{
    uint32_t i;

    for (i = 0; i < len; i++) {
        if (*(s + i) == '\n') {
            UartPutcReg(base, '\r');
        }
        UartPutcReg(base, *(s + i));
    }
}

uint32_t UartPutsReg(uintptr_t base, const char *s, uint32_t len, int8_t isLock)
{
    uint32_t intSave;

    if (g_uart_fputc_en == 0) {
        return 0;
    }

    if (isLock) {
        //LOS_SpinLockSave(&g_uartOutputSpin, &intSave);
        UartPutStr(base, s, len);
        //LOS_SpinUnlockRestore(&g_uartOutputSpin, intSave);
    } else {
        UartPutStr(base, s, len);
    }

    return len;
}

void UartPuts(const char *s, uint32_t len, int8_t isLock)
{
    uintptr_t base = uart_to_ptr(0);
    (void)UartPutsReg(base, s, len, isLock);
}

int32_t uart_puts(const char *s, uintptr_t len, void *state)
{
    (void)state;
    uintptr_t i;

    for (i = 0; i < len; i++) {
        if (*(s + i) != '\0') {
            if (*(s + i) == '\n') {
                (void)uart_fputc('\r', NULL);
            }

            (void)uart_fputc(*(s + i), NULL);
        }
    }

    return (int32_t)len;
}

#if 0
void uart_handler(void)
{
    char c;
    uintptr_t base = uart_to_ptr(0);

    c = UARTREG(base, UART_DR);

    switch (c) {
        case '\r':
        case '\n':
            if (g_inputIdx < CMD_LENGTH - 1) {
                g_inputCmd[g_inputIdx++] = '\0';
                LOS_EventWrite(&g_stShellEvent, 0x1);
                (void)uart_putc(0, '\r');
                (void)uart_putc(0, '\n');
            }
            break;
        case 0x8:   /* backspace */
        case 0x7f:  /* delete */
            if (g_inputIdx > 0) {
                g_inputIdx--;
                (void)uart_putc(0, '\b');
                (void)uart_putc(0, ' ');
                (void)uart_putc(0, '\b');
            }
            break;
        default:
            if (g_inputIdx < CMD_LENGTH - 1) {
                (void)uart_putc(0, c);
                g_inputCmd[g_inputIdx++] = c;
            }
    }
}
#endif

void uart_early_init(void)
{
    /* enable uart transmit */
    UARTREG(UART_REG_BASE, UART_CR) = (1 << 8) | (1 << 0);
}

void arm_serialinit(void)
{
    uint32_t ret;

    /* uart interrupt priority should be the highest in interrupt preemption mode */
    //ret = LOS_HwiCreate(NUM_HAL_INTERRUPT_UART, 0, 0, (HWI_PROC_FUNC)uart_handler, NULL);
    //if (ret != LOS_OK) {
    //    PRINT_ERR("%s,%d, uart interrupt created error:%x\n", __FUNCTION__, __LINE__, ret);
    //} else {
        /* clear all irqs */
        UARTREG(UART_REG_BASE, UART_ICR) = 0x3ff;

        /* set fifo trigger level */
        UARTREG(UART_REG_BASE, UART_IFLS) = 0;

        /* enable rx interrupt */
        UARTREG(UART_REG_BASE, UART_IMSC) = (1 << 4 | 1 << 6);

        /* enable receive */
        UARTREG(UART_REG_BASE, UART_CR) |= (1 << 9);

		up_enable_irq(32 + 1);
   //}
}
