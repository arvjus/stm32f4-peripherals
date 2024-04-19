
/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <sys/time.h>
#include <sys/times.h>

#include "../drivers/stm32f407xx_gpio.h"
#include "../drivers/stm32f407xx_usart.h"


/* Variables */
//#undef errno
extern int errno;
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

register char * stack_ptr asm("sp");

char *__env[1] = { 0 };
char **environ = __env;

USART_Handle_t usartHandle;

/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
	return 1;
}

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit (int status)
{
	_kill(status, -1);
	while (1) {}		/* Make sure we hang here */
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		*ptr++ = __io_getchar();
	}

return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    USART_SendData(&usartHandle, (uint8_t *)ptr, len);
	return len;
}

int _close(int file)
{
	return -1;
}


int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _open(char *path, int flags, ...)
{
	/* Pretend like we always fail */
	return -1;
}

int _wait(int *status)
{
	errno = ECHILD;
	return -1;
}

int _unlink(char *name)
{
	errno = ENOENT;
	return -1;
}

int _times(struct tms *buf)
{
	return -1;
}

int _stat(char *file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _link(char *old, char *new)
{
	errno = EMLINK;
	return -1;
}

int _fork(void)
{
	errno = EAGAIN;
	return -1;
}

int _execve(char *name, char **argv, char **env)
{
	errno = ENOMEM;
	return -1;
}

/**
 _sbrk
 Increase program data space. Malloc and related functions depend on this
**/
caddr_t _sbrk(int incr)
{
	extern char end asm("end");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > stack_ptr)
	{
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

/*
 * Call this functioni before calling any printf();
 * ex.
 * connect USB-UART white to PA2, green to PA3
 * stdio_usart_init(GPIOA, 2, 3, USART2);
 */
void stdio_usart_init(GPIO_RegDef_t *pGPIOx, uint8_t pinTx, uint8_t pinRx, USART_RegDef_t *pUSARTx)
{
    // Init GPIO pins for I2C
    GPIO_Handle_t usart2Pins;
    memset(&usart2Pins, 0, sizeof(usart2Pins));
    usart2Pins.pGPIOx = pGPIOx;
    usart2Pins.pinConfig.mode = GPIO_PIN_MODE_ALTFN;
    usart2Pins.pinConfig.altFunMode = 7;
    usart2Pins.pinConfig.opType = GPIO_PIN_OPTYPE_PP;
    usart2Pins.pinConfig.puPdControl = GPIO_PIN_PU;
    usart2Pins.pinConfig.speed = GPIO_PIN_SPEED_MEDIUM;
    // TX
    usart2Pins.pinConfig.number = pinTx;
    GPIO_Init(&usart2Pins);
    // RX
    usart2Pins.pinConfig.number = pinRx;
    GPIO_Init(&usart2Pins);

    // Init USART
    memset(&usartHandle, 0, sizeof(usartHandle));
    usartHandle.pUSARTx = pUSARTx;
    usartHandle.config.mode = USART_MODE_ONLY_TX;
    usartHandle.config.baudRate = USART_STD_BAUD_9600;
    usartHandle.config.stopBits = USART_PARITY_DI;
    usartHandle.config.wordLength = USART_WORDLEN_8BITS;
    usartHandle.config.stopBits = USART_STOPBITS_1;
    usartHandle.config.hwFlowCtrl = USART_HW_FLOW_CTRL_NONE;
    USART_Init(&usartHandle);
    USART_PeriphCtrl(pUSARTx, ENABLE);
}
