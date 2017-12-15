/*
 * UartWithBuffer.cpp
 *
 *  Created on: Nov 27, 2017
 *      Author: yusaku
 */

#include "UartWithBuffer.h"


UartWithBuffer<USART1_BASE, Uart::TxBufSize, Uart::RxBufSize> *Uart::Uart1 = new UartWithBuffer<USART1_BASE, Uart::TxBufSize, Uart::RxBufSize>(Uart::BaudRate);


