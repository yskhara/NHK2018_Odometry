/*
 * STM32Hardware.h
 *
 *  Created on: Nov 25, 2017
 *      Author: yusaku
 */

#pragma once


#include "Timer.h"
#include "UartWithBuffer.h"

namespace {
	void initUSART1(void)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_StructInit(&GPIO_InitStruct);
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStruct);

		USART_InitTypeDef USART_InitStruct;
		USART_StructInit(&USART_InitStruct);
		USART_InitStruct.USART_BaudRate = Uart::BaudRate;
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_InitStruct.USART_Parity = USART_Parity_No;
		USART_InitStruct.USART_StopBits = USART_StopBits_1;
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;

		USART_Init(USART1, &USART_InitStruct);
		USART_Cmd(USART1, ENABLE);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		NVIC_EnableIRQ(USART1_IRQn);
	}
}

//using SERIAL_CLASS = UartWithBuffer<USART1_BASE, Uart::TxBufSize, Uart::RxBufSize>;

//static SERIAL_CLASS * USART1_Wrapper;

SERIAL_CLASS * const Uart::Uart1 = new SERIAL_CLASS();

extern "C" void USART1_IRQHandler(void)
{
	/*
	 * linker will take care of this
	if(Uart::Uart1 == nullptr)
	{
		return;
	}
	 */

	Uart::Uart1->OnInterrupt();
}

class STM32Hardware {


  public:
  //STM32Hardware(SERIAL_CLASS* _com , long baud = 57600){
      //com = _com;
      //baud_ = baud;
      //USART1_Wrapper = com;
    //}
    STM32Hardware()
    {
        baud_ = Uart::BaudRate;
        //USART1_Wrapper = new SERIAL_CLASS(baud_);
        //__NOP();
        com = (SERIAL_CLASS * const)Uart::Uart1;

        initUSART1();
    }

    //STM32Hardware(STM32Hardware& h)
    //{
    //  this->baud_ = h.baud_;
    //}

    void setBaud(long baud)
    {
      this->baud_= baud;
    }

    int getBaud()
    {
    	return baud_;
    }

    void init()
    {
      //iostream->begin(baud_);
    }

    int read()
    {
    	return com->Rx_Dequeue();
    }

    void write(uint8_t* data, int length)
    {
    	com->Tx_Enqueue(data, length);
    }

    unsigned long time()
    {
    	return Timer::GetTick();
    }

  protected:
    SERIAL_CLASS* com;
    long baud_;
};
