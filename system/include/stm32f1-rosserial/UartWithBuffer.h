/*
 * UartWithBuffer.h
 *
 *  Created on: Nov 27, 2017
 *      Author: yusaku
 */

#pragma once



template<int USART, int TxBufferSize, int RxBufferSize>
class UartWithBuffer;


#include "stm32f10x_conf.h"
#include "FifoCircularBuffer.h"
#include "diag/Trace.h"


class Uart
{
public:
	static constexpr int BaudRate = 1843200; //115200;
	static constexpr int TxBufSize = 2048;
	static constexpr int RxBufSize = 2048;
	static UartWithBuffer<USART1_BASE, TxBufSize, RxBufSize> * const Uart1;
};

using SERIAL_CLASS = UartWithBuffer<USART1_BASE, Uart::TxBufSize, Uart::RxBufSize>;

template<int USART, int TxBufferSize, int RxBufferSize>
class UartWithBuffer
{
private:
	USART_TypeDef * const _usart = (USART_TypeDef *)USART;
	FifoCircularBuffer * const _txBuf = new FifoCircularBuffer(TxBufferSize);
	FifoCircularBuffer * const _rxBuf = new FifoCircularBuffer(RxBufferSize);
	bool _error = false;

public:
	//UartWithBuffer(void)
	//{
	//
	//}

	inline void Tx_Enqueue(uint8_t c)
	{
		_txBuf->Enqueue(c);

		_usart->CR1 |= USART_CR1_TXEIE;
	}

	inline void Tx_Enqueue(uint8_t * const c, uint16_t size)
	{
		for(int i = 0; i < size; i++)
		{
			_txBuf->Enqueue(c[i]);
		}

		_usart->CR1 |= USART_CR1_TXEIE;
	}

	inline int Tx_Dequeue(void)
	{
		if(_txBuf->GetCount() == 0)
		{
			_usart->CR1 &= ~USART_CR1_TXEIE;

			return -1;
		}
		else
		{
			return (int)_txBuf->Dequeue();
		}
	}

	inline int Tx_Count()
	{
		return _txBuf->GetCount();
	}

	inline void Rx_Enqueue(uint8_t c)
	{
		_rxBuf->Enqueue(c);
	}

	inline int Rx_Dequeue(void)
	{
		if(_rxBuf->GetCount() == 0)
		{
			return -1;
		}
		else
		{
			return (int)_rxBuf->Dequeue();
		}
	}

	inline int Rx_Count()
	{
		return _rxBuf->GetCount();
	}

	inline void OnInterrupt()
	{
		//if(this->_usart == nullptr) return;

		if(USART_GetITStatus(this->_usart, USART_IT_TXE))
		{
			int c = this->Tx_Dequeue();
			if(c > -1)
			{
				this->_usart->DR = (uint8_t)c;
			}
			//USART_ClearITPendingBit(this->_usart, USART_IT_TXE);
		}

		if(USART_GetITStatus(this->_usart, USART_IT_RXNE))
		{
			this->Rx_Enqueue(this->_usart->DR);
			USART_ClearITPendingBit(this->_usart, USART_IT_RXNE);
		}
	}

	inline bool Error(void)
	{
		return this->_error;
	}
};





