#pragma once

class USART_wrapper;

template<int USART, int TxBufferSize, int RxBufferSize>
class UartWithBuffer;

//#include "cmsis_device.h"
#include "stm32f10x_conf.h"
#include "FIFORingBuffer.h"
#include "diag/Trace.h"

template<int USART, int TxBufferSize, int RxBufferSize>
class UartWithBuffer
{
public:
	UartWithBuffer(int baudRate = 57600)//Uart::BaudRate)
	{
		this->_usart = reinterpret_cast<USART_TypeDef *>(USART);

		if(this->_usart == nullptr)
		{
			// error!
			this->_error = true;
#ifdef DEBUG
			trace_puts("USART_ERROR (Reason: invalid argument nullptr)");
			//exit(1);
			while(1) ;
#endif
		}

		this->_txBuf = new FIFORingBuffer<uint8_t>(TxBufferSize);
		this->_rxBuf = new FIFORingBuffer<uint8_t>(RxBufferSize);
	}

	inline void Tx_Enqueue(uint8_t c)
	{
		if(!(_txBuf->Enqueue(c)))
		{
			trace_puts("USART_TxOVF");
			_txBuf->Clear();

			this->_error = true;
		}

		_usart->CR1 |= USART_CR1_TXEIE;
	}

	inline bool Tx_Dequeue(uint8_t * c)
	{
		if(c == nullptr)
		{
			return false;
		}

		if(_txBuf->GetItemCount() == 0)
		{
			_usart->CR1 &= ~USART_CR1_TXEIE;
			return false;
		}

		*c = _txBuf->Dequeue();

		/*
		if(_txBuf->GetItemCount() == 0)
		{
			_usart->CR1 &= ~USART_CR1_TXEIE;
		}
		*/
		return true;
	}

	inline int Tx_Count()
	{
		return _txBuf->GetItemCount();
	}

	inline bool Tx_IsFull()
	{
		return _txBuf->IsFull();
	}

	inline void Rx_Enqueue(uint8_t c)
	{
		if(!(_rxBuf->Enqueue(c)))
		{
			// ovf
			trace_puts("USART_RxOVF");
			_rxBuf->Clear();

			this->_error = true;
		}
	}

	inline bool Rx_Dequeue(uint8_t * c)
	{
		if(_rxBuf->GetItemCount() == 0 || c == nullptr)
		{
			return false;
		}

		*c = _rxBuf->Dequeue();

		return true;
	}

	inline int Rx_Count()
	{
		return _rxBuf->GetItemCount();
	}

	inline void OnInterrupt()
	{
		if(this->_usart == nullptr) return;

		if(USART_GetITStatus(this->_usart, USART_IT_TXE))
		{
			unsigned char c;
			if(this->Tx_Dequeue(&c))
			{
				this->_usart->DR = c;
			}
			//USART_ClearITPendingBit(this->_usart, USART_IT_TXE);
		}

		if(USART_GetITStatus(this->_usart, USART_IT_RXNE))
		{
			this->Rx_Enqueue(this->_usart->DR);
			USART_ClearITPendingBit(this->_usart, USART_IT_RXNE);
		}
	}

private:
	USART_TypeDef * _usart = nullptr;
	FIFORingBuffer<uint8_t> * _txBuf = nullptr;
	FIFORingBuffer<uint8_t> * _rxBuf = nullptr;
	bool _error = false;
};

class USART
{
public:

	static void Init(uint8_t capacity);

	static inline void InsertLast(uint8_t c)
	{
		// bloody inatomic operations
		TxBuf->Enqueue(c);
		USART1->CR1 |= USART_CR1_TXEIE;
	}

	/*
	 * Returns false if the buffer was empty.
	 *
	 */
	static inline bool EraseFirst(uint8_t * out)
	{
		if(TxBuf->GetItemCount() == 0 || out == nullptr)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;
			return false;
		}

		*out = TxBuf->Dequeue();

		if(TxBuf->GetItemCount() == 0)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}

		return true;
	}

	static inline uint16_t GetTxBufItemCount(void)
	{
		return TxBuf->GetItemCount();
	}

	static void SendChar(char c);
	static void SendStr(const char *s);
	static void SendCRLF(void);

	static void SendInt32(int32_t i);
	static void SendInt16(int16_t i);

private:
	static FIFORingBuffer<uint8_t> * TxBuf;
};

