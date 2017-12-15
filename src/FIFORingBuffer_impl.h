/*
 * FIFORingBuffer.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: yusaku
 */

#include "diag/Trace.h"
#include "FIFORingBuffer.h"

template <typename T>
FIFORingBuffer<T>::FIFORingBuffer(uint16_t capacity)
{
	this->length = pow2_(capacity);
	this->buf = new T[this->length];
	this->head = 0;
	this->tail = 0;
	this->mask = (uint16_t)(this->length - 1);
}

/*
 * Returns false when a overflow is detected
 */
template <typename T>
bool FIFORingBuffer<T>::Enqueue(T item)
{
	if (this->GetItemCount() >= this->length - 1)
	{
		// Buffer Overrun, Eww...
		//this.Extend();

		this->head = 0;
		this->tail = 0;

		return false;
	}

	this->buf[this->tail] = item;
	this->tail = (uint16_t)((this->tail + 1) & this->mask);

	return true;
}


template <typename T>
T FIFORingBuffer<T>::Dequeue(void)
{
	T c = this->buf[this->head];
	this->head = (uint16_t)((this->head+ 1) & this->mask);
	return c;
}

template <typename T>
void FIFORingBuffer<T>::Clear(void)
{
	this->head = 0;
	this->tail = 0;

	/*
	 * �K�v�Ȃ��H���ȁH
	for(uint8_t i = 0; i < this->length; i++)
	{
		this->buf[i] = 0;
	}
	 */
}

template <typename T>
uint16_t FIFORingBuffer<T>::GetItemCount(void)
{
    int count = this->tail - this->head;
    if (count < 0) count += this->length;
    return (uint16_t)count;
}

template <typename T>
uint16_t FIFORingBuffer<T>::pow2_(uint16_t n)
{
	--n;
	uint64_t p = 0;
	for (; n != 0; n = (uint64_t)(n >> 1)) p = (uint64_t)((p << 1) + 1);
	return (uint16_t)(p + 1);
}


