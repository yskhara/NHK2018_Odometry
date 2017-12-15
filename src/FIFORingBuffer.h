/*
 * FIFORingBuffer.h
 *
 *  Created on: Mar 23, 2017
 *      Author: yusaku
 */

#ifndef INCLUDE_FIFORINGBUFFER_H_
#define INCLUDE_FIFORINGBUFFER_H_

#include <stdint.h>


template <typename T>
class FIFORingBuffer
{
public:
	FIFORingBuffer(uint16_t capacity);

	bool Enqueue(T item);
	T Dequeue(void);
	void Clear(void);

	uint16_t GetItemCount(void);
	inline bool IsFull(void)
	{
		return !(length - GetItemCount());
	}

private:
	uint16_t		length;
	uint16_t		head;
	uint16_t		tail;
	uint16_t		mask;

	static uint16_t pow2_(uint16_t n);

	T *			buf;
};


#include "FIFORingBuffer_impl.h"


#endif /* INCLUDE_FIFORINGBUFFER_H_ */
