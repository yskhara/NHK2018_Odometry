/*
 * FifoCircularBuffer.h
 *
 *  Created on: Mar 23, 2017
 *      Author: yusaku
 */

#pragma once

#include <stdint.h>


class FifoCircularBuffer
{
private:
	uint16_t		length;
	uint16_t		read;
	uint16_t		write;
	uint16_t		mask;

	static inline uint16_t pow2_(uint16_t n)
	{
		--n;
		uint64_t p = 0;
		for (; n != 0; n = (uint64_t)(n >> 1)) p = (uint64_t)((p << 1) + 1);
		return (uint16_t)(p + 1);
	}

	uint8_t *			buf;

public:
	inline FifoCircularBuffer(uint16_t capacity)
	{
		this->length = pow2_(capacity);
		this->buf = new uint8_t[this->length];
		this->read = 0;
		this->write = 0;
		this->mask = (uint16_t)(this->length - 1);
	}

	inline void Enqueue(uint8_t item)
	{
		this->buf[this->write] = item;
		this->write = (uint16_t)((this->write + 1) & this->mask);
	}


	/* returns -1 for errors */
	inline int Dequeue(void)
	{
		if(this->read == this->write)
		{
			return -1;
		}

		int c = this->buf[this->read];
		this->read = (uint16_t)((this->read + 1) & this->mask);
		return c;
	}

	inline void Clear(void)
	{
		this->read = 0;
		this->write = 0;
	}

	inline uint16_t GetCount(void)
	{
	    return (this->length + this->write - this->read) & this->mask;
	}
};

