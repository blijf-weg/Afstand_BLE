/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *   KU Leuven - Technology Campus Gent,
 *   Gebroeders De Smetstraat 1,
 *   B-9000 Gent, Belgium
 *
 *         File: circ_buffer.h
 *      Created: 2018-10-22
 *       Author: Geoffrey Ottoy
 *
 *  Description: Header file for circ_buffer.cpp circular buffer class.
 *
 * MIT License
 * 
 * Copyright (c) 2018 DRAMCO - KU Leuven
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __CIRC_BUFFER_H__
#define	__CIRC_BUFFER_H__

#include <Arduino.h>

#define MAX_BUF_SIZE 128

typedef enum circBufferStatuses{
	CB_SUCCESS,
	CB_EMPTY,
	CB_NOT_INITIALIZED,
	CB_NOT_NULL,
	CB_ARG_NULL,
	CB_ERROR
} CircBufferStatus_t;

class CircBuffer{
	public:
		CircBuffer(void);
		CircBufferStatus_t init(uint8_t length);
		void reset(void);
		bool isEmpty(void);
		uint8_t getFill();
		CircBufferStatus_t put(float data);
		CircBufferStatus_t get(float * data);
		CircBufferStatus_t getAverage(float * avg);
		float getAverage();
	
	private:
		float * buffer;
		uint8_t head;
		uint8_t tail;
		uint8_t length;
		uint8_t fill;
		float sum;
};

#endif	/* __CIRC_BUFFER_H__ */
