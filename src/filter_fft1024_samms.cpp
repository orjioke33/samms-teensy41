/* Audio Library for Teensy 3.X
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Arduino.h"
#include "filter_fft1024_samms.h"
#include "sqrt_integer.h"
#include "utility/dspinst.h"

#if defined(__ARM_ARCH_7EM__)
static void copy_to_nlms_buffer(void *destination, const void *source)
{
	const uint16_t *src = (const uint16_t *)source;
	uint32_t *dst = (uint32_t *)destination;

	for (int i=0; i < AUDIO_BLOCK_SAMPLES; i++) {
		dst[i] = src[i];
	}
}
#endif

float AudioFilterSamms::get_avg() {
    float sum = 0;
    for (int i = 0; i < 1024; i++) {
        sum += bufferLeft[i];
    }

    avg = sum / 1024;
    return avg;
}

int AudioFilterSamms::get_state() {
    return state;
}

void AudioFilterSamms::update(void) {
	audio_block_t *blockLeft;

    // grab audio inputs
	blockLeft = receiveReadOnly();

    // if blocks are NULL, quit
	if (!blockLeft) {
        state = -1;
        return;
    }

#if defined(__ARM_ARCH_7EM__)
	switch (state) {
	case 0:
		blocklistLeft[0] = blockLeft;
		state = 1;
		break;
	case 1:
		blocklistLeft[1] = blockLeft;
		state = 2;
		break;
	case 2:
		blocklistLeft[2] = blockLeft;
		state = 3;
		break;
	case 3:
		blocklistLeft[3] = blockLeft;
		state = 4;
		break;
	case 4:
		blocklistLeft[4] = blockLeft;
		state = 5;
		break;
	case 5:
		blocklistLeft[5] = blockLeft;
		state = 6;
		break;
	case 6:
		blocklistLeft[6] = blockLeft;
		state = 7;
		break;
	case 7:
		blocklistLeft[7] = blockLeft;
		// TODO: perhaps distribute the work over multiple update() ??
		//       github pull requsts welcome......

        // Left buffer
		copy_to_nlms_buffer(bufferLeft+0x000, blocklistLeft[0]->data);
		copy_to_nlms_buffer(bufferLeft+0x080, blocklistLeft[1]->data);
		copy_to_nlms_buffer(bufferLeft+0x100, blocklistLeft[2]->data);
		copy_to_nlms_buffer(bufferLeft+0x180, blocklistLeft[3]->data);
		copy_to_nlms_buffer(bufferLeft+0x200, blocklistLeft[4]->data);
		copy_to_nlms_buffer(bufferLeft+0x280, blocklistLeft[5]->data);
		copy_to_nlms_buffer(bufferLeft+0x300, blocklistLeft[6]->data);
		copy_to_nlms_buffer(bufferLeft+0x380, blocklistLeft[7]->data);

        // donlms
        // dofft

		outputflag = true;
		release(blocklistLeft[0]);
		release(blocklistLeft[1]);
		release(blocklistLeft[2]);
		release(blocklistLeft[3]);
		blocklistLeft[0] = blocklistLeft[4];
		blocklistLeft[1] = blocklistLeft[5];
		blocklistLeft[2] = blocklistLeft[6];
		blocklistLeft[3] = blocklistLeft[7];
		state = 4;
		break;
	}
#else
	release(blockLeft);
    state = -4;
#endif
}
