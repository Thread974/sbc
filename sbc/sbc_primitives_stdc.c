/*
 *
 *  Bluetooth low-complexity, subband codec (SBC) library
 *
 *  Copyright (C) 2008-2010  Nokia Corporation
 *  Copyright (C) 2004-2010  Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2004-2005  Henryk Ploetz <henryk@ploetzli.ch>
 *  Copyright (C) 2005-2006  Brad Midgley <bmidgley@xmission.com>
 *  Copyright (C) 2012 Intel Corporation
 *
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <stdint.h>
#include <limits.h>
#include <string.h>
#include "sbc.h"
#include "sbc_math.h"
#include "sbc_tables.h"

#include "sbc_primitives.h"
#include "sbc_primitives_stdc.h"

/*
 * stdc optimizations
 */

#ifdef SBC_BUILD_WITH_STDC_SUPPORT

/*
 * A standard C code of analysis filter.
 */
static inline void sbc_analyze_four_stdc(const int16_t *in, int32_t *out)
{
	FIXED_A t1[4];
	FIXED_T t2[4];
	int i = 0, hop = 0;

	/* rounding coefficient */
	t1[0] = t1[1] = t1[2] = t1[3] =
		(FIXED_A) 1 << (SBC_PROTO_FIXED4_SCALE - 1);

	/* low pass polyphase filter */
	for (hop = 0; hop < 40; hop += 8) {
		t1[0] += (FIXED_A) in[hop] * _sbc_proto_fixed4[hop];
		t1[1] += (FIXED_A) in[hop + 1] * _sbc_proto_fixed4[hop + 1];
		t1[2] += (FIXED_A) in[hop + 2] * _sbc_proto_fixed4[hop + 2];
		t1[1] += (FIXED_A) in[hop + 3] * _sbc_proto_fixed4[hop + 3];
		t1[0] += (FIXED_A) in[hop + 4] * _sbc_proto_fixed4[hop + 4];
		t1[3] += (FIXED_A) in[hop + 5] * _sbc_proto_fixed4[hop + 5];
		t1[3] += (FIXED_A) in[hop + 7] * _sbc_proto_fixed4[hop + 7];
	}

	/* scaling */
	t2[0] = t1[0] >> SBC_PROTO_FIXED4_SCALE;
	t2[1] = t1[1] >> SBC_PROTO_FIXED4_SCALE;
	t2[2] = t1[2] >> SBC_PROTO_FIXED4_SCALE;
	t2[3] = t1[3] >> SBC_PROTO_FIXED4_SCALE;

	/* do the cos transform */
	for (i = 0, hop = 0; i < 4; hop += 8, i++) {
		out[i] = ((FIXED_A) t2[0] * cos_table_fixed_4[0 + hop] +
			  (FIXED_A) t2[1] * cos_table_fixed_4[1 + hop] +
			  (FIXED_A) t2[2] * cos_table_fixed_4[2 + hop] +
			  (FIXED_A) t2[3] * cos_table_fixed_4[5 + hop]) >>
			(SBC_COS_TABLE_FIXED4_SCALE - SCALE_OUT_BITS);
	}
}

static inline void sbc_analyze_eight_stdc(const int16_t *in, int32_t *out)
{
	FIXED_A t1[8];
	FIXED_T t2[8];
	int i, hop;

	/* rounding coefficient */
	t1[0] = t1[1] = t1[2] = t1[3] = t1[4] = t1[5] = t1[6] = t1[7] =
		(FIXED_A) 1 << (SBC_PROTO_FIXED8_SCALE-1);

	/* low pass polyphase filter */
	for (hop = 0; hop < 80; hop += 16) {
		t1[0] += (FIXED_A) in[hop] * _sbc_proto_fixed8[hop];
		t1[1] += (FIXED_A) in[hop + 1] * _sbc_proto_fixed8[hop + 1];
		t1[2] += (FIXED_A) in[hop + 2] * _sbc_proto_fixed8[hop + 2];
		t1[3] += (FIXED_A) in[hop + 3] * _sbc_proto_fixed8[hop + 3];
		t1[4] += (FIXED_A) in[hop + 4] * _sbc_proto_fixed8[hop + 4];
		t1[3] += (FIXED_A) in[hop + 5] * _sbc_proto_fixed8[hop + 5];
		t1[2] += (FIXED_A) in[hop + 6] * _sbc_proto_fixed8[hop + 6];
		t1[1] += (FIXED_A) in[hop + 7] * _sbc_proto_fixed8[hop + 7];
		t1[0] += (FIXED_A) in[hop + 8] * _sbc_proto_fixed8[hop + 8];
		t1[5] += (FIXED_A) in[hop + 9] * _sbc_proto_fixed8[hop + 9];
		t1[6] += (FIXED_A) in[hop + 10] * _sbc_proto_fixed8[hop + 10];
		t1[7] += (FIXED_A) in[hop + 11] * _sbc_proto_fixed8[hop + 11];
		t1[7] += (FIXED_A) in[hop + 13] * _sbc_proto_fixed8[hop + 13];
		t1[6] += (FIXED_A) in[hop + 14] * _sbc_proto_fixed8[hop + 14];
		t1[5] += (FIXED_A) in[hop + 15] * _sbc_proto_fixed8[hop + 15];
	}

	/* scaling */
	t2[0] = t1[0] >> SBC_PROTO_FIXED8_SCALE;
	t2[1] = t1[1] >> SBC_PROTO_FIXED8_SCALE;
	t2[2] = t1[2] >> SBC_PROTO_FIXED8_SCALE;
	t2[3] = t1[3] >> SBC_PROTO_FIXED8_SCALE;
	t2[4] = t1[4] >> SBC_PROTO_FIXED8_SCALE;
	t2[5] = t1[5] >> SBC_PROTO_FIXED8_SCALE;
	t2[6] = t1[6] >> SBC_PROTO_FIXED8_SCALE;
	t2[7] = t1[7] >> SBC_PROTO_FIXED8_SCALE;

	/* do the cos transform */
	for (i = 0, hop = 0; i < 8; hop += 16, i++) {
		out[i] = ((FIXED_A) t2[0] * cos_table_fixed_8[0 + hop] +
			  (FIXED_A) t2[1] * cos_table_fixed_8[1 + hop] +
			  (FIXED_A) t2[2] * cos_table_fixed_8[2 + hop] +
			  (FIXED_A) t2[3] * cos_table_fixed_8[3 + hop] +
			  (FIXED_A) t2[4] * cos_table_fixed_8[4 + hop] +
			  (FIXED_A) t2[5] * cos_table_fixed_8[9 + hop] +
			  (FIXED_A) t2[6] * cos_table_fixed_8[10 + hop] +
			  (FIXED_A) t2[7] * cos_table_fixed_8[11 + hop]) >>
			(SBC_COS_TABLE_FIXED8_SCALE - SCALE_OUT_BITS);
	}
}

static inline void sbc_analyze_4b_4s_stdc(int16_t *x, int32_t *out,
						int out_stride)
{
	/* Analyze blocks */
	sbc_analyze_four_stdc(x + 0, out);
}

static inline void sbc_analyze_4b_8s_stdc(int16_t *x, int32_t *out,
						int out_stride)
{
	/* Analyze blocks */
	sbc_analyze_eight_stdc(x + 0, out);
}

static inline int16_t unaligned16_be(const uint8_t *ptr)
{
	return (int16_t) ((ptr[0] << 8) | ptr[1]);
}

static inline int16_t unaligned16_le(const uint8_t *ptr)
{
	return (int16_t) (ptr[0] | (ptr[1] << 8));
}

/*
 * Internal helper functions for input data processing. In order to get
 * optimal performance, it is important to have "nsamples", "nchannels"
 * and "big_endian" arguments used with this inline function as compile
 * time constants.
 */

static SBC_ALWAYS_INLINE int sbc_encoder_process_input_s4_stdc(
	int position,
	const uint8_t *pcm, int16_t X[2][SBC_X_BUFFER_SIZE],
	int nsamples, int nchannels, int big_endian)
{
	/* handle X buffer wraparound */
	if (position < nsamples) {
		if (nchannels > 0)
			memcpy(&X[0][SBC_X_BUFFER_SIZE - 40], &X[0][position],
							36 * sizeof(int16_t));
		if (nchannels > 1)
			memcpy(&X[1][SBC_X_BUFFER_SIZE - 40], &X[1][position],
							36 * sizeof(int16_t));
		position = SBC_X_BUFFER_SIZE - 40;
	}

	#define PCM(i) (big_endian ? \
		unaligned16_be(pcm + (i) * 2) : unaligned16_le(pcm + (i) * 2))

	/* copy audio samples */
	while ((nsamples -= 1) >= 0) {
		position -= 1;
		if (nchannels > 0) {
			int16_t *x = &X[0][position];
			x[0]  = PCM(0 + 0 * nchannels);
		}
		if (nchannels > 1) {
			int16_t *x = &X[1][position];
			x[1]  = PCM(1 + 0 * nchannels);
		}
		pcm += 2 * nchannels;
	}


	#undef PCM

	return position;
}


static SBC_ALWAYS_INLINE int sbc_encoder_process_input_s8_stdc(
	int position,
	const uint8_t *pcm, int16_t X[2][SBC_X_BUFFER_SIZE],
	int nsamples, int nchannels, int big_endian)
{
	/* handle X buffer wraparound */
	if (position < nsamples) {
		if (nchannels > 0)
			memcpy(&X[0][SBC_X_BUFFER_SIZE - 72], &X[0][position],
							72 * sizeof(int16_t));
		if (nchannels > 1)
			memcpy(&X[1][SBC_X_BUFFER_SIZE - 72], &X[1][position],
							72 * sizeof(int16_t));
		position = SBC_X_BUFFER_SIZE - 72;
	}

	#define PCM(i) (big_endian ? \
		unaligned16_be(pcm + (i) * 2) : unaligned16_le(pcm + (i) * 2))

	/* copy audio samples */
	while ((nsamples -= 1) >= 0) {
		position -= 1;
		if (nchannels > 0) {
			int16_t *x = &X[0][position];
			x[0]  = PCM(0 + 0 * nchannels);
		}
		if (nchannels > 1) {
			int16_t *x = &X[1][position];
			x[1]  = PCM(1 + 0 * nchannels);
		}
		pcm += 2 * nchannels;
	}

	#undef PCM
	return position;
}

/*
 * Input data processing functions. The data is endian converted if needed,
 * channels are deintrleaved and audio samples are reordered for use in
 * SIMD-friendly analysis filter function. The results are put into "X"
 * array, getting appended to the previous data (or it is better to say
 * prepended, as the buffer is filled from top to bottom). Old data is
 * discarded when neededed, but availability of (10 * nrof_subbands)
 * contiguous samples is always guaranteed for the input to the analysis
 * filter. This is achieved by copying a sufficient part of old data
 * to the top of the buffer on buffer wraparound.
 */

static int sbc_enc_process_input_4s_le_stdc(int position,
		const uint8_t *pcm, int16_t X[2][SBC_X_BUFFER_SIZE],
		int nsamples, int nchannels)
{
	if (nchannels > 1)
		return sbc_encoder_process_input_s4_stdc(
			position, pcm, X, nsamples, 2, 0);
	else
		return sbc_encoder_process_input_s4_stdc(
			position, pcm, X, nsamples, 1, 0);
}

static int sbc_enc_process_input_4s_be_stdc(int position,
		const uint8_t *pcm, int16_t X[2][SBC_X_BUFFER_SIZE],
		int nsamples, int nchannels)
{
	if (nchannels > 1)
		return sbc_encoder_process_input_s4_stdc(
			position, pcm, X, nsamples, 2, 1);
	else
		return sbc_encoder_process_input_s4_stdc(
			position, pcm, X, nsamples, 1, 1);
}

static int sbc_enc_process_input_8s_le_stdc(int position,
		const uint8_t *pcm, int16_t X[2][SBC_X_BUFFER_SIZE],
		int nsamples, int nchannels)
{
	if (nchannels > 1)
		return sbc_encoder_process_input_s8_stdc(
			position, pcm, X, nsamples, 2, 0);
	else
		return sbc_encoder_process_input_s8_stdc(
			position, pcm, X, nsamples, 1, 0);
}

static int sbc_enc_process_input_8s_be_stdc(int position,
		const uint8_t *pcm, int16_t X[2][SBC_X_BUFFER_SIZE],
		int nsamples, int nchannels)
{
	if (nchannels > 1)
		return sbc_encoder_process_input_s8_stdc(
			position, pcm, X, nsamples, 2, 1);
	else
		return sbc_encoder_process_input_s8_stdc(
			position, pcm, X, nsamples, 1, 1);
}


void sbc_init_primitives_stdc(struct sbc_encoder_state *state)
{
	state->inc = 1;
	state->sbc_analyze_4b_4s = sbc_analyze_4b_4s_stdc;
	state->sbc_analyze_4b_8s = sbc_analyze_4b_8s_stdc;

	/* Default implementation for input reordering / deinterleaving */
	state->sbc_enc_process_input_4s_le = sbc_enc_process_input_4s_le_stdc;
	state->sbc_enc_process_input_4s_be = sbc_enc_process_input_4s_be_stdc;
	state->sbc_enc_process_input_8s_le = sbc_enc_process_input_8s_le_stdc;
	state->sbc_enc_process_input_8s_be = sbc_enc_process_input_8s_be_stdc;

	state->implementation_info = "stdc";
}

#endif
