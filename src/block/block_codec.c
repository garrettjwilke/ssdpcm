/*
 * ssdpcm: implementation of the SSDPCM audio codec designed by Algorithm.
 * Copyright (C) 2022-2025 Kagamiin~
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "ssdpcm_block_funcs.h"
#include "types.h"

#include <stdio.h>

/* ------------------------------------------------------------------------- */
inline sample_t
calc_sample_ (ssdpcm_block_iterator *dec, codeword_t slope_code)
{
	sample_t slope = dec->block->slopes[slope_code];
	return dec->sample_state + slope;
}

/* ------------------------------------------------------------------------- */
inline sample_t
uncalc_sample_ (ssdpcm_block_iterator *dec, codeword_t slope_code)
{
	sample_t slope = dec->block->slopes[slope_code];
	return dec->sample_state - slope;
}

/* ------------------------------------------------------------------------- */
inline void
decode_one_sample_no_advance_ (ssdpcm_block_iterator *dec)
{
	codeword_t slope_code;
	
	debug_assert(dec->index < dec->block->length);
	slope_code = dec->block->deltas[dec->index];
	dec->sample_state = calc_sample_(dec, slope_code);
	dec->raw[dec->index] = dec->sample_state;
}

/* ------------------------------------------------------------------------- */
inline void
undecode_one_sample_no_backtrack_ (ssdpcm_block_iterator *dec)
{
	codeword_t slope_code;
	
	debug_assert(dec->index < dec->block->length);
	slope_code = dec->block->deltas[dec->index];
	dec->sample_state = uncalc_sample_(dec, slope_code);
	dec->raw[dec->index] = dec->sample_state;
}

/* ------------------------------------------------------------------------- */
inline void
decode_one_sample_ (ssdpcm_block_iterator *dec)
{
	decode_one_sample_no_advance_(dec);
	dec->index++;
}

/* ------------------------------------------------------------------------- */
inline void
enqueue_one_codeword_ (ssdpcm_block_iterator *dest, codeword_t c)
{
	debug_assert(dest->index < dest->block->length);
	dest->block->deltas[dest->index] = c;
	dest->index++;
}

/* ------------------------------------------------------------------------- */
inline void
dequeue_one_codeword_ (ssdpcm_block_iterator *dest)
{
	debug_assert(dest->index > 0);
	dest->index--;
}

/* ------------------------------------------------------------------------- */
inline void
block_iterator_init_ (ssdpcm_block_iterator *iter, ssdpcm_block *block, sample_t *raw)
{
	iter->block = block;
	iter->raw = raw;
	iter->index = 0;
	iter->sample_state = block->initial_sample;
}

/* ------------------------------------------------------------------------- */
// ssdpcm_block_decode: Decodes an SSDPCM block into a sample buffer.
void
ssdpcm_block_decode (sample_t *out, ssdpcm_block *block)
{
	ssdpcm_block_iterator dec;
	block_iterator_init_(&dec, block, out);
	while (dec.index < dec.block->length)
	{
		decode_one_sample_(&dec);
	}
}

/* ------------------------------------------------------------------------- */
inline codeword_t
find_best_delta_ (ssdpcm_encoder *enc)
{
	codeword_t c;
	codeword_t best;
	uint64_t best_error = UINT64_MAX;
	ssdpcm_block *block = enc->iter.block;
	for (c = 0, best = 0; c < block->num_deltas; c++)
	{
		uint64_t error = enc->sigma->methods->calc_error(&enc->sigma->state, c);
		if (error < best_error)
		{
			best_error = error;
			best = c;
		}
	}
	return best;
}

/* ------------------------------------------------------------------------- */
inline codeword_t
find_best_delta_lookahead_ (ssdpcm_encoder *enc, uint8_t lookahead, uint64_t *out_error)
{
	codeword_t c;
	codeword_t best;
	uint64_t best_error = UINT64_MAX;
	ssdpcm_block *block = enc->iter.block;
	for (c = 0, best = 0; c < block->num_deltas; c++)
	{
		uint64_t error = enc->sigma->methods->calc_error(&enc->sigma->state, c);
		uint64_t lookahead_error = 0;
		//uint64_t local_lookahead_error = 0;
		if (lookahead > 0 && (enc->iter.index + 1) < enc->iter.block->length)
		{
			enqueue_one_codeword_(&enc->iter, c);
			enc->sigma->methods->advance(&enc->sigma->state, 0);
			codeword_t unused = find_best_delta_lookahead_(enc, lookahead - 1, &lookahead_error);
			(void)unused;
#if 0			
			if (out_error != NULL && *out_error == 0xDEADBEEFCAFEBABE)
			{
				void **ext_state = &enc->sigma->state;
				sigma_tracker_internal *state = *ext_state;
				ssdpcm_block_iterator *dec = state->sigma_dec;
				fprintf(stderr, "cw: %d, slope:%5ld, localerr:%9ld, totalerr:%9ld\n", c, dec->block->slopes[c], error, error + lookahead_error);
			}
#endif
			dequeue_one_codeword_(&enc->iter);
			enc->sigma->methods->backtrack(&enc->sigma->state, 0);
			error += lookahead_error;
		}
		if (error < best_error)
		{
			best_error = error;
			best = c;
		}
	}
	if (out_error != NULL)
	{
		*out_error = best_error;
	}
	return best;
}

/* ------------------------------------------------------------------------- */
// ssdpcm_block_encode: Encodes an SSDPCM block (given preinitialized fields),
// and given as input:
// - A sample buffer
// - An error tracker (with preallocated state).
// Returns the accumulated error metric.
uint64_t
ssdpcm_block_encode (ssdpcm_block *block, sample_t *in, sigma_tracker *sigma)
{
	uint64_t error_metric;
	ssdpcm_encoder enc;
	block_iterator_init_(&enc.iter, block, in);
	enc.sigma = sigma;
	enc.sigma->methods->init(&enc.sigma->state, &enc.iter);
	while (enc.iter.index < enc.iter.block->length)
	{
		uint64_t error = 0xDEADBEEFCAFEBABE;
		codeword_t best_delta = find_best_delta_lookahead_(&enc, enc.iter.block->lookahead, &error);
		enqueue_one_codeword_(&enc.iter, best_delta);
		enc.sigma->methods->advance(&enc.sigma->state, error);
	}
	error_metric = enc.sigma->methods->get_accumulated_error(&enc.sigma->state);
	return error_metric;
}
