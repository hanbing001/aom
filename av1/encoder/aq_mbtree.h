/*
 * Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent.
 */

#ifndef AV1_ENCODER_AQ_MBTREE_H_
#define AV1_ENCODER_AQ_MBTREE_H_

#include <limits.h>
#include <math.h>

#include "av1/encoder/segmentation.h"

typedef struct MBTreeContext {
  uint8_t *scratch_buf;

  float *prop_cost;
  float *last_intra;
} MBTreeContext;

void av1_mbtree_update(struct AV1_COMP *cpi);

int av1_mbtree_get_mb_delta(struct AV1_COMP *cpi, int mb_row, int mb_col);

void av1_mbtree_init(struct AV1_COMP *cpi);
void av1_mbtree_uninit(struct AV1_COMP *cpi);

#endif  // AV1_ENCODER_AQ_MBTREE_H_
