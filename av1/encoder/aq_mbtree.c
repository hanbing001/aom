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

#include <limits.h>
#include <math.h>
#include <float.h>

#include "./av1_rtcd.h"
#include "./aom_dsp_rtcd.h"

#include "av1/encoder/encoder.h"
#include "aom_dsp/aom_dsp_common.h"
#include "av1/encoder/segmentation.h"
#include "av1/encoder/encodeframe.h"
#include "av1/encoder/mbgraph.h"
#include "av1/common/reconintra.h"
#include "av1/common/reconinter.h"
#include "av1/encoder/aq_mbtree.h"

static int find_best_64x64_intra(AV1_COMP *cpi)
{
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;
  PREDICTION_MODE mode;
  unsigned int best_err = INT_MAX;

  /* Pick the best prediction mode */
  for (mode = DC_PRED; mode <= TM_PRED; mode++) {
    unsigned int err;

    xd->mi[0]->mbmi.mode = mode;
    av1_predict_intra_block(xd, 64, 64, BLOCK_16X16, mode, x->plane[0].src.buf,
                            x->plane[0].src.stride, xd->plane[0].dst.buf,
                            xd->plane[0].dst.stride, 0, 0, 0);
    err = aom_sad64x64(x->plane[0].src.buf, x->plane[0].src.stride,
                       xd->plane[0].dst.buf, xd->plane[0].dst.stride);

    // find best
    if (err < best_err)
      best_err = err;
  }

  return best_err;
}

static int do_64x64_motion_iteration(AV1_COMP *cpi, const MV *ref_mv,
                                     int mb_row, int mb_col) {
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;
  const MV_SPEED_FEATURES *const mv_sf = &cpi->sf.mv;
  const aom_variance_fn_ptr_t v_fn_ptr = cpi->fn_ptr[BLOCK_64X64];

  const int tmp_col_min = x->mv_col_min;
  const int tmp_col_max = x->mv_col_max;
  const int tmp_row_min = x->mv_row_min;
  const int tmp_row_max = x->mv_row_max;
  MV ref_full;
  int cost_list[5];

  // Further step/diamond searches as necessary
  int step_param = mv_sf->reduce_first_step_size;
  step_param = AOMMIN(step_param, MAX_MVSEARCH_STEPS - 2);

  av1_set_mv_search_range(x, ref_mv);

  ref_full.col = ref_mv->col >> 3;
  ref_full.row = ref_mv->row >> 3;

  /*cpi->sf.search_method == HEX*/
  av1_hex_search(x, &ref_full, step_param, x->errorperbit, 0,
                 cond_cost_list(cpi, cost_list), &v_fn_ptr, 0, ref_mv);

  // Try sub-pixel MC
  // if (bestsme > error_thresh && bestsme < INT_MAX)
  {
    int distortion;
    unsigned int sse;
    cpi->find_fractional_mv_step(x, ref_mv, cpi->common.allow_high_precision_mv,
                                 x->errorperbit, &v_fn_ptr, 0,
                                 mv_sf->subpel_iters_per_step,
                                 cond_cost_list(cpi, cost_list), NULL, NULL,
                                 &distortion, &sse, NULL, 0, 0, 0);
  }

#if CONFIG_EXT_INTER
  if (has_second_ref(&xd->mi[0]->mbmi))
    xd->mi[0]->mbmi.mode = NEW_NEWMV;
  else
#endif  // CONFIG_EXT_INTER
    xd->mi[0]->mbmi.mode = NEWMV;

  xd->mi[0]->mbmi.mv[0] = x->best_mv;
#if CONFIG_EXT_INTER
  xd->mi[0]->mbmi.ref_frame[1] = NONE_FRAME;
#endif  // CONFIG_EXT_INTER

  av1_build_inter_predictors_sby(xd, mb_row, mb_col, NULL, BLOCK_64X64);

  /* restore UMV window */
  x->mv_col_min = tmp_col_min;
  x->mv_col_max = tmp_col_max;
  x->mv_row_min = tmp_row_min;
  x->mv_row_max = tmp_row_max;

  return aom_sad64x64(x->plane[0].src.buf, x->plane[0].src.stride,
                      xd->plane[0].dst.buf, xd->plane[0].dst.stride);
}

static int do_64x64_motion_search(AV1_COMP *cpi, const MV *ref_mv, int mb_row,
                                  int mb_col) {
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;
  unsigned int err, tmp_err;
  MV best_mv;

  // Try zero MV first
  // FIXME should really use something like near/nearest MV and/or MV prediction
  err = aom_sad64x64(x->plane[0].src.buf, x->plane[0].src.stride,
                     xd->plane[0].pre[0].buf, xd->plane[0].pre[0].stride);
  best_mv.col = best_mv.row = 0;

  // Test last reference frame using the previous best mv as the
  // starting point (best reference) for the search
  tmp_err = do_64x64_motion_iteration(cpi, ref_mv, mb_row, mb_col);
  if (tmp_err < err) {
    err = tmp_err;
    best_mv = x->best_mv.as_mv;
  }

  // If the current best reference mv is not centered on 0,0 then do a 0,0
  // based search as well.
  if (ref_mv->row != 0 || ref_mv->col != 0) {
    MV zero_ref_mv = { 0, 0 };

    tmp_err = do_64x64_motion_iteration(cpi, &zero_ref_mv, mb_row, mb_col);
    if (tmp_err < err) {
      err = tmp_err;
      best_mv = x->best_mv.as_mv;
    }
  }

  x->best_mv.as_mv = best_mv;
  return err;
}

static void process_frame(AV1_COMP *cpi, MBTreeContext *mbt, int index)
{
  int mb_row, mb_col;
  AV1_COMMON *const cm = &cpi->common;
  struct lookahead_entry *q_0;
  MACROBLOCK *const x = &cpi->td.mb;
  MACROBLOCKD *const xd = &x->e_mbd;

  q_0 = av1_lookahead_peek(cpi->lookahead, index);

  MODE_INFO mi_local;
  av1_zero(mi_local);

  xd->mi[0] = &mi_local;
  mi_local.mbmi.sb_type = BLOCK_64X64;
  mi_local.mbmi.ref_frame[0] = LAST_FRAME;
  mi_local.mbmi.ref_frame[1] = NONE_FRAME;

  YV12_BUFFER_CONFIG *buf0 = &q_0->img;
  YV12_BUFFER_CONFIG tsa = *buf0;
  tsa.y_buffer = mbt->scratch_buf;
  tsa.y_stride = buf0->y_stride;
  tsa.uv_stride = buf0->uv_stride;
  YV12_BUFFER_CONFIG *buf1 = &tsa;

  if (!buf0 || !buf1)
    return;

  struct scale_factors *sf = (struct scale_factors *)&xd->block_refs[1]->sf;

#if CONFIG_AOM_HIGHBITDEPTH
    av1_setup_scale_factors_for_frame(
        sf, buf1->y_crop_width, buf1->y_crop_height,
        buf1->y_crop_width, buf1->y_crop_height,
        cpi->common.use_highbitdepth);
#else
    av1_setup_scale_factors_for_frame(
        sf, buf1->y_crop_width, buf1->y_crop_height,
        buf1->y_crop_width, buf1->y_crop_height);
#endif  // CONFIG_AOM_HIGHBITDEPTH

  x->nmvjointcost = x->nmv_vec_cost[0];
  x->nmvjointsadcost = x->nmvjointcost;
  x->plane[0].src.stride = buf0->y_stride;
  xd->plane[0].dst.stride = buf1->y_stride;
  xd->plane[1].dst.stride = buf1->uv_stride;
  xd->plane[0].pre[0].stride = buf1->y_stride;

  MV gld_top_mv = { 0, 0 };

  uint8_t *buf0_start = buf0->y_buffer;
  uint8_t *buf1_start = buf1->y_buffer;

  for (mb_row = 0; mb_row < cm->mb_rows; mb_row++) {
    for (mb_col = 0; mb_col < cm->mb_cols; mb_col++) {
      float *prop_in = &mbt->prop_cost[mb_row*cm->mb_cols + mb_col];
      float *last_intra = &mbt->last_intra[mb_row*cm->mb_cols + mb_col];

      x->plane[0].src.buf = buf0_start + mb_col*64;
      xd->plane[0].dst.buf = buf1_start + mb_col*64;
      xd->plane[0].pre[0].buf = xd->plane[0].dst.buf;

      float ierr = (float)find_best_64x64_intra(cpi);
      *last_intra = ierr;

      float perr = (float)do_64x64_motion_search(cpi, &gld_top_mv, mb_row, mb_col);

      if (perr > ierr)
        perr = ierr;

      float prop_num = ierr - perr;
      float prop_denom = ierr;

      float prop_fraction = prop_num/prop_denom;
      float prop_amount = (ierr + *prop_in)*prop_fraction;
      *prop_in += prop_amount;
    }
    buf0_start += buf0->y_stride;
    buf1_start += buf1->y_stride;
  }
}

void av1_mbtree_update(struct AV1_COMP *cpi)
{
  MBTreeContext *mbt = &cpi->mbtree;
  AV1_COMMON *const cm = &cpi->common;

  memset(mbt->prop_cost, 0.0f, sizeof(float)*cm->mb_rows*cm->mb_cols);

  int last_lookahead = av1_lookahead_depth(cpi->lookahead);
  for (int i = last_lookahead - 1; i >= 0; i--)
    process_frame(cpi, mbt, i);
}

int av1_mbtree_get_mb_delta(struct AV1_COMP *cpi, int mb_row, int mb_col)
{
  MBTreeContext *mbt = &cpi->mbtree;
  AV1_COMMON *const cm = &cpi->common;
  float prop_cost = mbt->prop_cost[mb_row*cm->mb_cols + mb_col];
  float last_intra = mbt->last_intra[mb_row*cm->mb_cols + mb_col];
  float qdif = 90*((log2f((last_intra + prop_cost + 1)/(prop_cost + 1))/18.0) - 1.0f);
  if (isnan(qdif))
    return 0;
  if (qdif == 0.0f)
    return 0;
  int r = floor(qdif);
  if (r > 1)
    return r;
  return 0;
}

void av1_mbtree_init(struct AV1_COMP *cpi)
{
    MBTreeContext *mbt = &cpi->mbtree;
    AV1_COMMON *const cm = &cpi->common;

    mbt->prop_cost = aom_calloc(cm->mb_rows*cm->mb_cols + 1, sizeof(*mbt->prop_cost));
    mbt->last_intra = aom_calloc(cm->mb_rows*cm->mb_cols + 1, sizeof(*mbt->last_intra));
    mbt->scratch_buf = aom_calloc(cm->render_width*cm->render_height, 6);
}

void av1_mbtree_uninit(struct AV1_COMP *cpi)
{
    MBTreeContext *mbt = &cpi->mbtree;

    aom_free(mbt->prop_cost);
    aom_free(mbt->last_intra);
    aom_free(mbt->scratch_buf);
}
