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

#define SIMD_CHECK 1
#include "third_party/googletest/src/googletest/include/gtest/gtest.h"
#include "test/clear_system_state.h"
#include "test/register_state_check.h"
#include "aom_dsp/aom_simd_inline.h"
#include "aom_dsp/simd/v128_intrinsics_c.h"

namespace SIMD_NAMESPACE {

template <typename param_signature>
class TestIntrinsic : public ::testing::TestWithParam<param_signature> {
 public:
  virtual ~TestIntrinsic() {}
  virtual void SetUp() {
    mask = std::tr1::get<0>(this->GetParam());
    maskwidth = std::tr1::get<1>(this->GetParam());
    name = std::tr1::get<2>(this->GetParam());
  }

  virtual void TearDown() { libaom_test::ClearSystemState(); }

 protected:
  uint32_t mask, maskwidth;
  const char *name;
};

// Create one typedef for each function signature
#define TYPEDEF_SIMD(name)                                                  \
  typedef TestIntrinsic<std::tr1::tuple<uint32_t, uint32_t, const char *> > \
      ARCH_POSTFIX(name)

TYPEDEF_SIMD(V64_U8);
TYPEDEF_SIMD(V64_U16);
TYPEDEF_SIMD(V64_U32);
TYPEDEF_SIMD(V64_V64);
TYPEDEF_SIMD(U32_V64);
TYPEDEF_SIMD(S32_V64);
TYPEDEF_SIMD(U64_V64);
TYPEDEF_SIMD(S64_V64);
TYPEDEF_SIMD(V64_U32U32);
TYPEDEF_SIMD(V64_V64V64);
TYPEDEF_SIMD(S64_V64V64);
TYPEDEF_SIMD(V64_V64U32);
TYPEDEF_SIMD(U32_V64V64);
TYPEDEF_SIMD(V128_V64);
TYPEDEF_SIMD(V128_V128);
TYPEDEF_SIMD(U32_V128);
TYPEDEF_SIMD(U64_V128);
TYPEDEF_SIMD(V64_V128);
TYPEDEF_SIMD(V128_U8);
TYPEDEF_SIMD(V128_U16);
TYPEDEF_SIMD(V128_U32);
TYPEDEF_SIMD(V128_U64U64);
TYPEDEF_SIMD(V128_V64V64);
TYPEDEF_SIMD(V128_V128V128);
TYPEDEF_SIMD(S64_V128V128);
TYPEDEF_SIMD(V128_V128U32);
TYPEDEF_SIMD(U32_V128V128);

// Google Test allows up to 50 tests per case, so split the largest
typedef ARCH_POSTFIX(V64_V64) ARCH_POSTFIX(V64_V64_Part2);
typedef ARCH_POSTFIX(V64_V64V64) ARCH_POSTFIX(V64_V64V64_Part2);
typedef ARCH_POSTFIX(V128_V128) ARCH_POSTFIX(V128_V128_Part2);
typedef ARCH_POSTFIX(V128_V128) ARCH_POSTFIX(V128_V128_Part3);
typedef ARCH_POSTFIX(V128_V128V128) ARCH_POSTFIX(V128_V128V128_Part2);

// These functions are machine tuned located elsewhere
template <typename c_ret, typename c_arg>
void TestSimd1Arg(uint32_t iterations, uint32_t mask, uint32_t maskwidth,
                  const char *name);

template <typename c_ret, typename c_arg1, typename c_arg2>
void TestSimd2Args(uint32_t iterations, uint32_t mask, uint32_t maskwidth,
                   const char *name);

const int kIterations = 65536;

// Add a macro layer since TEST_P will quote the name so we need to
// expand it first with the prefix.
#define MY_TEST_P(name, test) TEST_P(name, test)

MY_TEST_P(ARCH_POSTFIX(V64_U8), TestIntrinsics) {
  TestSimd1Arg<c_v64, uint8_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_U16), TestIntrinsics) {
  TestSimd1Arg<c_v64, uint16_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_U32), TestIntrinsics) {
  TestSimd1Arg<c_v64, uint32_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_V64), TestIntrinsics) {
  TestSimd1Arg<c_v64, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(U64_V64), TestIntrinsics) {
  TestSimd1Arg<uint64_t, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(S64_V64), TestIntrinsics) {
  TestSimd1Arg<int64_t, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(U32_V64), TestIntrinsics) {
  TestSimd1Arg<uint32_t, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(S32_V64), TestIntrinsics) {
  TestSimd1Arg<int32_t, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_U32U32), TestIntrinsics) {
  TestSimd2Args<c_v64, uint32_t, uint32_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_V64V64), TestIntrinsics) {
  TestSimd2Args<c_v64, c_v64, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(S64_V64V64), TestIntrinsics) {
  TestSimd2Args<int64_t, c_v64, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(U32_V64V64), TestIntrinsics) {
  TestSimd2Args<uint32_t, c_v64, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_V64U32), TestIntrinsics) {
  TestSimd2Args<c_v64, c_v64, uint32_t>(kIterations, mask, maskwidth, name);
}

// Google Test allows up to 50 tests per case, so split the largest
MY_TEST_P(ARCH_POSTFIX(V64_V64_Part2), TestIntrinsics) {
  TestSimd1Arg<c_v64, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_V64V64_Part2), TestIntrinsics) {
  TestSimd2Args<c_v64, c_v64, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(U32_V128), TestIntrinsics) {
  TestSimd1Arg<uint32_t, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(U64_V128), TestIntrinsics) {
  TestSimd1Arg<uint64_t, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V64_V128), TestIntrinsics) {
  TestSimd1Arg<c_v64, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V128), TestIntrinsics) {
  TestSimd1Arg<c_v128, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_U8), TestIntrinsics) {
  TestSimd1Arg<c_v128, uint8_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_U16), TestIntrinsics) {
  TestSimd1Arg<c_v128, uint16_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_U32), TestIntrinsics) {
  TestSimd1Arg<c_v128, uint32_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V64), TestIntrinsics) {
  TestSimd1Arg<c_v128, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V128V128), TestIntrinsics) {
  TestSimd2Args<c_v128, c_v128, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(U32_V128V128), TestIntrinsics) {
  TestSimd2Args<uint32_t, c_v128, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(S64_V128V128), TestIntrinsics) {
  TestSimd2Args<int64_t, c_v128, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_U64U64), TestIntrinsics) {
  TestSimd2Args<c_v128, uint64_t, uint64_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V64V64), TestIntrinsics) {
  TestSimd2Args<c_v128, c_v64, c_v64>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V128U32), TestIntrinsics) {
  TestSimd2Args<c_v128, c_v128, uint32_t>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V128V128_Part2), TestIntrinsics) {
  TestSimd2Args<c_v128, c_v128, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V128_Part2), TestIntrinsics) {
  TestSimd1Arg<c_v128, c_v128>(kIterations, mask, maskwidth, name);
}

MY_TEST_P(ARCH_POSTFIX(V128_V128_Part3), TestIntrinsics) {
  TestSimd1Arg<c_v128, c_v128>(kIterations, mask, maskwidth, name);
}

// Add a macro layer since INSTANTIATE_TEST_CASE_P will quote the name
// so we need to expand it first with the prefix
#define INSTANTIATE(name, type, ...) \
  INSTANTIATE_TEST_CASE_P(name, type, ::testing::Values(__VA_ARGS__))

#define SIMD_TUPLE(name, mask, maskwidth) \
  std::tr1::make_tuple(mask, maskwidth, static_cast<const char *>(#name))

INSTANTIATE(ARCH, ARCH_POSTFIX(U32_V64V64),
            (SIMD_TUPLE(v64_sad_u8, 0U, 0U), SIMD_TUPLE(v64_ssd_u8, 0U, 0U)));

INSTANTIATE(
    ARCH, ARCH_POSTFIX(V64_V64V64), SIMD_TUPLE(v64_add_8, 0U, 0U),
    SIMD_TUPLE(v64_add_16, 0U, 0U), SIMD_TUPLE(v64_sadd_s16, 0U, 0U),
    SIMD_TUPLE(v64_add_32, 0U, 0U), SIMD_TUPLE(v64_sub_8, 0U, 0U),
    SIMD_TUPLE(v64_ssub_u8, 0U, 0U), SIMD_TUPLE(v64_ssub_s8, 0U, 0U),
    SIMD_TUPLE(v64_sub_16, 0U, 0U), SIMD_TUPLE(v64_ssub_s16, 0U, 0U),
    SIMD_TUPLE(v64_sub_32, 0U, 0U), SIMD_TUPLE(v64_ziplo_8, 0U, 0U),
    SIMD_TUPLE(v64_ziphi_8, 0U, 0U), SIMD_TUPLE(v64_ziplo_16, 0U, 0U),
    SIMD_TUPLE(v64_ziphi_16, 0U, 0U), SIMD_TUPLE(v64_ziplo_32, 0U, 0U),
    SIMD_TUPLE(v64_ziphi_32, 0U, 0U), SIMD_TUPLE(v64_pack_s32_s16, 0U, 0U),
    SIMD_TUPLE(v64_pack_s16_u8, 0U, 0U), SIMD_TUPLE(v64_pack_s16_s8, 0U, 0U),
    SIMD_TUPLE(v64_unziphi_8, 0U, 0U), SIMD_TUPLE(v64_unziplo_8, 0U, 0U),
    SIMD_TUPLE(v64_unziphi_16, 0U, 0U), SIMD_TUPLE(v64_unziplo_16, 0U, 0U),
    SIMD_TUPLE(v64_or, 0U, 0U), SIMD_TUPLE(v64_xor, 0U, 0U),
    SIMD_TUPLE(v64_and, 0U, 0U), SIMD_TUPLE(v64_andn, 0U, 0U),
    SIMD_TUPLE(v64_mullo_s16, 0U, 0U), SIMD_TUPLE(v64_mulhi_s16, 0U, 0U),
    SIMD_TUPLE(v64_mullo_s32, 0U, 0U), SIMD_TUPLE(v64_madd_s16, 0U, 0U),
    SIMD_TUPLE(v64_madd_us8, 0U, 0U), SIMD_TUPLE(v64_avg_u8, 0U, 0U),
    SIMD_TUPLE(v64_rdavg_u8, 0U, 0U), SIMD_TUPLE(v64_avg_u16, 0U, 0U),
    SIMD_TUPLE(v64_min_u8, 0U, 0U), SIMD_TUPLE(v64_max_u8, 0U, 0U),
    SIMD_TUPLE(v64_min_s8, 0U, 0U), SIMD_TUPLE(v64_max_s8, 0U, 0U),
    SIMD_TUPLE(v64_min_s16, 0U, 0U), SIMD_TUPLE(v64_max_s16, 0U, 0U),
    SIMD_TUPLE(v64_cmpgt_s8, 0U, 0U), SIMD_TUPLE(v64_cmplt_s8, 0U, 0U),
    SIMD_TUPLE(v64_cmpeq_8, 0U, 0U), SIMD_TUPLE(v64_cmpgt_s16, 0U, 0U),
    SIMD_TUPLE(v64_cmplt_s16, 0U, 0U), SIMD_TUPLE(v64_cmpeq_16, 0U, 0U),
    SIMD_TUPLE(v64_shuffle_8, 7U, 8U));

INSTANTIATE(
    ARCH, ARCH_POSTFIX(V64_V64V64_Part2), SIMD_TUPLE(imm_v64_align<1>, 0U, 0U),
    SIMD_TUPLE(imm_v64_align<2>, 0U, 0U), SIMD_TUPLE(imm_v64_align<3>, 0U, 0U),
    SIMD_TUPLE(imm_v64_align<4>, 0U, 0U), SIMD_TUPLE(imm_v64_align<5>, 0U, 0U),
    SIMD_TUPLE(imm_v64_align<6>, 0U, 0U), SIMD_TUPLE(imm_v64_align<7>, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_V64), SIMD_TUPLE(v64_abs_s16, 0U, 0U),
            SIMD_TUPLE(v64_unpacklo_u8_s16, 0U, 0U),
            SIMD_TUPLE(v64_unpackhi_u8_s16, 0U, 0U),
            SIMD_TUPLE(v64_unpacklo_u16_s32, 0U, 0U),
            SIMD_TUPLE(v64_unpacklo_s16_s32, 0U, 0U),
            SIMD_TUPLE(v64_unpackhi_u16_s32, 0U, 0U),
            SIMD_TUPLE(v64_unpackhi_s16_s32, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_byte<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_byte<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_byte<3>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_byte<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_byte<5>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_byte<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_byte<7>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_byte<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_byte<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_byte<3>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_byte<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_byte<5>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_byte<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_byte<7>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_8<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_8<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_8<3>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_8<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_8<5>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_8<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_8<7>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u8<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u8<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u8<3>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u8<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u8<5>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u8<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u8<7>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s8<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s8<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s8<3>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s8<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s8<5>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s8<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s8<7>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<8>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<10>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<12>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_16<14>, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_V64_Part2),
            SIMD_TUPLE(imm_v64_shr_n_u16<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u16<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u16<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u16<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u16<8>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u16<10>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u16<12>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u16<14>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<2>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<6>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<8>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<10>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<12>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s16<14>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<8>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<12>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<16>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<20>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<24>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shl_n_32<28>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<8>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<12>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<16>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<20>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<24>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_u32<28>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<1>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<4>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<8>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<12>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<16>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<20>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<24>, 0U, 0U),
            SIMD_TUPLE(imm_v64_shr_n_s32<28>, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_V64U32), SIMD_TUPLE(v64_shl_8, 7U, 32U),
            SIMD_TUPLE(v64_shr_u8, 7U, 32U), SIMD_TUPLE(v64_shr_s8, 7U, 32U),
            SIMD_TUPLE(v64_shl_16, 15U, 32U), SIMD_TUPLE(v64_shr_u16, 15U, 32U),
            SIMD_TUPLE(v64_shr_s16, 15U, 32U), SIMD_TUPLE(v64_shl_32, 31U, 32U),
            SIMD_TUPLE(v64_shr_u32, 31U, 32U),
            SIMD_TUPLE(v64_shr_s32, 31U, 32U));

INSTANTIATE(ARCH, ARCH_POSTFIX(U64_V64), SIMD_TUPLE(v64_hadd_u8, 0U, 0U),
            SIMD_TUPLE(v64_u64, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(S64_V64), SIMD_TUPLE(v64_hadd_s16, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(U32_V64), SIMD_TUPLE(v64_low_u32, 0U, 0U),
            SIMD_TUPLE(v64_high_u32, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(S32_V64), SIMD_TUPLE(v64_low_s32, 0U, 0U),
            SIMD_TUPLE(v64_high_s32, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(S64_V64V64), SIMD_TUPLE(v64_dotp_s16, 0U, 0U),
            SIMD_TUPLE(v64_dotp_su8, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_U8), SIMD_TUPLE(v64_dup_8, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_U16), SIMD_TUPLE(v64_dup_16, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_U32), SIMD_TUPLE(v64_dup_32, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_U32U32), SIMD_TUPLE(v64_from_32, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(U32_V128V128), SIMD_TUPLE(v128_sad_u8, 0U, 0U),
            SIMD_TUPLE(v128_ssd_u8, 0U, 0U));

INSTANTIATE(
    ARCH, ARCH_POSTFIX(V128_V128V128), SIMD_TUPLE(v128_add_8, 0U, 0U),
    SIMD_TUPLE(v128_add_16, 0U, 0U), SIMD_TUPLE(v128_sadd_s16, 0U, 0U),
    SIMD_TUPLE(v128_add_32, 0U, 0U), SIMD_TUPLE(v128_sub_8, 0U, 0U),
    SIMD_TUPLE(v128_ssub_u8, 0U, 0U), SIMD_TUPLE(v128_ssub_s8, 0U, 0U),
    SIMD_TUPLE(v128_sub_16, 0U, 0U), SIMD_TUPLE(v128_ssub_s16, 0U, 0U),
    SIMD_TUPLE(v128_sub_32, 0U, 0U), SIMD_TUPLE(v128_ziplo_8, 0U, 0U),
    SIMD_TUPLE(v128_ziphi_8, 0U, 0U), SIMD_TUPLE(v128_ziplo_16, 0U, 0U),
    SIMD_TUPLE(v128_ziphi_16, 0U, 0U), SIMD_TUPLE(v128_ziplo_32, 0U, 0U),
    SIMD_TUPLE(v128_ziphi_32, 0U, 0U), SIMD_TUPLE(v128_ziplo_64, 0U, 0U),
    SIMD_TUPLE(v128_ziphi_64, 0U, 0U), SIMD_TUPLE(v128_unziphi_8, 0U, 0U),
    SIMD_TUPLE(v128_unziplo_8, 0U, 0U), SIMD_TUPLE(v128_unziphi_16, 0U, 0U),
    SIMD_TUPLE(v128_unziplo_16, 0U, 0U), SIMD_TUPLE(v128_unziphi_32, 0U, 0U),
    SIMD_TUPLE(v128_unziplo_32, 0U, 0U), SIMD_TUPLE(v128_pack_s32_s16, 0U, 0U),
    SIMD_TUPLE(v128_pack_s16_u8, 0U, 0U), SIMD_TUPLE(v128_pack_s16_s8, 0U, 0U),
    SIMD_TUPLE(v128_or, 0U, 0U), SIMD_TUPLE(v128_xor, 0U, 0U),
    SIMD_TUPLE(v128_and, 0U, 0U), SIMD_TUPLE(v128_andn, 0U, 0U),
    SIMD_TUPLE(v128_mullo_s16, 0U, 0U), SIMD_TUPLE(v128_mulhi_s16, 0U, 0U),
    SIMD_TUPLE(v128_mullo_s32, 0U, 0U), SIMD_TUPLE(v128_madd_s16, 0U, 0U),
    SIMD_TUPLE(v128_madd_us8, 0U, 0U), SIMD_TUPLE(v128_avg_u8, 0U, 0U),
    SIMD_TUPLE(v128_rdavg_u8, 0U, 0U), SIMD_TUPLE(v128_avg_u16, 0U, 0U),
    SIMD_TUPLE(v128_min_u8, 0U, 0U), SIMD_TUPLE(v128_max_u8, 0U, 0U),
    SIMD_TUPLE(v128_min_s8, 0U, 0U), SIMD_TUPLE(v128_max_s8, 0U, 0U),
    SIMD_TUPLE(v128_min_s16, 0U, 0U), SIMD_TUPLE(v128_max_s16, 0U, 0U),
    SIMD_TUPLE(v128_cmpgt_s8, 0U, 0U), SIMD_TUPLE(v128_cmplt_s8, 0U, 0U),
    SIMD_TUPLE(v128_cmpeq_8, 0U, 0U), SIMD_TUPLE(v128_cmpgt_s16, 0U, 0U),
    SIMD_TUPLE(v128_cmpeq_16, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_V128V128_Part2),
            SIMD_TUPLE(v128_cmplt_s16, 0U, 0U),
            SIMD_TUPLE(v128_shuffle_8, 15U, 8U),
            SIMD_TUPLE(imm_v128_align<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<3>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<5>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<7>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<9>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<10>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<11>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<13>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<14>, 0U, 0U),
            SIMD_TUPLE(imm_v128_align<15>, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_V128), SIMD_TUPLE(v128_abs_s16, 0U, 0U),
            SIMD_TUPLE(v128_padd_s16, 0U, 0U),
            SIMD_TUPLE(v128_unpacklo_u8_s16, 0U, 0U),
            SIMD_TUPLE(v128_unpacklo_u16_s32, 0U, 0U),
            SIMD_TUPLE(v128_unpacklo_s16_s32, 0U, 0U),
            SIMD_TUPLE(v128_unpackhi_u8_s16, 0U, 0U),
            SIMD_TUPLE(v128_unpackhi_u16_s32, 0U, 0U),
            SIMD_TUPLE(v128_unpackhi_s16_s32, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<3>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<5>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<7>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<9>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<10>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<11>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<13>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<14>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_byte<15>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<3>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<5>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<7>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<9>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<10>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<11>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<13>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<14>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_byte<15>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_8<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_8<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_8<3>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_8<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_8<5>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_8<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_8<7>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u8<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u8<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u8<3>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u8<4>, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_V128_Part2),
            SIMD_TUPLE(imm_v128_shr_n_u8<5>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u8<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u8<7>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s8<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s8<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s8<3>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s8<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s8<5>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s8<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s8<7>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<10>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_16<14>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<10>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u16<14>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<2>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<6>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<10>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s16<14>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<16>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<20>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<24>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shl_n_32<28>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u32<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u32<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u32<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u32<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u32<16>, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_V128_Part3),
            SIMD_TUPLE(imm_v128_shr_n_u32<20>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u32<24>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_u32<28>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<1>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<4>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<8>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<12>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<16>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<20>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<24>, 0U, 0U),
            SIMD_TUPLE(imm_v128_shr_n_s32<28>, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_V64V64), SIMD_TUPLE(v128_from_v64, 0U, 0U),
            SIMD_TUPLE(v128_zip_8, 0U, 0U), SIMD_TUPLE(v128_zip_16, 0U, 0U),
            SIMD_TUPLE(v128_zip_32, 0U, 0U), SIMD_TUPLE(v128_mul_s16, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_U64U64), SIMD_TUPLE(v128_from_64, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_V64),
            SIMD_TUPLE(v128_unpack_u8_s16, 0U, 0U),
            SIMD_TUPLE(v128_unpack_u16_s32, 0U, 0U),
            SIMD_TUPLE(v128_unpack_s16_s32, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_V128U32), SIMD_TUPLE(v128_shl_8, 7U, 32U),
            SIMD_TUPLE(v128_shr_u8, 7U, 32U), SIMD_TUPLE(v128_shr_s8, 7U, 32U),
            SIMD_TUPLE(v128_shl_16, 15U, 32U),
            SIMD_TUPLE(v128_shr_u16, 15U, 32U),
            SIMD_TUPLE(v128_shr_s16, 15U, 32U),
            SIMD_TUPLE(v128_shl_32, 31U, 32U),
            SIMD_TUPLE(v128_shr_u32, 31U, 32U),
            SIMD_TUPLE(v128_shr_s32, 31U, 32U));

INSTANTIATE(ARCH, ARCH_POSTFIX(U32_V128), SIMD_TUPLE(v128_low_u32, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(U64_V128), SIMD_TUPLE(v128_hadd_u8, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V64_V128), SIMD_TUPLE(v128_low_v64, 0U, 0U),
            SIMD_TUPLE(v128_high_v64, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_U8), SIMD_TUPLE(v128_dup_8, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_U16), SIMD_TUPLE(v128_dup_16, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(V128_U32), SIMD_TUPLE(v128_dup_32, 0U, 0U));

INSTANTIATE(ARCH, ARCH_POSTFIX(S64_V128V128),
            SIMD_TUPLE(v128_dotp_s16, 0U, 0U));

}  // namespace SIMD_NAMESPACE
