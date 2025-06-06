/****************************************************************************
 * arch/risc-v/include/elf.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_RISCV_INCLUDE_ELF_H
#define __ARCH_RISCV_INCLUDE_ELF_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* https://github.com/riscv/riscv-elf-psabi-doc/blob/master/riscv-elf.md */

#define R_RISCV_NONE           0
#define R_RISCV_32             1
#define R_RISCV_64             2
#define R_RISCV_RELATIVE       3
#define R_RISCV_COPY           4
#define R_RISCV_JUMP_SLOT      5
#define R_RISCV_TLS_DTPMOD32   6
#define R_RISCV_TLS_DTPMOD64   7
#define R_RISCV_TLS_DTPREL32   8
#define R_RISCV_TLS_DTPREL64   9
#define R_RISCV_TLS_TPREL32   10
#define R_RISCV_TLS_TPREL64   11

#define R_RISCV_BRANCH        16
#define R_RISCV_JAL           17
#define R_RISCV_CALL          18
#define R_RISCV_CALL_PLT      19
#define R_RISCV_GOT_HI20      20
#define R_RISCV_TLS_GOT_HI20  21
#define R_RISCV_TLS_GD_HI20   22
#define R_RISCV_PCREL_HI20    23
#define R_RISCV_PCREL_LO12_I  24
#define R_RISCV_PCREL_LO12_S  25
#define R_RISCV_HI20          26
#define R_RISCV_LO12_I        27
#define R_RISCV_LO12_S        28
#define R_RISCV_TPREL_HI20    29
#define R_RISCV_TPREL_LO12_I  30
#define R_RISCV_TPREL_LO12_S  31
#define R_RISCV_TPREL_ADD     32
#define R_RISCV_ADD8          33
#define R_RISCV_ADD16         34
#define R_RISCV_ADD32         35
#define R_RISCV_ADD64         36
#define R_RISCV_SUB8          37
#define R_RISCV_SUB16         38
#define R_RISCV_SUB32         39
#define R_RISCV_SUB64         40
#define R_RISCV_GNU_VTINHERIT 41
#define R_RISCV_GNU_VTENTRY   42
#define R_RISCV_ALIGN         43
#define R_RISCV_RVC_BRANCH    44
#define R_RISCV_RVC_JUMP      45
#define R_RISCV_RVC_LUI       46
#define R_RISCV_GPREL_I       47
#define R_RISCV_GPREL_S       48
#define R_RISCV_TPREL_I       49
#define R_RISCV_TPREL_S       50
#define R_RISCV_RELAX         51
#define R_RISCV_SUB6          52
#define R_RISCV_SET6          53
#define R_RISCV_SET8          54
#define R_RISCV_SET16         55
#define R_RISCV_SET32         56
#define R_RISCV_32_PCREL      57

#define ARCH_ELFDATA          1
#define ARCH_ELF_RELCNT       8

#define EM_ARCH               EM_RISCV
#define EF_FLAG               0

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct arch_elfdata_s
{
  struct hi20_rels_s
  {
    uintptr_t hi20_rel;
    uintptr_t hi20_offset;
  }
  hi20_rels[ARCH_ELF_RELCNT];
};
typedef struct arch_elfdata_s arch_elfdata_t;

struct user_pt_regs
{
  uintreg_t regs[32];
};

#define ELF_NGREG (sizeof(struct user_pt_regs) / sizeof(uintreg_t))
typedef uintreg_t elf_gregset_t[ELF_NGREG];

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_ELF_H */
