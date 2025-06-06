/****************************************************************************
 * arch/arm64/include/goldfish/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM64_INCLUDE_GOLDFISH_IRQ_H
#define __ARCH_ARM64_INCLUDE_GOLDFISH_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NR_IRQS            220  /* Total number of interrupts */
#define MPID_TO_CORE(mpid) (((mpid) >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK)

#endif /* __ARCH_ARM64_INCLUDE_GOLDFISH_IRQ_H */
