# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-FileCopyrightText: Copyright (c) 2026 Philipp Steiner
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(precision_timing)

target_sources(app PRIVATE
  src/clock_sync.c
  src/main.c
)

target_include_directories(app PRIVATE
  ${ZEPHYR_BASE}/subsys/precision_timing
)
