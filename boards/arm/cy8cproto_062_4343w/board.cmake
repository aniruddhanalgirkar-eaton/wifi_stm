# Copyright (c) 2021 Cypress Semiconductor Corporation.
# SPDX-License-Identifier: Apache-2.0

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

board_runner_args(pyocd "--target=cy8c6xxa")
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
