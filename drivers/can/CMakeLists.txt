# ##############################################################################
# drivers/can/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS)

if(CONFIG_CAN OR CONFIG_NET_CAN)
  list(APPEND SRCS can_common.c)
endif()

if(CONFIG_CAN)
  list(APPEND SRCS can.c can_sender.c)
endif()

if(CONFIG_CAN_MCP2515)
  list(APPEND SRCS mcp2515.c)
endif()

if(CONFIG_CAN_KVASER)
  list(APPEND SRCS kvaser_pci.c)
endif()

if(CONFIG_CAN_CTUCANFD)
  list(APPEND SRCS ctucanfd_pci.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
