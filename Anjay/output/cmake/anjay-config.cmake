# Copyright 2017-2019 AVSystem <avsystem@avsystem.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(ANJAY_VERSION "1.15.2")

get_filename_component(CURR_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

find_package(avs_commons REQUIRED COMPONENTS algorithm;coap;list;vector;rbtree;buffer;net;stream;stream_net;utils;compat_threading;persistence)

include(${CURR_DIR}/anjay-targets.cmake)

get_filename_component(ANJAY_INCLUDE_DIRS "${CURR_DIR}/../../include" ABSOLUTE)
set(ANJAY_INCLUDE_DIRS "${ANJAY_INCLUDE_DIRS}" "${AVS_COMMONS_INCLUDE_DIRS}")
set(ANJAY_LIBRARIES anjay)
set(ANJAY_LIBRARIES_STATIC anjay_static)

unset(CURR_DIR)
