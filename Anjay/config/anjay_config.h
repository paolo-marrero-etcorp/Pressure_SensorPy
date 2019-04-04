/*
 * Copyright 2017-2019 AVSystem <avsystem@avsystem.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define ANJAY_VERSION "1.15.2"

#ifndef ANJAY_TEST
#define HAVE_VISIBILITY
#endif

#ifdef HAVE_VISIBILITY
/* set default visibility for external symbols */
#pragma GCC visibility push(default)
#define VISIBILITY_SOURCE_BEGIN         _Pragma("GCC visibility push(hidden)")
#define VISIBILITY_PRIVATE_HEADER_BEGIN _Pragma("GCC visibility push(hidden)")
#define VISIBILITY_PRIVATE_HEADER_END   _Pragma("GCC visibility pop")
#else
#define VISIBILITY_SOURCE_BEGIN
#define VISIBILITY_PRIVATE_HEADER_BEGIN
#define VISIBILITY_PRIVATE_HEADER_END
#endif

/* #undef AVS_LOG_WITH_TRACE */

#define WITH_ACCESS_CONTROL
#define WITH_AVS_LOG
#define WITH_BLOCK_DOWNLOAD
#define WITH_BLOCK_RECEIVE
#define WITH_BLOCK_SEND
#define WITH_BOOTSTRAP
#define WITH_DISCOVER
#define WITH_DOWNLOADER
#define WITH_OBSERVE
/* #undef WITH_HTTP_DOWNLOAD */
#define WITH_JSON
/* #undef WITH_CON_ATTR */
/* #undef WITH_LEGACY_CONTENT_FORMAT_SUPPORT */
#define WITH_NET_STATS
#define WITH_AVS_PERSISTENCE

#define ANJAY_MAX_PK_OR_IDENTITY_SIZE 2048
#define ANJAY_MAX_SERVER_PK_OR_IDENTITY_SIZE 2048
#define ANJAY_MAX_SECRET_KEY_SIZE 256

#define ANJAY_MAX_OBSERVABLE_RESOURCE_SIZE 2048

#define ANJAY_MAX_FLOAT_STRING_SIZE 64
#define ANJAY_MAX_DOUBLE_STRING_SIZE 512

#define ANJAY_MAX_URI_SEGMENT_SIZE 256
#define ANJAY_MAX_URI_QUERY_SEGMENT_SIZE 256

#define ANJAY_DTLS_SESSION_BUFFER_SIZE 1024
