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

#ifndef ANJAY_CONFIG_LOG_H
#define ANJAY_CONFIG_LOG_H

#include <anjay_modules/utils_core.h>

VISIBILITY_PRIVATE_HEADER_BEGIN

static inline void _anjay_log_feature_list(void) {

    _anjay_log(anjay, TRACE, "ANJAY_DTLS_SESSION_BUFFER_SIZE = " AVS_QUOTE_MACRO(ANJAY_DTLS_SESSION_BUFFER_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_DOUBLE_STRING_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_DOUBLE_STRING_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_FLOAT_STRING_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_FLOAT_STRING_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_OBSERVABLE_RESOURCE_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_OBSERVABLE_RESOURCE_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_PK_OR_IDENTITY_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_PK_OR_IDENTITY_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_SECRET_KEY_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_SECRET_KEY_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_SERVER_PK_OR_IDENTITY_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_SERVER_PK_OR_IDENTITY_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_URI_QUERY_SEGMENT_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_URI_QUERY_SEGMENT_SIZE));
    _anjay_log(anjay, TRACE, "ANJAY_MAX_URI_SEGMENT_SIZE = " AVS_QUOTE_MACRO(ANJAY_MAX_URI_SEGMENT_SIZE));
#ifdef AVS_LOG_WITH_TRACE
    _anjay_log(anjay, TRACE, "AVS_LOG_WITH_TRACE = ON");
#else // AVS_LOG_WITH_TRACE
    _anjay_log(anjay, TRACE, "AVS_LOG_WITH_TRACE = OFF");
#endif // AVS_LOG_WITH_TRACE
#ifdef HAVE_VISIBILITY
    _anjay_log(anjay, TRACE, "HAVE_VISIBILITY = ON");
#else // HAVE_VISIBILITY
    _anjay_log(anjay, TRACE, "HAVE_VISIBILITY = OFF");
#endif // HAVE_VISIBILITY
#ifdef WITH_ACCESS_CONTROL
    _anjay_log(anjay, TRACE, "WITH_ACCESS_CONTROL = ON");
#else // WITH_ACCESS_CONTROL
    _anjay_log(anjay, TRACE, "WITH_ACCESS_CONTROL = OFF");
#endif // WITH_ACCESS_CONTROL
#ifdef WITH_AVS_LOG
    _anjay_log(anjay, TRACE, "WITH_AVS_LOG = ON");
#else // WITH_AVS_LOG
    _anjay_log(anjay, TRACE, "WITH_AVS_LOG = OFF");
#endif // WITH_AVS_LOG
#ifdef WITH_AVS_PERSISTENCE
    _anjay_log(anjay, TRACE, "WITH_AVS_PERSISTENCE = ON");
#else // WITH_AVS_PERSISTENCE
    _anjay_log(anjay, TRACE, "WITH_AVS_PERSISTENCE = OFF");
#endif // WITH_AVS_PERSISTENCE
#ifdef WITH_BLOCK_DOWNLOAD
    _anjay_log(anjay, TRACE, "WITH_BLOCK_DOWNLOAD = ON");
#else // WITH_BLOCK_DOWNLOAD
    _anjay_log(anjay, TRACE, "WITH_BLOCK_DOWNLOAD = OFF");
#endif // WITH_BLOCK_DOWNLOAD
#ifdef WITH_BLOCK_RECEIVE
    _anjay_log(anjay, TRACE, "WITH_BLOCK_RECEIVE = ON");
#else // WITH_BLOCK_RECEIVE
    _anjay_log(anjay, TRACE, "WITH_BLOCK_RECEIVE = OFF");
#endif // WITH_BLOCK_RECEIVE
#ifdef WITH_BLOCK_SEND
    _anjay_log(anjay, TRACE, "WITH_BLOCK_SEND = ON");
#else // WITH_BLOCK_SEND
    _anjay_log(anjay, TRACE, "WITH_BLOCK_SEND = OFF");
#endif // WITH_BLOCK_SEND
#ifdef WITH_BOOTSTRAP
    _anjay_log(anjay, TRACE, "WITH_BOOTSTRAP = ON");
#else // WITH_BOOTSTRAP
    _anjay_log(anjay, TRACE, "WITH_BOOTSTRAP = OFF");
#endif // WITH_BOOTSTRAP
#ifdef WITH_CON_ATTR
    _anjay_log(anjay, TRACE, "WITH_CON_ATTR = ON");
#else // WITH_CON_ATTR
    _anjay_log(anjay, TRACE, "WITH_CON_ATTR = OFF");
#endif // WITH_CON_ATTR
#ifdef WITH_DISCOVER
    _anjay_log(anjay, TRACE, "WITH_DISCOVER = ON");
#else // WITH_DISCOVER
    _anjay_log(anjay, TRACE, "WITH_DISCOVER = OFF");
#endif // WITH_DISCOVER
#ifdef WITH_DOWNLOADER
    _anjay_log(anjay, TRACE, "WITH_DOWNLOADER = ON");
#else // WITH_DOWNLOADER
    _anjay_log(anjay, TRACE, "WITH_DOWNLOADER = OFF");
#endif // WITH_DOWNLOADER
#ifdef WITH_HTTP_DOWNLOAD
    _anjay_log(anjay, TRACE, "WITH_HTTP_DOWNLOAD = ON");
#else // WITH_HTTP_DOWNLOAD
    _anjay_log(anjay, TRACE, "WITH_HTTP_DOWNLOAD = OFF");
#endif // WITH_HTTP_DOWNLOAD
#ifdef WITH_JSON
    _anjay_log(anjay, TRACE, "WITH_JSON = ON");
#else // WITH_JSON
    _anjay_log(anjay, TRACE, "WITH_JSON = OFF");
#endif // WITH_JSON
#ifdef WITH_LEGACY_CONTENT_FORMAT_SUPPORT
    _anjay_log(anjay, TRACE, "WITH_LEGACY_CONTENT_FORMAT_SUPPORT = ON");
#else // WITH_LEGACY_CONTENT_FORMAT_SUPPORT
    _anjay_log(anjay, TRACE, "WITH_LEGACY_CONTENT_FORMAT_SUPPORT = OFF");
#endif // WITH_LEGACY_CONTENT_FORMAT_SUPPORT
#ifdef WITH_NET_STATS
    _anjay_log(anjay, TRACE, "WITH_NET_STATS = ON");
#else // WITH_NET_STATS
    _anjay_log(anjay, TRACE, "WITH_NET_STATS = OFF");
#endif // WITH_NET_STATS
#ifdef WITH_OBSERVE
    _anjay_log(anjay, TRACE, "WITH_OBSERVE = ON");
#else // WITH_OBSERVE
    _anjay_log(anjay, TRACE, "WITH_OBSERVE = OFF");
#endif // WITH_OBSERVE
}

VISIBILITY_PRIVATE_HEADER_END

#endif /* ANJAY_CONFIG_LOG_H */
