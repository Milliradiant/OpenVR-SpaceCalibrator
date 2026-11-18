#pragma once

// The overlay's and the driver's code logs through quill. The bench builds the overlay's calibration code as it is, and gives it
// this in log.h's place: every line it would log goes to the replay, which reads what the calibration decided out of them.

#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <fmt/format.h>
#include <string>
#include <type_traits>

namespace bench {

void replayLog(std::string&& line);

template <class T>
    requires std::is_enum_v<T>
long long loggable(const T& value)
{
    return static_cast<long long>(value);
}

template <class T>
    requires(!std::is_enum_v<T>)
const T& loggable(const T& value)
{
    return value;
}

template <class... Arguments>
void replayLogFormat(const char* format, const Arguments&... arguments)
{
    replayLog(fmt::format(fmt::runtime(format), loggable(arguments)...));
}

} // namespace bench

#define BENCH_LOG(format, ...) ::bench::replayLogFormat(format, ##__VA_ARGS__)

#define LOG_DEBUG(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_NOTICE(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_WARN(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CRITICAL(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) BENCH_LOG(format, ##__VA_ARGS__)

#define LOG_OPENVR_DEBUG(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_OPENVR_INFO(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_OPENVR_WARNING(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_OPENVR_WARN(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_OPENVR_ERROR(format, ...) BENCH_LOG(format, ##__VA_ARGS__)

#define LOG_IPC_DEBUG(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_IPC_INFO(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_IPC_WARNING(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_IPC_WARN(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_IPC_ERROR(format, ...) BENCH_LOG(format, ##__VA_ARGS__)

#define LOG_CALIB_DEBUG(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CALIB_INFO(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CALIB_NOTICE(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CALIB_WARNING(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CALIB_WARN(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CALIB_ERROR(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CALIB_CRITICAL(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
#define LOG_CALIB_FATAL(format, ...) BENCH_LOG(format, ##__VA_ARGS__)
