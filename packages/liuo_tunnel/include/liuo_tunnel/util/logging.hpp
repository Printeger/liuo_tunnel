// Adapted from IAP include/iap/util/logging.hpp @ fbccae168; namespace and independent project paths. See THIRD_PARTY_NOTICES.md.
#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ringbuffer_sink.h>

namespace liuo_tunnel {

std::shared_ptr<spdlog::logger> get_default_logger();

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger);

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size = 128);

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name);

std::string sanitize_log_name(const std::string& name);

}  // namespace liuo_tunnel
