// Adapted from IAP src/iap/util/run_log_manager.cpp @ fbccae168; namespace and independent project paths. See THIRD_PARTY_NOTICES.md.
#include <liuo_tunnel/util/run_log_manager.hpp>

#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <sys/file.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <liuo_tunnel/util/config.hpp>

namespace liuo_tunnel {

namespace {

std::unique_ptr<RunLogManager> g_run_log_manager;

std::string iso_utc_timestamp(std::time_t t) {
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

std::string run_directory_timestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
  const auto tt = std::chrono::system_clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &tt);
#else
  gmtime_r(&tt, &tm);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%dT%H%M%SZ")
      << "_" << std::setw(3) << std::setfill('0') << millis;
  return oss.str();
}

std::optional<std::string> read_command_output(const std::string& command) {
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return std::nullopt;
  }

  std::string output;
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  const int rc = pclose(pipe);
  if (rc != 0 || output.empty()) {
    return std::nullopt;
  }

  while (!output.empty() && (output.back() == '\n' || output.back() == '\r' || output.back() == ' ')) {
    output.pop_back();
  }
  return output.empty() ? std::nullopt : std::optional<std::string>(output);
}

std::string getenv_or_empty(const char* name) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

std::string safe_filename(std::string value) {
  for (char& ch : value) {
    const bool safe = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                      (ch >= '0' && ch <= '9') || ch == '_' || ch == '-';
    if (!safe) {
      ch = '_';
    }
  }
  return value.empty() ? "process" : value;
}

void atomic_write_json(const std::filesystem::path& path, const nlohmann::json& value) {
  const auto temporary = path.parent_path() /
      ("." + path.filename().string() + "." + std::to_string(::getpid()) + ".tmp");
  {
    std::ofstream ofs(temporary);
    if (!ofs.is_open()) {
      throw std::runtime_error("failed to open artifact manifest: " + temporary.string());
    }
    ofs << std::setw(2) << value << std::endl;
  }
  std::error_code ec;
  std::filesystem::rename(temporary, path, ec);
  if (ec) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("failed to publish artifact manifest '" +
                             path.string() + "': " + ec.message());
  }
}

bool truthy(const std::string& value) {
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return normalized == "1" || normalized == "true" ||
         normalized == "yes" || normalized == "on";
}

bool retention_enabled(const std::filesystem::path& root) {
  const std::string configured = getenv_or_empty("LIUO_RETENTION_ENABLED");
  if (!configured.empty()) {
    return truthy(configured);
  }
  std::error_code ec;
  const auto repository_root = std::filesystem::weakly_canonical(
      std::filesystem::path(LIUO_SOURCE_ROOT) / "log", ec);
  return !ec && root == repository_root;
}

std::optional<std::time_t> parse_utc_timestamp(const std::string& value) {
  std::tm tm{};
  std::istringstream stream(value);
  stream >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  if (stream.fail()) {
    return std::nullopt;
  }
#if defined(_WIN32)
  return _mkgmtime(&tm);
#else
  return timegm(&tm);
#endif
}

struct RetentionCandidate {
  std::filesystem::path path;
  std::time_t ended_at = 0;
};

std::optional<RetentionCandidate> load_retention_candidate(
    const std::filesystem::path& root,
    const std::filesystem::path& path) {
  static const std::regex run_id_pattern(
      R"(^\d{8}T\d{6}Z_\d{3}(_\d{2})?$)");
  std::error_code ec;
  const auto status = std::filesystem::symlink_status(path, ec);
  if (ec || status.type() != std::filesystem::file_type::directory ||
      !std::regex_match(path.filename().string(), run_id_pattern)) {
    return std::nullopt;
  }
  const auto manifest_path = path / "metadata" / "run_manifest.json";
  const auto manifest_status = std::filesystem::symlink_status(manifest_path, ec);
  if (ec || manifest_status.type() != std::filesystem::file_type::regular) {
    return std::nullopt;
  }
  nlohmann::json manifest;
  try {
    std::ifstream stream(manifest_path);
    stream >> manifest;
  } catch (const std::exception&) {
    return std::nullopt;
  }
  try {
    const std::string lifecycle = manifest.value("lifecycle", "");
    if (manifest.value("schema_version", "") != "liuo_run_artifact_v1" ||
        manifest.value("run_id", "") != path.filename().string() ||
        manifest.value("run_root", "") != root.string() ||
        manifest.value("run_dir", "") != path.string() ||
        (lifecycle != "completed" && lifecycle != "failed" && lifecycle != "interrupted") ||
        manifest.value("run_class", "") != "development" ||
        manifest.value("retention_class", "") != "ordinary") {
      return std::nullopt;
    }
    const auto ended_at = parse_utc_timestamp(manifest.value("ended_at_utc", ""));
    if (!ended_at) {
      return std::nullopt;
    }
    return RetentionCandidate{path, *ended_at};
  } catch (const nlohmann::json::exception&) {
    return std::nullopt;
  }
}

bool run_is_locked(const std::filesystem::path& run_dir) {
  const auto lock_path = run_dir / "metadata" / ".active.lock";
  std::error_code ec;
  const auto status = std::filesystem::symlink_status(lock_path, ec);
  if (status.type() == std::filesystem::file_type::not_found) {
    return false;
  }
  if (ec || status.type() != std::filesystem::file_type::regular) {
    return true;
  }
  const int fd = ::open(lock_path.c_str(), O_RDWR | O_NOFOLLOW);
  if (fd < 0) {
    return true;
  }
  const bool locked = ::flock(fd, LOCK_EX | LOCK_NB) != 0;
  if (!locked) {
    ::flock(fd, LOCK_UN);
  }
  ::close(fd);
  return locked;
}

void prune_development_runs(const std::filesystem::path& root) {
  if (!retention_enabled(root)) {
    return;
  }
  const auto lock_path = root / ".retention.lock";
  const int lock_fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_NOFOLLOW, 0600);
  if (lock_fd < 0 || ::flock(lock_fd, LOCK_EX) != 0) {
    if (lock_fd >= 0) {
      ::close(lock_fd);
    }
    spdlog::warn("[RunLogManager] retention disabled: cannot lock {}", lock_path.string());
    return;
  }

  try {
    std::vector<RetentionCandidate> candidates;
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
      if (const auto candidate = load_retention_candidate(root, entry.path())) {
        candidates.push_back(*candidate);
      }
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.path.filename().string() > rhs.path.filename().string();
              });
    const std::time_t now = std::time(nullptr);
    constexpr double seven_days_seconds = 7.0 * 24.0 * 60.0 * 60.0;
    for (std::size_t index = 3; index < candidates.size(); ++index) {
      const auto& candidate = candidates[index];
      if (std::difftime(now, candidate.ended_at) <= seven_days_seconds ||
          run_is_locked(candidate.path) ||
          !load_retention_candidate(root, candidate.path)) {
        continue;
      }
      const auto quarantine = root /
          (".retention-trash." + candidate.path.filename().string() + "." +
           std::to_string(::getpid()));
      std::error_code ec;
      std::filesystem::rename(candidate.path, quarantine, ec);
      if (ec) {
        spdlog::warn("[RunLogManager] retention could not quarantine {}: {}",
                     candidate.path.string(), ec.message());
        continue;
      }
      std::filesystem::remove_all(quarantine, ec);
      if (ec) {
        spdlog::warn("[RunLogManager] retention could not remove {}: {}",
                     quarantine.string(), ec.message());
      }
    }
  } catch (const std::exception& error) {
    spdlog::warn("[RunLogManager] retention skipped after error: {}", error.what());
  }
  ::flock(lock_fd, LOCK_UN);
  ::close(lock_fd);
}

}  // namespace

RunLogManager& RunLogManager::initialize(const std::string& process_name,
                                         const std::string& config_dir_or_empty) {
  if (!g_run_log_manager) {
    g_run_log_manager.reset(new RunLogManager(process_name, config_dir_or_empty));
    std::atexit([] {
      if (g_run_log_manager) {
        g_run_log_manager->finalize_owner_manifest();
        g_run_log_manager.reset();
      }
    });
  }
  return *g_run_log_manager;
}

RunLogManager& RunLogManager::instance() {
  if (!g_run_log_manager) {
    throw std::runtime_error("RunLogManager is not initialized");
  }
  return *g_run_log_manager;
}

RunLogManager* RunLogManager::get_if_initialized() {
  return g_run_log_manager.get();
}

RunLogManager::RunLogManager(std::string process_name, std::string config_dir_or_empty)
    : process_name_(std::move(process_name)),
      config_dir_(std::move(config_dir_or_empty)),
      start_timestamp_(run_directory_timestamp()),
      start_timestamp_iso_(iso_utc_timestamp(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()))) {
  const std::string adopted_run = getenv_or_empty("LIUO_RUN_DIR");
  if (!adopted_run.empty()) {
    const std::filesystem::path requested(adopted_run);
    if (!requested.is_absolute()) {
      throw std::invalid_argument("LIUO_RUN_DIR must be absolute");
    }
    std::error_code ec;
    run_dir_ = std::filesystem::canonical(requested, ec);
    if (ec || !std::filesystem::is_directory(run_dir_)) {
      throw std::invalid_argument("LIUO_RUN_DIR must name an existing directory: " + adopted_run);
    }
    if (run_dir_ == run_dir_.root_path()) {
      throw std::invalid_argument("LIUO_RUN_DIR cannot be filesystem root");
    }
    log_root_ = run_dir_.parent_path();
    start_timestamp_ = run_dir_.filename().string();
  } else {
    owns_run_ = true;
    log_root_ = resolve_log_root();
    prune_development_runs(log_root_);
    allocate_run_directory();
  }
  create_layout();
  if (owns_run_) {
    write_owner_manifest();
    update_latest_symlink();
    acquire_active_lock();
  }
}

const std::filesystem::path& RunLogManager::log_root() const {
  return log_root_;
}

const std::filesystem::path& RunLogManager::run_dir() const {
  return run_dir_;
}

std::filesystem::path RunLogManager::runtime_path(const std::string& name) const {
  return category_path("runtime", name);
}

std::filesystem::path RunLogManager::profiling_path(const std::string& name) const {
  return category_path("profiling", name);
}

std::filesystem::path RunLogManager::export_path(const std::string& name) const {
  return category_path("export", name);
}

std::filesystem::path RunLogManager::metadata_path(const std::string& name) const {
  return category_path("metadata", name);
}

std::filesystem::path RunLogManager::category_path(const std::string& category,
                                                   const std::string& name) const {
  const std::filesystem::path base = run_dir_ / category;
  if (name.empty()) {
    return base;
  }
  const std::filesystem::path relative(name);
  if (relative.is_absolute()) {
    throw std::invalid_argument("artifact name must be relative: " + name);
  }
  for (const auto& component : relative) {
    if (component == "..") {
      throw std::invalid_argument("artifact name must not contain '..': " + name);
    }
  }
  std::error_code ec;
  const auto resolved_base = std::filesystem::weakly_canonical(base, ec);
  if (ec) {
    throw std::invalid_argument("artifact category cannot be resolved: " + category);
  }
  const auto resolved = std::filesystem::weakly_canonical(base / relative, ec);
  if (ec || std::mismatch(
                resolved_base.begin(), resolved_base.end(), resolved.begin(), resolved.end())
                .first != resolved_base.end()) {
    throw std::invalid_argument("artifact path escapes its category: " + name);
  }
  return resolved;
}

std::filesystem::path RunLogManager::resolve_log_root() const {
  std::filesystem::path root;
  const std::string configured_root = getenv_or_empty("LIUO_RUN_ROOT");
  if (!configured_root.empty()) {
    root = std::filesystem::path(configured_root);
    if (!root.is_absolute()) {
      throw std::invalid_argument("LIUO_RUN_ROOT must be absolute");
    }
  } else {
    const std::filesystem::path source_root(LIUO_SOURCE_ROOT);
    if (std::filesystem::is_regular_file(source_root / "CMakeLists.txt") &&
        std::filesystem::is_directory(source_root / "src")) {
      root = source_root / "log";
    } else {
      const std::string xdg_state_home = getenv_or_empty("XDG_STATE_HOME");
      const std::string user_home = getenv_or_empty("HOME");
      if (!xdg_state_home.empty()) {
        root = std::filesystem::path(xdg_state_home) / "liuo_tunnel" / "log";
      } else if (!user_home.empty()) {
        root = std::filesystem::path(user_home) / ".local" / "state" / "liuo_tunnel" / "log";
      } else {
        throw std::runtime_error("cannot resolve IAP run root without HOME or XDG_STATE_HOME");
      }
    }
  }
  if (root == root.root_path()) {
    throw std::invalid_argument("LIUO_RUN_ROOT cannot be filesystem root");
  }
  std::error_code ec;
  std::filesystem::create_directories(root, ec);
  if (ec) {
    throw std::runtime_error("failed to create LIUO_RUN_ROOT '" + root.string() + "': " + ec.message());
  }
  return std::filesystem::canonical(root);
}

void RunLogManager::allocate_run_directory() {
  for (int collision = 0; collision < 100; ++collision) {
    std::ostringstream suffix;
    if (collision > 0) {
      suffix << "_" << std::setw(2) << std::setfill('0') << collision;
    }
    const auto candidate = log_root_ / (start_timestamp_ + suffix.str());
    std::error_code ec;
    if (std::filesystem::create_directory(candidate, ec)) {
      run_dir_ = std::filesystem::canonical(candidate);
      start_timestamp_ = run_dir_.filename().string();
      return;
    }
    if (!ec && std::filesystem::exists(candidate)) {
      continue;
    }
    if (ec != std::errc::file_exists) {
      throw std::runtime_error("failed to allocate run directory '" + candidate.string() + "': " + ec.message());
    }
  }
  throw std::runtime_error("could not allocate a unique run directory below " + log_root_.string());
}

void RunLogManager::create_layout() {
  for (const auto& directory : {
           runtime_path(""), profiling_path(""), export_path(""), metadata_path("")}) {
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    if (ec) {
      throw std::runtime_error("failed to create run artifact directory '" +
                               directory.string() + "': " + ec.message());
    }
  }
  for (const auto& directory : {
           runtime_path("ros"), export_path("odometry"), export_path("simulation"),
           export_path("capture"), export_path("analysis"),
           metadata_path("config"), metadata_path("processes"),
           metadata_path("manifests")}) {
    std::error_code ec;
    std::filesystem::create_directories(directory, ec);
    if (ec) {
      throw std::runtime_error("failed to create run artifact namespace '" +
                               directory.string() + "': " + ec.message());
    }
  }
}

void RunLogManager::update_latest_symlink() const {
  const auto latest = log_root_ / "latest";
  const auto lock_path = log_root_ / ".latest.lock";
  const int lock_fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_NOFOLLOW, 0600);
  if (lock_fd < 0 || ::flock(lock_fd, LOCK_EX) != 0) {
    if (lock_fd >= 0) {
      ::close(lock_fd);
    }
    throw std::runtime_error("failed to lock latest symlink update: " + lock_path.string());
  }
  std::error_code ec;
  try {
    if (std::filesystem::exists(latest, ec) && !std::filesystem::is_symlink(latest, ec)) {
      throw std::runtime_error("IAP latest path exists and is not a symlink: " + latest.string());
    }
    ec.clear();
    if (std::filesystem::is_symlink(latest, ec)) {
      const auto current = std::filesystem::read_symlink(latest, ec).filename().string();
      if (ec) {
        throw std::runtime_error("failed to read latest symlink: " + ec.message());
      }
      if (current >= run_dir_.filename().string()) {
        ::flock(lock_fd, LOCK_UN);
        ::close(lock_fd);
        return;
      }
    }

    const auto temporary = log_root_ /
        (".latest." + std::to_string(::getpid()) + "." + start_timestamp_);
    ec.clear();
    std::filesystem::remove(temporary, ec);
    ec.clear();
    std::filesystem::create_directory_symlink(run_dir_.filename(), temporary, ec);
    if (ec) {
      throw std::runtime_error("failed to create temporary latest symlink: " + ec.message());
    }
    std::filesystem::rename(temporary, latest, ec);
    if (ec) {
      std::filesystem::remove(temporary);
      throw std::runtime_error("failed to update latest symlink: " + ec.message());
    }
  } catch (...) {
    ::flock(lock_fd, LOCK_UN);
    ::close(lock_fd);
    throw;
  }
  ::flock(lock_fd, LOCK_UN);
  ::close(lock_fd);
}

std::map<std::string, std::string> RunLogManager::collect_run_info_fields() const {
  std::map<std::string, std::string> fields;

  fields["start_timestamp"] = start_timestamp_iso_;
  fields["process_name"] = process_name_;
  fields["run_directory"] = run_dir_.string();
  fields["log_root"] = log_root_.string();
  fields["working_directory"] = std::filesystem::current_path().string();
  fields["config_dir"] = config_dir_;
  fields["build_type"] = LIUO_BUILD_TYPE;
  fields["source_root"] = LIUO_SOURCE_ROOT;

  char hostname[256] = {};
  if (gethostname(hostname, sizeof(hostname)) == 0) {
    fields["hostname"] = hostname;
  }

  std::string username = getenv_or_empty("USER");
  if (username.empty()) {
    username = getenv_or_empty("LOGNAME");
  }
  if (!username.empty()) {
    fields["username"] = username;
  }

  if (const auto git_commit = read_command_output("git -C \"" + std::string(LIUO_SOURCE_ROOT) + "\" rev-parse HEAD 2>/dev/null")) {
    fields["git_commit"] = *git_commit;
  }

  if (const auto* config = GlobalConfig::get_if_initialized()) {
    fields["selected_config_root"] = config->param<std::string>("global", "config_path", std::string());
  }

  return fields;
}

void RunLogManager::write_owner_manifest() const {
  nlohmann::json manifest = {
      {"schema_version", "liuo_run_artifact_v1"},
      {"run_id", run_dir_.filename().string()},
      {"run_root", log_root_.string()},
      {"run_dir", run_dir_.string()},
      {"entrypoint", process_name_},
      {"scenario", nullptr},
      {"modules", nlohmann::json::array()},
      {"started_at_utc", start_timestamp_iso_},
      {"ended_at_utc", nullptr},
      {"lifecycle", "active"},
      {"safety_outcome", "not_applicable"},
      {"acceptance", "not_evaluated"},
      {"run_class", "development"},
      {"retention_class", "ordinary"},
      {"build", {{"build_type", LIUO_BUILD_TYPE}}},
      {"host", nlohmann::json::object()},
      {"config_snapshots", {"metadata/config"}},
      {"subordinate_manifests", nlohmann::json::array()},
      {"external_exports", nlohmann::json::array()},
  };
  if (const auto fields = collect_run_info_fields(); fields.count("hostname")) {
    manifest["host"]["hostname"] = fields.at("hostname");
  }
  nlohmann::json source = {
      {"git_commit", nullptr},
      {"git_worktree_clean", nullptr},
  };
  if (const auto commit = read_command_output(
          "git -C \"" + std::string(LIUO_SOURCE_ROOT) + "\" rev-parse HEAD 2>/dev/null")) {
    source["git_commit"] = *commit;
  }
  if (const auto status = read_command_output(
          "git -C \"" + std::string(LIUO_SOURCE_ROOT) + "\" status --porcelain 2>/dev/null")) {
    source["git_worktree_clean"] = status->empty();
  } else {
    source["git_worktree_clean"] = true;
  }
  manifest["source"] = source;
  atomic_write_json(metadata_path("run_manifest.json"), manifest);
}

void RunLogManager::acquire_active_lock() {
  const auto path = metadata_path(".active.lock");
  active_lock_fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_NOFOLLOW, 0600);
  if (active_lock_fd_ < 0 || ::flock(active_lock_fd_, LOCK_EX | LOCK_NB) != 0) {
    if (active_lock_fd_ >= 0) {
      ::close(active_lock_fd_);
      active_lock_fd_ = -1;
    }
    throw std::runtime_error("failed to acquire run active lock: " + path.string());
  }
}

void RunLogManager::finalize_owner_manifest() noexcept {
  if (!owns_run_) {
    return;
  }
  bool finalized = false;
  try {
    const auto path = metadata_path("run_manifest.json");
    nlohmann::json manifest;
    {
      std::ifstream stream(path);
      stream >> manifest;
    }
    if (manifest.value("run_id", "") != run_dir_.filename().string() ||
        manifest.value("run_dir", "") != run_dir_.string()) {
      throw std::runtime_error("owner manifest identity changed before finalization");
    }
    manifest["lifecycle"] = "completed";
    manifest["ended_at_utc"] = iso_utc_timestamp(std::time(nullptr));
    atomic_write_json(path, manifest);
    finalized = true;
  } catch (const std::exception& error) {
    spdlog::warn("[RunLogManager] failed to finalize owner manifest: {}", error.what());
  }
  if (active_lock_fd_ >= 0) {
    ::flock(active_lock_fd_, LOCK_UN);
    ::close(active_lock_fd_);
    active_lock_fd_ = -1;
  }
  if (finalized) {
    owns_run_ = false;
  }
}

void RunLogManager::complete_owned_run() noexcept {
  finalize_owner_manifest();
}

void RunLogManager::write_run_info(const std::map<std::string, std::string>& extra_fields) const {
  nlohmann::json json;
  for (const auto& field : collect_run_info_fields()) {
    json[field.first] = field.second;
  }

  if (const auto* config = GlobalConfig::get_if_initialized()) {
    nlohmann::json config_paths = nlohmann::json::object();
    for (const auto& item : config->list_config_paths()) {
      config_paths[item.first] = item.second;
    }
    json["selected_config_paths"] = config_paths;
  }

  for (const auto& field : extra_fields) {
    json[field.first] = field.second;
  }

  std::error_code ec;
  std::filesystem::create_directories(metadata_path("processes"), ec);
  try {
    atomic_write_json(
        metadata_path("processes") / (safe_filename(process_name_) + ".json"), json);
  } catch (const std::exception& error) {
    spdlog::warn("[RunLogManager] failed to write process metadata: {}", error.what());
  }
}

}  // namespace liuo_tunnel
