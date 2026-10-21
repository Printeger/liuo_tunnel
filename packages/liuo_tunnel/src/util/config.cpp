// Adapted from IAP src/iap/util/config.cpp @ fbccae168; namespace and independent project paths. See THIRD_PARTY_NOTICES.md.
#include <liuo_tunnel/util/config.hpp>

#include <boost/filesystem.hpp>
#include <filesystem>
#include <liuo_tunnel/util/config_impl.hpp>

namespace liuo_tunnel {

GlobalConfig* GlobalConfig::inst = nullptr;

Config::Config(const std::string& config_filename) {
  nlohmann::json json;
  if (config_filename.empty()) {
    config = json;
    return;
  }

  std::ifstream ifs(config_filename);
  if (!ifs) {
    throw std::runtime_error("missing configuration: " + config_filename);
  } else {
    json = nlohmann::json::parse(ifs, nullptr, true, true);
  }

  config = json;
}

Config::~Config() {}

bool Config::has_param(const std::string& module_name, const std::string& param_name) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);
  auto module = json.find(module_name);
  if (module == json.end()) {
    return false;
  }

  return module->find(param_name) != module->end();
}

void Config::save(const std::string& path) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  std::ofstream ofs(path);
  ofs << std::setw(2) << json << std::endl;
}

DEFINE_CONFIG_IO_SPECIALIZATION(bool)
DEFINE_CONFIG_IO_SPECIALIZATION(int)
DEFINE_CONFIG_IO_SPECIALIZATION(size_t)
DEFINE_CONFIG_IO_SPECIALIZATION(float)
DEFINE_CONFIG_IO_SPECIALIZATION(double)
DEFINE_CONFIG_IO_SPECIALIZATION(std::string)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<bool>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<int>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<double>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<std::string>)

DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector2d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector3d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector4d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Quaterniond)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Isometry3d)

DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<Eigen::Isometry3d>)

GlobalConfig* GlobalConfig::instance(const std::string& config_path, bool override_path) {
  if (inst == nullptr || override_path) {
    if (inst) {
      delete inst;
    }

    inst = new GlobalConfig(config_path + "/config.json");
    inst->override_param("global", "config_path", config_path);
  }
  return inst;
}

GlobalConfig* GlobalConfig::get_if_initialized() {
  return inst;
}

std::string GlobalConfig::get_config_path(const std::string& config_name) {
  auto config = instance();
  const std::string directory = config->param<std::string>("global", "config_path", ".");

  if (config->has_param("global", config_name)) {
    const auto filename = config->param<std::string>("global", config_name);
    if (filename) {
      return directory + "/" + *filename;
    }
    spdlog::warn("global/{} exists but is not a string; fallback to default filename", config_name);
  }

  if (config_name == "config_preprocess") {
    const auto sensors_file = config->param<std::string>("global", "config_sensors");
    if (sensors_file) {
      spdlog::warn("global/config_preprocess not found; fallback to global/config_sensors ({})", *sensors_file);
      return directory + "/" + *sensors_file;
    }
  }

  const std::string filename = config_name + ".json";
  return directory + "/" + filename;
}

std::vector<std::pair<std::string, std::string>> GlobalConfig::list_config_paths() const {
  std::vector<std::pair<std::string, std::string>> paths;

  const auto& json = std::any_cast<const nlohmann::json&>(config);
  auto global_itr = json.find("global");
  if (global_itr == json.end() || !global_itr->is_object()) {
    return paths;
  }

  for (const auto& param : global_itr->items()) {
    const std::string config_name = param.key();
    if (config_name.rfind("config_", 0) != 0 || config_name == "config_path" || config_name == "config_ext") {
      continue;
    }

    if (!param.value().is_string()) {
      spdlog::warn("skip listing global/{} because value is not string", config_name);
      continue;
    }

    paths.emplace_back(config_name, get_config_path(config_name));
  }

  return paths;
}

void GlobalConfig::dump(const std::string& path) {
  spdlog::debug("dumping config to {} (config_path={})", path, param<std::string>("global", "config_path", "."));
  boost::filesystem::create_directories(path);
  this->save(path + "/config.json");

  for (const auto& config_path : list_config_paths()) {
    const auto& config_name = config_path.first;
    const auto& resolved_path = config_path.second;
    const auto config_file = std::filesystem::path(resolved_path).filename().string();

    spdlog::debug("dumping {} : {}", config_name, config_file);
    const Config conf(resolved_path);
    conf.save(path + "/" + config_file);
  }

  spdlog::debug("dumping global config done");
}

}  // namespace liuo_tunnel
