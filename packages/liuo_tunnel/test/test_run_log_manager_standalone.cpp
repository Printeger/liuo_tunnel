#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <unistd.h>

#include <nlohmann/json.hpp>

#include <liuo_tunnel/util/run_log_manager.hpp>

namespace {
struct ScopedDirectory {
  explicit ScopedDirectory(std::filesystem::path value) : path(std::move(value)) {}
  ~ScopedDirectory() { std::filesystem::remove_all(path); }
  std::filesystem::path path;
};

void write_finalized_run(const std::filesystem::path& root, const std::string& name) {
  const auto run = root / name;
  std::filesystem::create_directories(run / "metadata");
  const nlohmann::json manifest = {
    {"schema_version", "liuo_run_artifact_v1"},
    {"run_id", name},
    {"run_root", root.string()},
    {"run_dir", run.string()},
    {"ended_at_utc", "2020-01-01T00:00:00Z"},
    {"lifecycle", "completed"},
    {"run_class", "development"},
    {"retention_class", "ordinary"},
  };
  std::ofstream(run / "metadata" / "run_manifest.json") << manifest;
}
}  // namespace

TEST(RunLogManagerStandaloneTest, AllocatesDirectlyBelowConfiguredRoot) {
  const auto unique = std::to_string(::getpid());
  const std::filesystem::path root = std::filesystem::temp_directory_path() / ("liuo_run_log_standalone_" + unique);
  const ScopedDirectory cleanup(root);
  ASSERT_EQ(::unsetenv("LIUO_RUN_DIR"), 0);
  ASSERT_EQ(::setenv("LIUO_RUN_ROOT", root.c_str(), 1), 0);
  ASSERT_EQ(::setenv("LIUO_RETENTION_ENABLED", "1", 1), 0);
  std::filesystem::create_directories(root);
  for (int day = 1; day <= 4; ++day) {
    write_finalized_run(root, "2020010" + std::to_string(day) + "T000000Z_000");
  }

  auto& logs = liuo_tunnel::RunLogManager::initialize("standalone");

  EXPECT_EQ(logs.log_root(), std::filesystem::canonical(root));
  EXPECT_EQ(logs.run_dir().parent_path(), std::filesystem::canonical(root));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "runtime"));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "profiling"));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "export"));
  EXPECT_TRUE(std::filesystem::exists(logs.run_dir() / "metadata"));
  EXPECT_TRUE(std::filesystem::is_regular_file(logs.run_dir() / "metadata" / "run_manifest.json"));
  EXPECT_FALSE(std::filesystem::exists(logs.run_dir() / "metadata" / "run_info.json"));
  ASSERT_TRUE(std::filesystem::is_symlink(root / "latest"));
  EXPECT_EQ(std::filesystem::canonical(root / "latest"), logs.run_dir());
  EXPECT_FALSE(std::filesystem::exists(root / "20200101T000000Z_000"));
  EXPECT_TRUE(std::filesystem::exists(root / "20200102T000000Z_000"));
  EXPECT_TRUE(std::filesystem::exists(root / "20200103T000000Z_000"));
  EXPECT_TRUE(std::filesystem::exists(root / "20200104T000000Z_000"));
  logs.complete_owned_run();
  nlohmann::json manifest;
  std::ifstream(logs.run_dir() / "metadata" / "run_manifest.json") >> manifest;
  EXPECT_EQ(manifest["lifecycle"], "completed");
  EXPECT_TRUE(manifest["ended_at_utc"].is_string());
}
