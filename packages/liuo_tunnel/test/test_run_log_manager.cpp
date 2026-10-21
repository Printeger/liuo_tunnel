#include <gtest/gtest.h>

#include <filesystem>
#include <cstdlib>

#include <unistd.h>

#include <liuo_tunnel/util/run_log_manager.hpp>

namespace {
struct ScopedDirectory {
  explicit ScopedDirectory(std::filesystem::path value) : path(std::move(value)) {}
  ~ScopedDirectory() { std::filesystem::remove_all(path); }
  std::filesystem::path path;
};
}  // namespace

TEST(RunLogManagerTest, AdoptsOwnerRunWithoutAllocatingNestedTimestamp) {
  const auto unique = std::to_string(::getpid());
  const std::filesystem::path temp_root = std::filesystem::temp_directory_path() / ("liuo_run_log_manager_test_" + unique);
  const ScopedDirectory cleanup(temp_root);
  const std::filesystem::path config_dir = temp_root / "config";
  const std::filesystem::path run_dir = temp_root / "20260929T120000Z_001";

  std::filesystem::create_directories(config_dir);
  std::filesystem::create_directories(run_dir);
  ASSERT_EQ(::setenv("LIUO_RUN_DIR", run_dir.c_str(), 1), 0);

  auto& first = liuo_tunnel::RunLogManager::initialize("test_run_log_manager", config_dir.string());
  auto& second = liuo_tunnel::RunLogManager::initialize("ignored_second_call", config_dir.string());

  EXPECT_EQ(&first, &second);
  EXPECT_EQ(first.run_dir(), std::filesystem::canonical(run_dir));
  EXPECT_EQ(first.log_root(), std::filesystem::canonical(temp_root));
  EXPECT_TRUE(std::filesystem::exists(first.runtime_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.profiling_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.export_path("")));
  EXPECT_TRUE(std::filesystem::exists(first.metadata_path("")));
  EXPECT_EQ(first.runtime_path("liuo_main.log").parent_path(), first.run_dir() / "runtime");
  EXPECT_EQ(first.profiling_path("liuo_timing.csv").parent_path(), first.run_dir() / "profiling");
  EXPECT_EQ(first.export_path("liuo_icp.csv").parent_path(), first.run_dir() / "export");
  first.write_run_info({{"status", "running"}});
  EXPECT_TRUE(std::filesystem::is_regular_file(first.metadata_path("processes/test_run_log_manager.json")));
  EXPECT_FALSE(std::filesystem::exists(first.metadata_path("run_info.json")));
  EXPECT_FALSE(std::filesystem::exists(run_dir / "20260929T120000Z_001"));
  EXPECT_FALSE(std::filesystem::exists(temp_root / "latest"));
  EXPECT_THROW(first.export_path("../escape.csv"), std::invalid_argument);
  EXPECT_THROW(first.runtime_path("/tmp/escape.log"), std::invalid_argument);
  const auto outside = temp_root / "outside";
  std::filesystem::create_directories(outside);
  std::filesystem::create_directory_symlink(outside, first.export_path("") / "linked");
  EXPECT_THROW(first.export_path("linked/escape.csv"), std::invalid_argument);
  std::filesystem::rename(run_dir / "export", run_dir / "original_export");
  std::filesystem::create_directory_symlink(outside, run_dir / "export");
  EXPECT_THROW(first.export_path("escape.csv"), std::invalid_argument);
  EXPECT_THROW(first.export_path(""), std::invalid_argument);
}
