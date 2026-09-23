/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/
/**
 * @file   test_pipelines_load.cpp
 * @brief  Every shipped pipeline must resolve (blocks included) to a complete
 *         configuration, and a pipeline block must be replaceable via its
 *         environment variable.
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{
namespace fs = std::filesystem;

fs::path pipelinesDir()
{
  const auto dir = mrpt::get_env<std::string>("LO_PIPELINES_DIR");
  EXPECT_FALSE(dir.empty()) << "LO_PIPELINES_DIR is not set";
  return dir;
}

std::vector<fs::path> shippedPipelines()
{
  std::vector<fs::path> files;
  for (const auto & sub : {fs::path(""), fs::path("extras")}) {
    for (const auto & e : fs::directory_iterator(pipelinesDir() / sub)) {
      if (e.path().extension() == ".yaml") {
        files.push_back(e.path());
      }
    }
  }
  return files;
}

void setEnv(const std::string & name, const std::string & value)
{
#ifdef _WIN32
  _putenv_s(name.c_str(), value.c_str());
#else
  ::setenv(name.c_str(), value.c_str(), 1);  // NOLINT(concurrency-mt-unsafe)
#endif
}

void unsetEnv(const std::string & name)
{
#ifdef _WIN32
  _putenv_s(name.c_str(), "");
#else
  ::unsetenv(name.c_str());                  // NOLINT(concurrency-mt-unsafe)
#endif
}
}  // namespace

TEST(Pipelines, AllResolveToCompleteConfig)
{
  const auto files = shippedPipelines();
  ASSERT_GE(files.size(), 5U);

  for (const auto & f : files) {
    SCOPED_TRACE(f.string());
    const auto cfg = mola::load_yaml_file(f.string());

    ASSERT_TRUE(cfg.has("params"));
    ASSERT_TRUE(cfg.has("icp_settings_with_vel"));
    ASSERT_TRUE(cfg.has("localmap_generator"));
    ASSERT_TRUE(cfg.has("observations_generator"));
    ASSERT_TRUE(cfg.has("observations_filter_1st_pass"));
    ASSERT_TRUE(cfg.has("insert_observation_into_local_map"));
    ASSERT_TRUE(cfg["params"].has("min_icp_goodness"));
    ASSERT_TRUE(cfg["params"].has("local_map_updates"));
  }
}

TEST(Pipelines, BlockIsReplaceableFromEnvironment)
{
  // A user block that imports the stock one and changes a single entry:
  const fs::path userBlock = fs::temp_directory_path() / "mola_lo_test_user_block.yaml";
  {
    std::ofstream o(userBlock);
    o << "$import: " << (pipelinesDir() / "blocks" / "adaptive-threshold.yaml").string() << "\n"
      << "params:\n"
      << "  adaptive_threshold:\n"
      << "    kp: 123.0\n";
  }

  const auto file = (pipelinesDir() / "lidar3d-gicp.yaml").string();
  const auto stock = mola::load_yaml_file(file);

  setEnv("MOLA_LO_BLOCK_ADAPTIVE_THRESHOLD", userBlock.string());
  const auto custom = mola::load_yaml_file(file);
  unsetEnv("MOLA_LO_BLOCK_ADAPTIVE_THRESHOLD");
  fs::remove(userBlock);

  const auto & at = custom["params"]["adaptive_threshold"];
  EXPECT_DOUBLE_EQ(at["kp"].as<double>(), 123.0);
  // The rest of the stock block is kept:
  EXPECT_DOUBLE_EQ(
    at["initial_sigma"].as<double>(),
    stock["params"]["adaptive_threshold"]["initial_sigma"].as<double>());
  EXPECT_NE(stock["params"]["adaptive_threshold"]["kp"].as<double>(), 123.0);
}
