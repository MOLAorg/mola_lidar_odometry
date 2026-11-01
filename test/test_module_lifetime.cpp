/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/
/**
 * @file   test_module_lifetime.cpp
 * @brief  LidarOdometry must not keep alive the modules it uses, nor be kept
 *         alive by them, once the system releases its references
 * @author Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mola_kernel/MinimalModuleContainer.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/TransformTreeSource.h>
#include <mola_lidar_odometry/LidarOdometry.h>
#include <mola_state_estimation_simple/StateEstimationSimple.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/get_env.h>

#include <memory>

namespace mola
{
/** Like a ROS bridge: exposes a /tf tree and subscribes to every localization
 *  source, holding a reference to each of them. */
class MockTfSourceSubscriber : public ExecutableBase, public TransformTreeSource
{
  DEFINE_MRPT_OBJECT(MockTfSourceSubscriber, mola)

public:
  MockTfSourceSubscriber() = default;
  ~MockTfSourceSubscriber() override = default;

  void initialize(const mrpt::containers::yaml &) override
  {
    for (const auto & m : findService<LocalizationSourceBase>()) {
      locSources_.push_back(std::dynamic_pointer_cast<LocalizationSourceBase>(m));
    }
  }
  void spinOnce() override {}

  std::optional<TransformTree> transform_tree(
    const std::string &, const std::optional<mrpt::Clock::time_point> &) const override
  {
    return std::nullopt;
  }

  size_t numLocSources() const { return locSources_.size(); }

private:
  std::vector<std::shared_ptr<LocalizationSourceBase>> locSources_;
};
}  // namespace mola
IMPLEMENTS_MRPT_OBJECT(MockTfSourceSubscriber, mola::ExecutableBase, mola)

// A module that is both a /tf source for LidarOdometry and a subscriber of it
// (as BridgeROS2 is) must not form a reference cycle with it: otherwise both
// outlive the system, and the bridge keeps calling into a destroyed launcher.
TEST(ModuleLifetime, NoReferenceCycleWithTfSourceSubscriber)
{
  std::weak_ptr<mola::LidarOdometry> weakLiodom;
  std::weak_ptr<mola::MockTfSourceSubscriber> weakTfSource;
  {
    auto liodom = mola::LidarOdometry::Create();
    auto stateEstimator = mola::state_estimation_simple::StateEstimationSimple::Create();
    auto tfSource = std::make_shared<mola::MockTfSourceSubscriber>();

    mola::MinimalModuleContainer moduleContainer = {{liodom, stateEstimator, tfSource}};

    liodom->setMinLoggingLevel(mrpt::system::LVL_WARN);
    liodom->initialize(mola::load_yaml_file(mrpt::get_env<std::string>("LO_PIPELINE_YAML")));

    stateEstimator->setMinLoggingLevel(mrpt::system::LVL_WARN);
    stateEstimator->initialize(
      mola::load_yaml_file(mrpt::get_env<std::string>("LO_STATE_ESTIM_YAML")));

    tfSource->initialize({});
    ASSERT_EQ(tfSource->numLocSources(), 1U);

    weakLiodom = liodom;
    weakTfSource = tfSource;
  }
  EXPECT_TRUE(weakLiodom.expired());
  EXPECT_TRUE(weakTfSource.expired());
}
