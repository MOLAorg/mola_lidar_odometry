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
 * @file   mola-sm-attach-observations.cpp
 * @brief  Adds observations from a rosbag2 (e.g. camera images) to the
 *         keyframes of an existing simplemap, the closest in time to each.
 * @author Jose Luis Blanco Claraco
 * @date   2026
 */

#include <mola_input_rosbag2/Rosbag2Dataset.h>
#include <mola_lidar_odometry/KeyframeAttachments.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace
{
struct Keyframe
{
  double stamp = 0;
  mrpt::obs::CSensoryFrame::Ptr sf;
};

/// Keyframe time: that of its "metadata" comment, else of its first observation.
std::optional<double> keyframe_stamp(const mrpt::obs::CSensoryFrame & sf)
{
  for (const auto & o : sf) {
    if (std::dynamic_pointer_cast<const mrpt::obs::CObservationComment>(o)) {
      return mrpt::Clock::toDouble(o->timestamp);
    }
  }
  if (sf.empty()) {
    return {};
  }
  return mrpt::Clock::toDouble((*sf.begin())->timestamp);
}

/// Whether the keyframe holds sensor data (not only the metadata comment of a
/// non-keyframe frame).
bool has_sensor_data(const mrpt::obs::CSensoryFrame & sf)
{
  return std::any_of(sf.begin(), sf.end(), [](const auto & o) {
    return !std::dynamic_pointer_cast<const mrpt::obs::CObservationComment>(o);
  });
}

bool has_label(const mrpt::obs::CSensoryFrame & sf, const std::string & label)
{
  return std::any_of(sf.begin(), sf.end(), [&](const auto & o) { return o->sensorLabel == label; });
}

/// Hard links (or copies) the files of `from` into `to`, so the output
/// simplemap finds the external files of the input one.
void mirror_lazy_load_dir(const std::string & from, const std::string & to)
{
  namespace fs = std::filesystem;
  fs::create_directories(to);
  size_t n = 0;
  for (const auto & e : fs::directory_iterator(from)) {
    if (!e.is_regular_file()) {
      continue;
    }
    const auto dst = fs::path(to) / e.path().filename();
    if (fs::exists(dst)) {
      continue;
    }
    std::error_code ec;
    fs::create_hard_link(e.path(), dst, ec);
    if (ec) {
      fs::copy_file(e.path(), dst);
    }
    n++;
  }
  std::cout << "Linked " << n << " external files of the input map into: " << to << "\n";
}

std::string lazy_load_dir_of(const std::string & simplemapFile)
{
  return mrpt::system::fileNameChangeExtension(simplemapFile, "") + "_Images";
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    CLI::App app(
      "mola-sm-attach-observations: adds observations from a rosbag2 (e.g. camera images) to "
      "the keyframes of a simplemap. Each observation goes to the keyframe closest in time "
      "(within --max-age), and each keyframe keeps the closest of them, so no observation is "
      "stored twice. Images are written to <output>_Images/.");

    std::string inputFile;
    std::string outputFile;
    std::string rosbag;
    std::vector<std::string> topics;
    double maxAge = 0.5;
    std::string imageFormat = "jpg";
    int jpegQuality = 95;
    std::string baseLink = "base_link";
    std::string tfTopic = "/tf";
    std::string tfStaticTopic = "/tf_static";

    app.add_option("-i,--input", inputFile, "Input .simplemap")->required();
    app.add_option("-o,--output", outputFile, "Output .simplemap (may be the input one)")
      ->required();
    app.add_option("--rosbag2", rosbag, "Input rosbag2 (directory or .mcap file)")->required();
    app
      .add_option(
        "--topic", topics,
        "Topic to read, as TOPIC or TOPIC:LABEL (default label: the topic name). Repeatable.")
      ->required();
    app.add_option("--max-age", maxAge, "Max time offset to the keyframe [s]")
      ->capture_default_str();
    app.add_option("--image-format", imageFormat, "jpg, png or inline")->capture_default_str();
    app.add_option("--jpeg-quality", jpegQuality, "JPEG quality [1-100]")->capture_default_str();
    app.add_option("--base-link-frame-id", baseLink, "Vehicle frame, for the sensor poses")
      ->capture_default_str()
      ->envname("MOLA_TF_BASE_LINK");
    app.add_option("--tf-topic", tfTopic)->capture_default_str()->envname("MOLA_TF_TOPIC");
    app.add_option("--tf-static-topic", tfStaticTopic)
      ->capture_default_str()
      ->envname("MOLA_TF_STATIC_TOPIC");

    CLI11_PARSE(app, argc, argv);

    if (!mola::is_valid_keyframe_image_format(imageFormat)) {
      throw std::invalid_argument("--image-format must be one of: jpg, png, inline");
    }

    // Topics and their sensor labels:
    std::map<std::string, std::string> label2topic;
    std::string sensorsYaml;
    for (const auto & t : topics) {
      const auto sep = t.find(':');
      const std::string topic = t.substr(0, sep);
      const std::string label = sep == std::string::npos ? topic : t.substr(sep + 1);
      label2topic[label] = topic;
      sensorsYaml += mrpt::format(
        "\n        - topic: '%s'\n          sensorLabel: '%s'", topic.c_str(), label.c_str());
    }

    // Input map:
    mrpt::maps::CSimpleMap sm;
    if (!sm.loadFromFile(inputFile)) {
      throw std::runtime_error("Cannot read: " + inputFile);
    }
    std::vector<Keyframe> kfs;
    for (const auto & kf : sm) {
      if (!kf.sf || !has_sensor_data(*kf.sf)) {
        continue;
      }
      if (const auto t = keyframe_stamp(*kf.sf); t) {
        kfs.push_back({*t, kf.sf});
      }
    }
    // Observations already in the map (e.g. from the live run), never added again:
    std::set<std::pair<std::string, mrpt::Clock::time_point>> existing;
    for (const auto & kf : kfs) {
      for (const auto & o : *kf.sf) {
        if (label2topic.count(o->sensorLabel) != 0) {
          existing.emplace(o->sensorLabel, o->timestamp);
        }
      }
    }
    std::sort(
      kfs.begin(), kfs.end(), [](const auto & a, const auto & b) { return a.stamp < b.stamp; });
    if (kfs.empty()) {
      throw std::runtime_error("The input simplemap has no keyframes with sensor data");
    }
    std::cout << "Input: " << sm.size() << " frames, " << kfs.size()
              << " keyframes, from t=" << mrpt::format("%.03f", kfs.front().stamp) << " to "
              << mrpt::format("%.03f", kfs.back().stamp) << "\n";

    // Output directory for external files, with those of the input map:
    const auto outDir = lazy_load_dir_of(outputFile);
    const auto inDir = lazy_load_dir_of(inputFile);
    if (
      mrpt::system::directoryExists(inDir) &&
      std::filesystem::weakly_canonical(inDir) != std::filesystem::weakly_canonical(outDir)) {
      mirror_lazy_load_dir(inDir, outDir);
    }
    if (imageFormat != "inline") {
      std::filesystem::create_directories(outDir);
    }

    // Per label and keyframe, the closest observation so far and its offset:
    struct Candidate
    {
      mrpt::obs::CObservation::Ptr obs;
      double dt = 0;
    };
    std::map<std::string, std::map<size_t, Candidate>> best;
    std::map<std::string, size_t> numAttached;
    size_t numSkippedExisting = 0;
    size_t numObs = 0;

    // Stores a final candidate in its keyframe (images: as an external file).
    const auto commit = [&](const std::string & label, size_t kfIdx, const Candidate & c) {
      auto & sf = *kfs.at(kfIdx).sf;
      if (has_label(sf, label)) {
        numSkippedExisting++;
        return;
      }
      auto obs = c.obs;
      if (auto img = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(obs);
          img && imageFormat != "inline" && !img->image.isExternallyStored() &&
          img->image.getPixelDepth() == mrpt::img::PixelDepth::D8U) {
        const auto name = mola::external_observation_file_name(*img, imageFormat);
        const auto path = mrpt::system::pathJoin({outDir, name});
        // Never write through a hard link to a file of the input map:
        std::filesystem::remove(path);
        if (!img->image.saveToFile(path, jpegQuality)) {
          throw std::runtime_error("Cannot write: " + path);
        }
        obs = mola::external_image_copy(*img, name);
      }
      sf.insert(obs);
      numAttached[label]++;
    };

    // Commits the candidates of keyframes that can no longer get a closer one.
    const auto commitUpTo = [&](double t) {
      for (auto & [label, m] : best) {
        for (auto it = m.begin(); it != m.end();) {
          if (kfs.at(it->first).stamp + maxAge < t) {
            commit(label, it->first, it->second);
            it = m.erase(it);
          } else {
            ++it;
          }
        }
      }
    };

    // The rosbag:
    auto ds = std::make_shared<mola::Rosbag2Dataset>();
    ds->setMinLoggingLevel(mrpt::system::LVL_WARN);
    ds->initialize(mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
      R""""(
    params:
      rosbag_filename: '%s'
      base_link_frame_id: '%s'
      tf_topic: '%s'
      tf_static_topic: '%s'
      sensors:%s
)"""",
      rosbag.c_str(), baseLink.c_str(), tfTopic.c_str(), tfStaticTopic.c_str(),
      sensorsYaml.c_str()))));

    const double tFirst = kfs.front().stamp - maxAge;
    const double tLast = kfs.back().stamp + maxAge;
    const size_t N = ds->datasetSize();
    for (size_t i = 0; i < N; i++) {
      const auto sf = ds->datasetGetObservations(i);
      if (!sf) {
        continue;
      }
      for (const auto & o : *sf) {
        if (!o || label2topic.count(o->sensorLabel) == 0) {
          continue;
        }
        numObs++;
        if (existing.count({o->sensorLabel, o->timestamp}) != 0) {
          continue;
        }
        const double t = mrpt::Clock::toDouble(o->timestamp);
        if (t < tFirst || t > tLast) {
          continue;
        }
        // Nearest keyframe in time:
        const auto it = std::lower_bound(
          kfs.begin(), kfs.end(), t, [](const Keyframe & k, double v) { return k.stamp < v; });
        size_t k = static_cast<size_t>(std::distance(kfs.begin(), it));
        if (k == kfs.size() || (k > 0 && t - kfs[k - 1].stamp < kfs[k].stamp - t)) {
          k--;
        }
        const double dt = std::abs(t - kfs[k].stamp);
        if (dt > maxAge) {
          continue;
        }
        auto & slot = best[o->sensorLabel];
        if (const auto c = slot.find(k); c == slot.end() || dt < c->second.dt) {
          slot[k] = {std::const_pointer_cast<mrpt::obs::CObservation>(o), dt};
        }
        // With a margin: the bag is in receive order, not in stamp order.
        constexpr double STAMP_ORDER_MARGIN = 2.0;  // [s]
        commitUpTo(t - STAMP_ORDER_MARGIN);
      }
      if (i % 1000 == 0) {
        std::cout << mrpt::format(
                       "\rReading rosbag: %5.01f%%",
                       100.0 * static_cast<double>(i) / static_cast<double>(N))
                  << std::flush;
      }
    }
    commitUpTo(std::numeric_limits<double>::max());
    std::cout << "\rReading rosbag: done.  \n";

    for (const auto & [label, topic] : label2topic) {
      std::cout << "'" << label << "' (" << topic << "): " << numAttached[label] << " keyframes of "
                << kfs.size() << "\n";
    }
    std::cout << "Observations read: " << numObs << "\n";
    if (numSkippedExisting > 0) {
      std::cout << "Keyframes skipped (already had an observation with that label): "
                << numSkippedExisting << "\n";
    }

    if (!sm.saveToFile(outputFile)) {
      throw std::runtime_error("Cannot write: " + outputFile);
    }
    std::cout << "Saved: " << outputFile << "\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
