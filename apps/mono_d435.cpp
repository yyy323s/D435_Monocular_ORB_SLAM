// SPDX-License-Identifier: GPL-3.0-or-later
// Live D435 (without IMU) entry point for the ORB-SLAM3 core in this project.

#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <librealsense2/rs.hpp>

#include "System.h"

namespace {

volatile std::sig_atomic_t gStopRequested = 0;

void handleSignal(int) {
    gStopRequested = 1;
}

struct CommandLine {
    std::string vocabularyPath = DEFAULT_VOCABULARY_PATH;
    std::string settingsPath = DEFAULT_SETTINGS_PATH;
    std::string runtimeDirectory = DEFAULT_RUNTIME_DIRECTORY;
    std::string serial;
    std::string keyframeTrajectory = "output/KeyFrameTrajectory.txt";
    std::string pointCloud = "output/MapPoints.ply";
    int maxFrames = 0;
    int warmupFramesOverride = -1;
    bool useViewer = true;
    bool saveTrajectory = true;
    bool savePointCloud = true;
    bool help = false;
};

struct CaptureSettings {
    int width = 640;
    int height = 480;
    int fps = 30;
    int infraredIndex = 1;
    int warmupFrames = 30;
    int timeoutMs = 250;
    int autoExposureLimitUs = 5000;
    bool emitterEnabled = false;
    bool autoExposure = true;
};

struct ImagePacket {
    cv::Mat image;
    double timestampSeconds = 0.0;
    std::uint64_t frameNumber = 0;
};

void printUsage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [options]\n\n"
        << "  --vocabulary FILE       ORB vocabulary (default: build vocabulary)\n"
        << "  --settings FILE         ORB-SLAM3/D435 YAML template\n"
        << "  --serial SERIAL         Select one D435 when several are connected\n"
        << "  --runtime-dir DIR       Where to write the effective calibrated YAML\n"
        << "  --trajectory FILE       Keyframe trajectory output path\n"
        << "  --pointcloud FILE       Sparse MapPoint PLY output path\n"
        << "  --no-save               Do not save a keyframe trajectory\n"
        << "  --no-pointcloud         Do not save the sparse point cloud\n"
        << "  --headless              Disable Pangolin/OpenCV viewer\n"
        << "  --warmup-frames N       Override Camera.warmupFrames\n"
        << "  --max-frames N          Stop after N tracked frames\n"
        << "  -h, --help              Show this help\n";
}

int parsePositiveInt(const std::string& text, const char* option) {
    const int value = std::stoi(text);
    if (value <= 0) {
        throw std::invalid_argument(std::string(option) + " must be positive");
    }
    return value;
}

CommandLine parseCommandLine(int argc, char** argv) {
    CommandLine options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        const auto requireValue = [&](const char* option) -> std::string {
            if (++index >= argc) {
                throw std::invalid_argument(std::string(option) + " requires a value");
            }
            return argv[index];
        };

        if (argument == "--help" || argument == "-h") {
            options.help = true;
        } else if (argument == "--vocabulary") {
            options.vocabularyPath = requireValue("--vocabulary");
        } else if (argument == "--settings") {
            options.settingsPath = requireValue("--settings");
        } else if (argument == "--serial") {
            options.serial = requireValue("--serial");
        } else if (argument == "--runtime-dir") {
            options.runtimeDirectory = requireValue("--runtime-dir");
        } else if (argument == "--trajectory") {
            options.keyframeTrajectory = requireValue("--trajectory");
        } else if (argument == "--pointcloud") {
            options.pointCloud = requireValue("--pointcloud");
        } else if (argument == "--no-save") {
            options.saveTrajectory = false;
        } else if (argument == "--no-pointcloud") {
            options.savePointCloud = false;
        } else if (argument == "--headless") {
            options.useViewer = false;
        } else if (argument == "--warmup-frames") {
            options.warmupFramesOverride =
                parsePositiveInt(requireValue("--warmup-frames"), "--warmup-frames");
        } else if (argument == "--max-frames") {
            options.maxFrames =
                parsePositiveInt(requireValue("--max-frames"), "--max-frames");
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    return options;
}

template <typename T>
void readIfPresent(const cv::FileStorage& storage, const char* key, T& value) {
    const cv::FileNode node = storage[key];
    if (!node.empty()) {
        node >> value;
    }
}

CaptureSettings readCaptureSettings(const std::string& settingsPath) {
    cv::FileStorage storage(settingsPath, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        throw std::runtime_error("cannot open settings template: " + settingsPath);
    }

    CaptureSettings settings;
    readIfPresent(storage, "Camera.width", settings.width);
    readIfPresent(storage, "Camera.height", settings.height);
    readIfPresent(storage, "Camera.fps", settings.fps);
    readIfPresent(storage, "Camera.infraredIndex", settings.infraredIndex);
    readIfPresent(storage, "Camera.warmupFrames", settings.warmupFrames);
    readIfPresent(storage, "Camera.frameTimeoutMs", settings.timeoutMs);
    readIfPresent(storage, "Camera.autoExposureLimitUs", settings.autoExposureLimitUs);

    int emitter = settings.emitterEnabled ? 1 : 0;
    int autoExposure = settings.autoExposure ? 1 : 0;
    readIfPresent(storage, "Camera.emitterEnabled", emitter);
    readIfPresent(storage, "Camera.autoExposure", autoExposure);
    settings.emitterEnabled = emitter != 0;
    settings.autoExposure = autoExposure != 0;

    if (settings.width <= 0 || settings.height <= 0 || settings.fps <= 0 ||
        settings.timeoutMs <= 0 || settings.warmupFrames < 0 ||
        settings.autoExposureLimitUs <= 0 ||
        (settings.infraredIndex != 1 && settings.infraredIndex != 2)) {
        throw std::runtime_error("invalid D435 capture values in settings template");
    }
    return settings;
}

std::string deviceInfo(const rs2::device& device, rs2_camera_info field) {
    return device.supports(field) ? device.get_info(field) : "unknown";
}

rs2::device selectDevice(const std::string& requestedSerial) {
    rs2::context context;
    const rs2::device_list devices = context.query_devices();
    std::vector<rs2::device> candidates;
    for (rs2::device device : devices) {
        const std::string name = deviceInfo(device, RS2_CAMERA_INFO_NAME);
        const std::string serial = deviceInfo(device, RS2_CAMERA_INFO_SERIAL_NUMBER);
        if ((!requestedSerial.empty() && serial == requestedSerial) ||
            (requestedSerial.empty() && name.find("D435") != std::string::npos)) {
            candidates.push_back(device);
        }
    }

    if (candidates.empty()) {
        throw std::runtime_error(requestedSerial.empty()
                                     ? "no Intel RealSense D435 device is connected"
                                     : "requested RealSense serial was not found");
    }
    if (candidates.size() != 1) {
        throw std::runtime_error("multiple D435 devices found; pass --serial explicitly");
    }
    return candidates.front();
}

void setOptionIfWritable(const rs2::sensor& sensor, rs2_option option, float value) {
    if (!sensor.supports(option) || sensor.is_option_read_only(option)) {
        return;
    }
    try {
        sensor.set_option(option, value);
    } catch (const rs2::error& error) {
        std::cerr << "warning: could not set RealSense option " << option
                  << ": " << error.what() << '\n';
    }
}

void configureSensors(const rs2::device& device, const CaptureSettings& settings) {
    for (const rs2::sensor& sensor : device.query_sensors()) {
        setOptionIfWritable(sensor,
                            RS2_OPTION_ENABLE_AUTO_EXPOSURE,
                            settings.autoExposure ? 1.0F : 0.0F);
        setOptionIfWritable(sensor,
                            RS2_OPTION_EMITTER_ENABLED,
                            settings.emitterEnabled ? 1.0F : 0.0F);
        if (settings.autoExposure) {
            setOptionIfWritable(sensor, RS2_OPTION_AUTO_EXPOSURE_LIMIT_TOGGLE, 1.0F);
            setOptionIfWritable(sensor,
                                RS2_OPTION_AUTO_EXPOSURE_LIMIT,
                                static_cast<float>(settings.autoExposureLimitUs));
        }
    }
}

std::optional<ImagePacket> tryGetInfraredFrame(rs2::pipeline& pipeline,
                                                int infraredIndex,
                                                int timeoutMs) {
    rs2::frameset frames;
    if (!pipeline.try_wait_for_frames(&frames, static_cast<unsigned int>(timeoutMs))) {
        return std::nullopt;
    }

    const rs2::video_frame infrared = frames.get_infrared_frame(infraredIndex);
    if (!infrared) {
        throw std::runtime_error("frameset did not contain the requested infrared stream");
    }
    if (infrared.get_profile().format() != RS2_FORMAT_Y8) {
        throw std::runtime_error("D435 infrared stream is not Y8");
    }

    const cv::Mat view(cv::Size(infrared.get_width(), infrared.get_height()),
                       CV_8UC1,
                       const_cast<void*>(infrared.get_data()),
                       static_cast<std::size_t>(infrared.get_stride_in_bytes()));
    ImagePacket packet;
    packet.image = view.clone();
    packet.timestampSeconds = infrared.get_timestamp() * 1e-3;
    packet.frameNumber = infrared.get_frame_number();
    return packet;
}

std::string trim(std::string value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string formatReal(double value) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(12) << value;
    return stream.str();
}

std::string writeEffectiveSettings(const std::string& templatePath,
                                   const std::string& runtimeDirectory,
                                   const std::string& serial,
                                   const rs2_intrinsics& intrinsics,
                                   const CaptureSettings& capture) {
    bool nonZeroDistortion = false;
    for (float coefficient : intrinsics.coeffs) {
        nonZeroDistortion = nonZeroDistortion || std::abs(coefficient) > 1e-8F;
    }
    if (intrinsics.model != RS2_DISTORTION_NONE &&
        intrinsics.model != RS2_DISTORTION_BROWN_CONRADY && nonZeroDistortion) {
        throw std::runtime_error("unsupported non-zero RealSense distortion model: " +
                                 std::string(rs2_distortion_to_string(intrinsics.model)));
    }

    std::ifstream input(templatePath);
    if (!input) {
        throw std::runtime_error("cannot read settings template: " + templatePath);
    }

    std::map<std::string, std::string> replacements = {
        {"Camera1.fx", formatReal(intrinsics.fx)},
        {"Camera1.fy", formatReal(intrinsics.fy)},
        {"Camera1.cx", formatReal(intrinsics.ppx)},
        {"Camera1.cy", formatReal(intrinsics.ppy)},
        {"Camera1.k1", formatReal(intrinsics.coeffs[0])},
        {"Camera1.k2", formatReal(intrinsics.coeffs[1])},
        {"Camera1.p1", formatReal(intrinsics.coeffs[2])},
        {"Camera1.p2", formatReal(intrinsics.coeffs[3])},
        {"Camera1.k3", formatReal(intrinsics.coeffs[4])},
        {"Camera.width", std::to_string(capture.width)},
        {"Camera.height", std::to_string(capture.height)},
        {"Camera.fps", std::to_string(capture.fps)},
    };
    std::map<std::string, bool> replaced;
    for (const auto& [key, value] : replacements) {
        static_cast<void>(value);
        replaced.emplace(key, false);
    }

    std::ostringstream output;
    std::string line;
    while (std::getline(input, line)) {
        const std::size_t first = line.find_first_not_of(" \t");
        const std::size_t colon = first == std::string::npos ? std::string::npos :
                                  line.find(':', first);
        if (colon != std::string::npos) {
            const std::string key = trim(line.substr(first, colon - first));
            const auto replacement = replacements.find(key);
            if (replacement != replacements.end()) {
                output << line.substr(0, first) << key << ": " << replacement->second << '\n';
                replaced[key] = true;
                continue;
            }
        }
        output << line << '\n';
    }
    for (const auto& [key, wasReplaced] : replaced) {
        if (!wasReplaced) {
            throw std::runtime_error("settings template is missing required key: " + key);
        }
    }

    std::filesystem::create_directories(runtimeDirectory);
    const std::string safeSerial = serial.empty() ? "device" : serial;
    const std::filesystem::path effectivePath =
        std::filesystem::path(runtimeDirectory) /
        ("D435_Monocular_" + safeSerial + ".effective.yaml");
    std::ofstream effective(effectivePath);
    if (!effective) {
        throw std::runtime_error("cannot write effective settings: " + effectivePath.string());
    }
    effective << output.str();
    return effectivePath.string();
}

const char* trackingStateName(int state) {
    switch (state) {
        case -1: return "SYSTEM_NOT_READY";
        case 0: return "NO_IMAGES_YET";
        case 1: return "NOT_INITIALIZED";
        case 2: return "OK";
        case 3: return "RECENTLY_LOST";
        case 4: return "LOST";
        case 5: return "OK_KLT";
        default: return "UNKNOWN";
    }
}

void ensureParentDirectory(const std::string& filePath) {
    const std::filesystem::path parent = std::filesystem::path(filePath).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
}

}  // namespace

int main(int argc, char** argv) {
    rs2::pipeline pipeline;
    bool pipelineStarted = false;
    std::unique_ptr<ORB_SLAM3::System> slam;

    try {
        const CommandLine options = parseCommandLine(argc, argv);
        if (options.help) {
            printUsage(argv[0]);
            return 0;
        }

        CaptureSettings capture = readCaptureSettings(options.settingsPath);
        if (options.warmupFramesOverride >= 0) {
            capture.warmupFrames = options.warmupFramesOverride;
        }

        const rs2::device selectedDevice = selectDevice(options.serial);
        const std::string serial = deviceInfo(selectedDevice, RS2_CAMERA_INFO_SERIAL_NUMBER);
        configureSensors(selectedDevice, capture);

        rs2::config configuration;
        configuration.enable_device(serial);
        configuration.enable_stream(RS2_STREAM_INFRARED,
                                    capture.infraredIndex,
                                    capture.width,
                                    capture.height,
                                    RS2_FORMAT_Y8,
                                    capture.fps);
        const rs2::pipeline_profile profile = pipeline.start(configuration);
        pipelineStarted = true;

        const rs2::stream_profile stream = profile.get_stream(
            RS2_STREAM_INFRARED, capture.infraredIndex);
        const rs2_intrinsics intrinsics =
            stream.as<rs2::video_stream_profile>().get_intrinsics();
        if (intrinsics.width != capture.width || intrinsics.height != capture.height) {
            throw std::runtime_error("active D435 profile differs from requested resolution");
        }

        std::cout << "RealSense: " << deviceInfo(profile.get_device(), RS2_CAMERA_INFO_NAME)
                  << "  S/N " << serial
                  << "  FW " << deviceInfo(profile.get_device(), RS2_CAMERA_INFO_FIRMWARE_VERSION)
                  << '\n'
                  << "IR" << capture.infraredIndex << ": " << intrinsics.width << 'x'
                  << intrinsics.height << " Y8 @ " << capture.fps << " Hz\n"
                  << std::fixed << std::setprecision(6)
                  << "Active intrinsics: fx=" << intrinsics.fx << " fy=" << intrinsics.fy
                  << " cx=" << intrinsics.ppx << " cy=" << intrinsics.ppy
                  << " distortion=" << intrinsics.model << '\n';

        const std::string effectiveSettings = writeEffectiveSettings(options.settingsPath,
                                                                       options.runtimeDirectory,
                                                                       serial,
                                                                       intrinsics,
                                                                       capture);
        std::cout << "Effective calibrated settings: " << effectiveSettings << '\n';

        std::signal(SIGINT, handleSignal);
        std::signal(SIGTERM, handleSignal);

        int warmedFrames = 0;
        while (!gStopRequested && warmedFrames < capture.warmupFrames) {
            if (tryGetInfraredFrame(pipeline, capture.infraredIndex, capture.timeoutMs)) {
                ++warmedFrames;
            }
        }
        if (gStopRequested) {
            throw std::runtime_error("stopped during camera warmup");
        }

        slam = std::make_unique<ORB_SLAM3::System>(options.vocabularyPath,
                                                   effectiveSettings,
                                                   ORB_SLAM3::System::MONOCULAR,
                                                   options.useViewer);

        std::uint64_t lastFrameNumber = 0;
        bool havePreviousFrame = false;
        double lastTimestamp = -1.0;
        int trackedFrames = 0;
        std::uint64_t droppedFrames = 0;
        int previousState = -2;

        while (!gStopRequested && !slam->isShutDown() &&
               (options.maxFrames == 0 || trackedFrames < options.maxFrames)) {
            const std::optional<ImagePacket> packet =
                tryGetInfraredFrame(pipeline, capture.infraredIndex, capture.timeoutMs);
            if (!packet) {
                continue;
            }
            if (!std::isfinite(packet->timestampSeconds)) {
                throw std::runtime_error("D435 supplied a non-finite timestamp");
            }
            if (lastTimestamp >= 0.0 && packet->timestampSeconds <= lastTimestamp) {
                if (packet->timestampSeconds == lastTimestamp) {
                    continue;
                }
                throw std::runtime_error("D435 timestamp moved backwards; ending this SLAM session");
            }
            if (havePreviousFrame && packet->frameNumber > lastFrameNumber + 1) {
                droppedFrames += packet->frameNumber - lastFrameNumber - 1;
            }
            lastFrameNumber = packet->frameNumber;
            havePreviousFrame = true;
            lastTimestamp = packet->timestampSeconds;

            slam->TrackMonocular(packet->image, packet->timestampSeconds);
            ++trackedFrames;

            const int state = slam->GetTrackingState();
            if (state != previousState || trackedFrames % 30 == 0) {
                std::cout << "frame=" << trackedFrames
                          << " state=" << trackingStateName(state)
                          << " dropped=" << droppedFrames << '\n';
                previousState = state;
            }
        }

        pipeline.stop();
        pipelineStarted = false;
        slam->Shutdown();
        if (options.saveTrajectory) {
            ensureParentDirectory(options.keyframeTrajectory);
            slam->SaveKeyFrameTrajectoryTUM(options.keyframeTrajectory);
            std::cout << "Saved monocular keyframe trajectory: "
                      << options.keyframeTrajectory << '\n';
        }
        if (options.savePointCloud) {
            ensureParentDirectory(options.pointCloud);
            slam->SaveMapPointsPLY(options.pointCloud);
        }
        return 0;
    } catch (const std::exception& error) {
        if (pipelineStarted) {
            try {
                pipeline.stop();
            } catch (...) {
            }
        }
        if (slam) {
            try {
                slam->Shutdown();
            } catch (...) {
            }
        }
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}
