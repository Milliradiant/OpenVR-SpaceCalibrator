#include "session.h"

#include <sstream>

namespace bench {

using namespace spacecal::recording;

constexpr int k_TRACKING_RESULT_RUNNING_OK = 200; // vr::TrackingResult_Running_OK

std::string Device::name() const
{
    std::string text = "[" + std::to_string(index) + "]";
    if (!serial.empty()) {
        text += " " + serial;
    }
    return text;
}

bool Device::worldAt(double time, Pose& pose, double maxGap) const
{
    if (samples.size() < 2 || time < samples.front().time || time > samples.back().time) {
        return false;
    }
    const auto upper = std::lower_bound(samples.begin(), samples.end(), time, [](const Sample& s, double t) { return s.time < t; });
    if (upper == samples.begin()) {
        if (!upper->valid) {
            return false;
        }
        pose = upper->world;
        return true;
    }
    const Sample& b = *upper;
    const Sample& a = *(upper - 1);
    if (!a.valid || !b.valid || b.time - a.time > maxGap) {
        return false;
    }
    const double span = b.time - a.time;
    pose = interpolate(a.world, b.world, span > 1e-9 ? (time - a.time) / span : 0.0);
    return true;
}

static Pose poseOf(const double rotation[4], const double translation[3])
{
    return Pose {
        Eigen::Quaterniond(rotation[0], rotation[1], rotation[2], rotation[3]).normalized(),
        Eigen::Vector3d(translation[0], translation[1], translation[2]),
    };
}

bool Session::load(const std::filesystem::path& file, std::string& error)
{
    Reader reader;
    if (!reader.open(file)) {
        error = reader.error();
        return false;
    }
    path = file;

    FrameRecord frames[k_MAX_DEVICES] = {};
    std::map<int, std::map<int, int>> pairVotes; // reference -> target -> how often the overlay named the pair

    Record record;
    while (reader.next(record)) {
        records++;
        duration = std::max(duration, record.time);
        Device* device = nullptr;
        if (record.device != k_NO_DEVICE) {
            device = &devices[record.device];
            device->index = record.device;
        }

        switch (record.type) {
        case RecordType::Session: {
            std::istringstream lines(record.text);
            std::string line;
            while (std::getline(lines, line)) {
                const size_t equals = line.find('=');
                if (equals != std::string::npos) {
                    info[line.substr(0, equals)] = line.substr(equals + 1);
                }
            }
            break;
        }
        case RecordType::Note:
            notes.emplace_back(record.time, record.text);
            break;
        case RecordType::Device:
            if (device) {
                std::istringstream fields(record.text);
                std::string deviceClassText;
                std::getline(fields, device->serial, '\t');
                std::getline(fields, device->model, '\t');
                std::getline(fields, device->trackingSystem, '\t');
                std::getline(fields, deviceClassText, '\t');
                device->deviceClass = deviceClassText.empty() ? 0 : std::atoi(deviceClassText.c_str());
            }
            break;
        case RecordType::Frame:
            if (device) {
                frames[record.device] = record.frame;
                if (!device->samples.empty()) {
                    device->frameChanges.push_back(record.time);
                }
            }
            break;
        case RecordType::Pose:
            if (device) {
                Sample sample;
                sample.time = record.time;
                sample.record = record.pose;
                sample.frame = frames[record.device];
                sample.valid = (record.pose.flags & PoseFlag_Valid) && (record.pose.flags & PoseFlag_Connected) && record.pose.result == k_TRACKING_RESULT_RUNNING_OK;
                sample.poseTimeOffset = record.pose.poseTimeOffset;
                sample.raw = poseOf(record.pose.rotation, record.pose.position);
                sample.world = poseOf(sample.frame.worldFromDriverRotation, sample.frame.worldFromDriverTranslation) * sample.raw;
                sample.velocity = Eigen::Vector3d(record.pose.velocity[0], record.pose.velocity[1], record.pose.velocity[2]);
                sample.angularVelocity = Eigen::Vector3d(record.pose.angularVelocity[0], record.pose.angularVelocity[1], record.pose.angularVelocity[2]);
                device->samples.push_back(sample);
            }
            break;
        case RecordType::Transform:
            if (device) {
                device->transforms.push_back({ record.time, record.transform });
                if ((record.transform.flags & TransformFlag_Enabled) && record.transform.referenceDevice < k_MAX_DEVICES && record.transform.targetDevice < k_MAX_DEVICES) {
                    pairVotes[static_cast<int>(record.transform.referenceDevice)][static_cast<int>(record.transform.targetDevice)]++;
                }
            }
            break;
        case RecordType::Applied:
            if (device) {
                device->applied.push_back({ record.time, poseOf(record.applied.rotation, record.applied.translation) });
            }
            break;
        case RecordType::Latency:
            if (device) {
                device->latency.push_back({ record.time, record.latency.latency, record.latency.confidence });
            }
            break;
        }
    }
    skippedRecords = reader.skippedRecords();

    int bestVotes = 0;
    for (const auto& [reference, targets] : pairVotes) {
        for (const auto& [target, votes] : targets) {
            if (votes > bestVotes) {
                bestVotes = votes;
                referenceIndex = reference;
                targetIndex = target;
            }
        }
    }
    return true;
}

const Device* Session::reference() const
{
    const auto it = devices.find(referenceIndex);
    return it == devices.end() ? nullptr : &it->second;
}

const Device* Session::target() const
{
    const auto it = devices.find(targetIndex);
    return it == devices.end() ? nullptr : &it->second;
}

} // namespace bench
