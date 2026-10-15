#pragma once
// BSD-3-Clause. Call only from an ordinary-priority application/executor thread.
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace eir::service {
inline std::string quote(const std::string& value) {
    std::string out = "\"";
    constexpr char hex[] = "0123456789abcdef";
    for (const unsigned char c : value) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c < 32) { out += "\\u00"; out += hex[c >> 4]; out += hex[c & 15]; }
        else out += c;
    }
    return out + '"';
}
inline uint64_t now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline void checked(const std::string& robot, const std::string& sdk_config) {
    std::cout << "{\"protocol\":1,\"robot\":" << quote(robot)
              << ",\"backend\":\"hardware\",\"sdk_config\":" << quote(sdk_config) << "}\n";
}
class Health {
    std::string path_, robot_, nonce_, instance_, deployment_;
    static std::string environment(const char* name) {
        const auto* value = std::getenv(name);
        if (!value || !*value) throw std::runtime_error(std::string("Missing managed environment: ") + name);
        return value;
    }
public:
    explicit Health(std::string robot) : path_(environment("EIR_MANAGER_HEALTH_FILE")), robot_(std::move(robot)),
        nonce_(environment("EIR_MANAGER_NONCE")), instance_(environment("EIR_MANAGER_INSTANCE")),
        deployment_(environment("EIR_MANAGER_DEPLOYMENT")) {
        if (!std::filesystem::path(path_).is_absolute() ||
            !std::filesystem::is_directory(std::filesystem::path(path_).parent_path()) ||
            std::filesystem::is_symlink(std::filesystem::path(path_).parent_path()))
            throw std::runtime_error("Invalid managed health directory");
    }
    void publish(unsigned mode, unsigned fault, uint64_t sequence, uint64_t feedback_stamp) const {
        std::ostringstream text;
        text << "{\"protocol\":1,\"pid\":" << getpid() << ",\"robot\":" << quote(robot_)
             << ",\"backend\":\"hardware\",\"instance\":" << quote(instance_)
             << ",\"deployment\":" << quote(deployment_) << ",\"nonce\":" << quote(nonce_)
             << ",\"monotonic_ns\":" << now() << ",\"mode\":" << mode << ",\"fault\":" << fault
             << ",\"feedback_sequence\":" << sequence << ",\"feedback_stamp_ns\":" << feedback_stamp << "}\n";
        const auto body = text.str();
        std::string temporary = path_ + ".XXXXXX";
        const int fd = mkstemp(temporary.data()); // Private file; atomic replacement, no symlink following.
        if (fd < 0) throw std::runtime_error("Cannot create managed health file");
        bool ok = true;
        std::size_t offset = 0;
        while (offset < body.size()) {
            const auto written = write(fd, body.data() + offset, body.size() - offset);
            if (written <= 0) { ok = false; break; }
            offset += static_cast<std::size_t>(written);
        }
        if (close(fd) || !ok || rename(temporary.c_str(), path_.c_str())) {
            unlink(temporary.c_str());
            throw std::runtime_error("Cannot publish managed health");
        }
    }
};
} // namespace eir::service
