#ifndef TEST_H
#define TEST_H

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace testutil 
{

namespace fs = std::filesystem;

inline bool near(double a, double b, double eps = 1e-12) {
    return std::abs(a - b) <= eps;
}

[[noreturn]] inline void fail(const std::string& msg) {
    std::cerr << "[FAIL] " << msg << "\n";
    std::exit(1);
}

inline void expect(bool cond, const std::string& msg) {
    if (!cond) fail(msg);
}

inline fs::path makeTempDir(const std::string& prefix) {
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    const fs::path dir = fs::temp_directory_path() / (prefix + "_" + std::to_string(stamp));
    fs::create_directories(dir);
    return dir;
}

inline void cleanupDir(const fs::path& dir) {
    std::error_code ec;
    fs::remove_all(dir, ec);
}

} // namespace testutil

#endif

