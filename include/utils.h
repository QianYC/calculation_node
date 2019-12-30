//
// Created by yc_qian on 19-12-18.
//
#include <k4a/k4atypes.h>
#include <k4a/k4aversion.h>
#include <string>
#include <sstream>

std::string version2String(const k4a_version_t &v) {
    std::stringstream ss;
    ss << v.major << '.' << v.minor << '.' << v.iteration;
    return ss.str();
}

std::string hardwareInfo2String(const k4a_hardware_version_t &h){
    std::stringstream ss;
    ss << "rgb: " << version2String(h.rgb) << ", depth: " << version2String(h.depth) << ",\naudio: "
       << version2String(h.audio) << ", depth sensor: " << version2String(h.depth_sensor) << ",\nbuild: "
       << h.firmware_build << ", signature: " << h.firmware_signature;
    return ss.str();
}

struct BgraPixel
{
    uint8_t Blue;
    uint8_t Green;
    uint8_t Red;
    uint8_t Alpha;
};