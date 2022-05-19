#pragma once

#include <thread>
#include <vector>
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include "open3d/Open3D.h"

using namespace open3d::io;

namespace open3d {
namespace io {
/// Runs Azurekinect capture thread. This function is called when user click start button.
class CAzk {

public:
    ~CAzk() = default;
    static CAzk* GetInstance() {
        if (_instance == nullptr) 
            _instance = new CAzk();
        return _instance;
    }
    static std::string get_connected_device_id();
    static k4a_device_configuration_t get_default_config();
    static k4a_device_configuration_t get_master_config();
    static k4a_device_configuration_t get_subordinate_config(int idx);
    static void run();
    static void capturer();
    static void stop();
    static void generate_point_cloud(std::shared_ptr<open3d::geometry::PointCloud> pointCloud, const k4a_image_t depth_image, const k4a_image_t color_image);

private:
    CAzk() = default;

private:
    static CAzk* _instance;
    static bool isRunning;
    static std::string _deviceId;
    static std::thread _thread;
};
}  // namespace visualization
}  // namespace open3d
