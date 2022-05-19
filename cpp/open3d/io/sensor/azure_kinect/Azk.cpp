// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------
#include "open3d/utility/Logging.h"
#include "Azk.h"
#include "PcdQueue.h"
#include <iostream>
#include <string>
//#include "open3d/utility/Log\ging.h"

namespace open3d {
namespace io {

CAzk* CAzk::_instance = nullptr;
bool CAzk::isRunning = false;
std::string CAzk::_deviceId = "";
std::thread CAzk::_thread;


std::string CAzk::get_connected_device_id() { 
    const uint32_t installedDevices = k4a_plugin::k4a_device_get_installed_count();
    k4a_device_t device=NULL;
    std::unique_ptr<char[]> _id;
    size_t id_len = 0;
    if(installedDevices>0) {
        if (K4A_RESULT_SUCCEEDED == k4a_plugin::k4a_device_open(K4A_DEVICE_DEFAULT, &device)) {
            k4a_buffer_result_t result_t = k4a_plugin::k4a_device_get_serialnum(device, NULL, &id_len);
            _id = std::make_unique<char[]>(id_len);
            result_t = k4a_plugin::k4a_device_get_serialnum(device, _id.get(), &id_len);
            _deviceId = std::move(_id.get());
        }
    } else {
        _deviceId = "No Devices";
    }

    return _deviceId;
}

void CAzk::run() { 
    isRunning = true;
    _thread = std::thread(CAzk::capturer); 
}

void CAzk::capturer() {
    constexpr int32_t TIMEOUT_IN_MS = 5000;
    int device_num = k4a_plugin::k4a_device_get_installed_count();
    if (device_num == 0) return;
    k4a_result_t result;
    k4a_device_t device = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation;
    k4a_device_configuration_t config = CAzk::get_master_config();
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;

    if (K4A_RESULT_SUCCEEDED != k4a_plugin::k4a_device_open(K4A_DEVICE_DEFAULT, &device)) {
        open3d::utility::LogError("Failed to open device");
        k4a_plugin::k4a_device_close(device);
        return;
    }

    result = k4a_plugin::k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration);

    if (K4A_RESULT_SUCCEEDED != result) {
        open3d::utility::LogError("k4a_device_get_calibration fail : {}", result);
        k4a_plugin::k4a_device_close(device);
        return;
    }

    transformation = k4a_plugin::k4a_transformation_create(&calibration);

    result = k4a_plugin::k4a_device_start_cameras(device, &config);
    if (K4A_RESULT_SUCCEEDED != result) {
        open3d::utility::LogError("k4a_device_start_cameras fail : {}", result);
        k4a_plugin::k4a_device_close(device);
        return;
    }

    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;

    while (isRunning) {
        std::shared_ptr<open3d::geometry::PointCloud> pcd = std::make_shared<open3d::geometry::PointCloud>();

        switch (k4a_plugin::k4a_device_get_capture(device, &capture,
                                                   TIMEOUT_IN_MS)) {
            case K4A_WAIT_RESULT_SUCCEEDED:
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                continue;
                break;
            case K4A_WAIT_RESULT_FAILED:
                k4a_plugin::k4a_device_close(device);
                k4a_plugin::k4a_capture_release(capture);
                open3d::utility::LogError("K4A_WAIT_RESULT_FAILED");
                return;
        }

        depth_image = k4a_plugin::k4a_capture_get_depth_image(capture);
        color_image = k4a_plugin::k4a_capture_get_color_image(capture);

        int width = k4a_plugin::k4a_image_get_width_pixels(depth_image);
        int height = k4a_plugin::k4a_image_get_height_pixels(depth_image);

        if (width == 0 || height == 0) continue;

        k4a_image_t point_cloud_image = nullptr;

        if (K4A_RESULT_SUCCEEDED !=
            k4a_plugin::k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, width, height, width * 3 * (int)sizeof(int16_t),
                             &point_cloud_image)) {
            k4a_plugin::k4a_image_release(point_cloud_image);
            k4a_plugin::k4a_image_release(color_image);
            k4a_plugin::k4a_image_release(color_image);
            k4a_plugin::k4a_capture_release(capture);
            continue;
        }
        
        if (K4A_RESULT_SUCCEEDED !=
            k4a_plugin::k4a_transformation_depth_image_to_point_cloud(
                    transformation, depth_image, K4A_CALIBRATION_TYPE_DEPTH,
                    point_cloud_image)) {
            k4a_plugin::k4a_image_release(point_cloud_image);
            k4a_plugin::k4a_image_release(color_image);
            k4a_plugin::k4a_image_release(depth_image);
            k4a_plugin::k4a_capture_release(capture);
            continue;
        }

        k4a_image_t transformed_color_image = nullptr;
        if (K4A_RESULT_SUCCEEDED !=
            k4a_plugin::k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width,
                                         height,
                             width * 4 * (int)sizeof(uint8_t),
                             &transformed_color_image)) {
            k4a_plugin::k4a_image_release(transformed_color_image);
            k4a_plugin::k4a_image_release(point_cloud_image);
            k4a_plugin::k4a_image_release(color_image);
            k4a_plugin::k4a_image_release(depth_image);
            k4a_plugin::k4a_capture_release(capture);
            continue;
        }

        if (K4A_RESULT_SUCCEEDED !=
            k4a_plugin::k4a_transformation_color_image_to_depth_camera(
                    transformation, depth_image, color_image,
                    transformed_color_image)) {
            k4a_plugin::k4a_image_release(transformed_color_image);
            k4a_plugin::k4a_image_release(point_cloud_image);
            k4a_plugin::k4a_image_release(color_image);
            k4a_plugin::k4a_image_release(depth_image);
            k4a_plugin::k4a_capture_release(capture);
            continue;
        }

        generate_point_cloud(pcd, point_cloud_image, transformed_color_image);

        CPcdQueue::GetInstance()->push(pcd);

        k4a_plugin::k4a_image_release(transformed_color_image);
        k4a_plugin::k4a_image_release(point_cloud_image);
        k4a_plugin::k4a_image_release(depth_image);
        k4a_plugin::k4a_image_release(color_image);
        k4a_plugin::k4a_capture_release(capture);
        pcd.reset();
    }
}

void CAzk::generate_point_cloud(std::shared_ptr<open3d::geometry::PointCloud> pointCloud, 
                                const k4a_image_t depth_image, 
                                const k4a_image_t color_image) {
    int nCntDet = 0;
    bool status = false;
    int width = k4a_plugin::k4a_image_get_width_pixels(depth_image);
    int height = k4a_plugin::k4a_image_get_height_pixels(depth_image);
    int size = width * height;
    int16_t* depth_data = (int16_t*)(void*)k4a_plugin::k4a_image_get_buffer(depth_image);
    uint8_t* color_data = k4a_plugin::k4a_image_get_buffer(color_image);
    int count = 0;
    for (int i = 0; i < size; i++) {
        if (depth_data[3 * i + 2] == 0) continue;
        Eigen::Vector3d point;
        point.x() = depth_data[3 * i + 0];
        point.y() = depth_data[3 * i + 1];
        point.z() = depth_data[3 * i + 2];
        pointCloud->points_.push_back(point);

        Eigen::Vector3d color(color_data[4 * i + 2] / 255.0,
                              color_data[4 * i + 1] / 255.0,
                              color_data[4 * i + 0] / 255.0);

        pointCloud->colors_.push_back(color);
        count++;
    }
}

k4a_device_configuration_t CAzk::get_default_config() {
    k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;  // No need for depth during calibration
    camera_config.camera_fps = K4A_FRAMES_PER_SECOND_30;  // Don't use all USB bandwidth
    camera_config.subordinate_delay_off_master_usec = 0;  // Must be zero for master
    camera_config.synchronized_images_only = true;  // ensures that depth and color images are both available in the capture
    return camera_config;
}

k4a_device_configuration_t CAzk::get_master_config() {
    k4a_device_configuration_t camera_config = get_default_config();
    camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
    camera_config.depth_delay_off_color_usec = 0;         // Internal Sync
    camera_config.subordinate_delay_off_master_usec = 0;  // External Mode
    camera_config.synchronized_images_only = true;  // ensures that depth and color images are both available in the capture
    return camera_config;
}

k4a_device_configuration_t CAzk::get_subordinate_config(int idx) {
    k4a_device_configuration_t camera_config = get_default_config();
    camera_config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
    camera_config.depth_delay_off_color_usec = 0;  // Internal Sync
    camera_config.subordinate_delay_off_master_usec = idx * 160;  // External Mode
    return camera_config;
}

void CAzk::stop() { 
	isRunning = false;
    _thread.join();
}

}  // namespace io
}  // namespace open3d
