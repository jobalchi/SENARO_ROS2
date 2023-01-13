#include <iostream>
#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <iostream>
#include <thread>
#include <regex>
#include <stitcher/stitcher.h>
#include <stitcher/common.h>
#include <camera/camera.h>
#include <camera/photography_settings.h>
#include <camera/device_discovery.h>
#include "rclcpp/rclcpp.hpp"
#include "insta360_interface/srv/take_picture.hpp"

using TakePicture = insta360_interface::srv::TakePicture;

class TestStreamDelegate : public ins_camera::StreamDelegate {
public:
    TestStreamDelegate() {
        file1_ = fopen("./01.h264", "wb");
        file2_ = fopen("./02.h264", "wb");
    }
    ~TestStreamDelegate() {
        fclose(file1_);
        fclose(file2_);
    }

    void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) override {
        std::cout << "on audio data:" << std::endl;
    }
    void OnVideoData(const uint8_t* data, size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) override {
        //std::cout << "on video frame:" << size << ";" << timestamp << std::endl;
        if (stream_index == 0) {
            fwrite(data, sizeof(uint8_t), size, file1_);
        }
        if (stream_index == 1) {
            fwrite(data, sizeof(uint8_t), size, file2_);
        }
    }
    void OnGyroData(const std::vector<ins_camera::GyroData>& data) override {
    }
    void OnExposureData(const ins_camera::ExposureData& data) override {
    }

private:
    FILE* file1_;
    FILE* file2_;
    int64_t last_timestamp = 0;
};

std::shared_ptr<ins_camera::Camera> cam;

class BasicServer : public rclcpp::Node {
private:
  rclcpp::Service<TakePicture>::SharedPtr m_service;

public:
  BasicServer() : Node("insta360_node") {
    RCLCPP_WARN(get_logger(), "insta360 service Server Started");
    std::cout << "begin open camera" << std::endl;
    ins_camera::DeviceDiscovery discovery;
    auto list = discovery.GetAvailableDevices();
    for (int i = 0; i < list.size(); ++i) {
        auto desc = list[i];
        std::cout << "serial:" << desc.serial_number << "\t"
            << "camera type:" << int(desc.camera_type) << "\t"
            << "lens type:" << int(desc.lens_type) << std::endl;
    }

    if (list.size() <= 0) {
        std::cerr << "no device found." << std::endl;
    }
    cam = std::make_shared<ins_camera::Camera>(list[0].info);
    //ins_camera::Camera cam(list[0].info);
    if (!cam->Open()) {
        std::cerr << "failed to open camera" << std::endl;
    }
    std::cout << "http base url:" << cam->GetHttpBaseUrl() << std::endl;

    std::shared_ptr<ins_camera::StreamDelegate> delegate = std::make_shared<TestStreamDelegate>();
    cam->SetStreamDelegate(delegate);

    discovery.FreeDeviceDescriptors(list);

    std::cout << "Succeed to open camera..." << std::endl;
    auto camera_type = cam->GetCameraType();

    auto start = time(NULL);
    cam->SyncLocalTimeToCamera(start);
    m_service = create_service<TakePicture>(
        "insta360_node/take_picture",
        std::bind(&BasicServer::response, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

  void response(
    std::shared_ptr<TakePicture::Request> request,
    std::shared_ptr<TakePicture::Response> response
  ) {
    std::string target_path = request->target_path;
    std::string source_path = request->source_path;
    bool execute_take_photo = request->execute_take_photo;

    if (execute_take_photo) {
        const auto url = cam->TakePhoto();
        if (!url.IsSingleOrigin() || url.Empty()) {
            std::cout << "failed to take picture" << std::endl;
        }
        source_path = url.GetSingleOrigin();
    }

    std::cout << "source_path: " << source_path << std::endl;

    std::string original_path = target_path + ".insp";
    std::string stitcher_path = target_path + ".jpg";

    const auto ret = cam->DownloadCameraFile(source_path, original_path);
    if (ret) {
        std::cout << "Download " << target_path << " succeed!!!" << std::endl;
        std::vector<std::string> input_paths = { original_path };
        STITCH_TYPE stitch_type = STITCH_TYPE::TEMPLATE;
        HDR_TYPE hdr_type = HDR_TYPE::ImageHdr_NONE;

        int output_width = 1920;
        int output_height = 960;
        int output_bitrate = 0;
        int gpu = 0;

        bool enable_flowstate = false;
        bool enable_cuda = true;
        bool enalbe_stitchfusion = false;
        bool enable_colorplus = false;
        bool enable_directionlock = false;
        bool enable_denoise = false;
        std::string colorpuls_model_path;

        auto image_stitcher = std::make_shared<ins_media::ImageStitcher>();
        image_stitcher->SetInputPath(input_paths);
        image_stitcher->SetStitchType(stitch_type);
        image_stitcher->SetHDRType(hdr_type);
        image_stitcher->SetOutputPath(stitcher_path);
        image_stitcher->SetOutputSize(output_width, output_height);
        image_stitcher->EnableFlowState(enable_flowstate);
        image_stitcher->EnableDenoise(enable_denoise);
        image_stitcher->EnableColorPlus(enable_colorplus, colorpuls_model_path);
        image_stitcher->Stitch();
        response->is_successed = true;
        response->source_path = target_path;
    }
    else {
        std::cout << "Download " << target_path << " failed!!!" << std::endl;
        response->is_successed = false;
        response->source_path = "";
    }
  }
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BasicServer>();

    rclcpp::spin(node);
    cam->Close();
    rclcpp::shutdown();    
    return 0;
}
