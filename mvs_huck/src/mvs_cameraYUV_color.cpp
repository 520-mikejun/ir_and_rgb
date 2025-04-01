#include <mvs_camera/mvs_camera.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "MvCameraControl.h"

namespace mvs_camera
{

  MvsCamera::MvsCamera(const ros::NodeHandle &nh, const std::string &publish_topic)
      : nh_(nh), publish_topic_(publish_topic)
  {
    pub_ = nh_.advertise<sensor_msgs::Image>(publish_topic_, 10);
    color_pub_ = nh_.advertise<sensor_msgs::Image>("/color_hk", 10);
    rectify_pub_ = nh_.advertise<sensor_msgs::Image>("/rectify_hk", 10); // 新增
    rectify_raw_pub_ = nh_.advertise<sensor_msgs::Image>("/rectify_raw", 10); // 新增

    // 设置相机内参和畸变系数
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 378.7773, 0.0, 321.8907, 0.0, 378.5598, 255.4721, 0.0, 0.0, 1.0);
    dist_coeffs_ = (cv::Mat_<double>(5, 1) << -0.1624, 0.0195, 0.0293, 0.0, 0.0);
  }

  MvsCamera::~MvsCamera() { CloseDevice(); }

  void MvsCamera::Init() { OpenDevice(); }

  void MvsCamera::Start() { StartGrabbing(); }

  void MvsCamera::OpenDevice()
  {
    ROS_INFO("Opening device.....");
    int nRet = MV_OK;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Enum Devices fail! nRet [0x%x]\n", nRet);
      std::exit(1);
    }
    if (stDeviceList.nDeviceNum > 0)
    {
      ROS_INFO("Found %d devices, use first one", stDeviceList.nDeviceNum);
    }
    else
    {
      ROS_ERROR("Find No Devices!\n");
      std::exit(2);
    }

    PrintDeviceInfo(stDeviceList.pDeviceInfo[0]);

    nRet = MV_CC_CreateHandle(&camera_handle_, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Create Handle fail! nRet is [0x%x]\n", nRet);
      std::exit(3);
    }

    nRet = MV_CC_OpenDevice(camera_handle_);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Open Device fail! nRet [0x%x]", nRet);
      std::exit(4);
    }

    if (stDeviceList.pDeviceInfo[0]->nTLayerType == MV_GIGE_DEVICE)
    {
      int nPacketSize = MV_CC_GetOptimalPacketSize(camera_handle_);
      if (nPacketSize > 0)
      {
        nRet = MV_CC_SetIntValue(camera_handle_, "GevSCPSPacketSize", nPacketSize);
        if (nRet != MV_OK)
        {
          ROS_WARN("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
        }
      }
      else
      {
        ROS_WARN("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
      }
      optimal_packet_size_ = nPacketSize;
    }

    nRet = MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
      std::exit(5);
    }

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(camera_handle_, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Get PayloadSize fail! nRet [0x%x]\n", nRet);
      std::exit(6);
    }
    payload_size_ = stParam.nCurValue;

    nRet = MV_CC_StartGrabbing(camera_handle_);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Start Grabbing fail! nRet [0x%x]\n", nRet);
      std::exit(7);
    }
    ROS_INFO("Camera started!");
  }

  void MvsCamera::CloseDevice()
  {
    ROS_INFO("Closing device.....");
    int nRet = 0;
    nRet = MV_CC_StopGrabbing(camera_handle_);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Stop Grabbing fail! nRet [0x%x]\n", nRet);
      std::exit(11);
    }

    nRet = MV_CC_CloseDevice(camera_handle_);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Close Device fail! nRet [0x%x]\n", nRet);
      std::exit(12);
    }

    nRet = MV_CC_DestroyHandle(camera_handle_);
    if (MV_OK != nRet)
    {
      ROS_ERROR("Destroy handle fail! nRet [0x%x]\n", nRet);
      std::exit(13);
    }

    if (nRet != MV_OK)
    {
      if (camera_handle_ != NULL)
      {
        MV_CC_DestroyHandle(camera_handle_);
        camera_handle_ = NULL;
      }
    }
  }

  bool MvsCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO *device_info)
  {
    if (nullptr == device_info)
    {
      ROS_INFO("The Pointer of device_info is NULL");
      return false;
    }

    if (device_info->nTLayerType == MV_GIGE_DEVICE)
    {
      int ip1 = ((device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
      int ip2 = ((device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
      int ip3 = ((device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
      int ip4 = (device_info->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

      ROS_INFO("Device Model Name: %s", device_info->SpecialInfo.stGigEInfo.chModelName);
      ROS_INFO("CurrentIp: %d.%d.%d.%d", ip1, ip2, ip3, ip4);
      ROS_INFO("UserDefinedName: %s", device_info->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (device_info->nTLayerType == MV_USB_DEVICE)
    {
      ROS_INFO("Device Model Name: %s", device_info->SpecialInfo.stUsb3VInfo.chModelName);
      ROS_INFO("UserDefinedName: %s", device_info->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
      ROS_INFO("Device is not support");
    }

    return true;
  }

  void MvsCamera::StartGrabbing()
  {
    ROS_INFO("Start grabbing image.....");
    MV_FRAME_OUT stOutFrame = {0};
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    int64_t frame_count = 0;

    while (ros::ok())
    {
      int nRet = MV_CC_GetImageBuffer(camera_handle_, &stOutFrame, 1000);
      if (nRet == MV_OK)
      {
        ROS_DEBUG("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
                  stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight,
                  stOutFrame.stFrameInfo.nFrameNum);
      }
      else
      {
        ROS_WARN("No data[0x%x]\n", nRet);
        continue;
      }

      // 转换为 OpenCV 格式 (YUV 格式的图像)
      cv::Mat image(cv::Size(stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight), CV_8UC2, stOutFrame.pBufAddr);

      // 转换为 RGB 格式
      cv::Mat image_rgb;
      cv::cvtColor(image, image_rgb, cv::COLOR_YUV2BGR_YUY2);
      

        // 保存图像并检查是否成功
      std::string output_path = "/home/sz/fast_lio_copy/1.bmp";
      bool success = cv::imwrite(output_path, image_rgb);      // 将 OpenCV 图像转换为 ROS 图像消息
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_rgb).toImageMsg();

      // 设置消息头
      msg->header.stamp = ros::Time::now();
      msg->header.seq = frame_count;
      msg->header.frame_id = "camera_init";

      // 发布原始图像消息
      pub_.publish(msg);

      // 提取 Y 通道（亮度）信息
      cv::Mat yChannel;
      std::vector<cv::Mat> channels(3);
      cv::split(image, channels);  // 从原始 YUV 数据中拆分出 Y、U、V 通道
      yChannel = channels[0];       // 使用第一个通道 Y

      // 确保 Y 通道为 CV_8UC1 类型（避免错误）
      yChannel.convertTo(yChannel, CV_8UC1);

      // 归一化 Y 通道的值到 0-255 范围
      cv::Mat yChannelNormalized;
      cv::normalize(yChannel, yChannelNormalized, 0, 600, cv::NORM_MINMAX);

      // 使用伪彩色调色板将归一化后的 Y 通道转换为伪彩色图像
      cv::Mat image_colormap;
      cv::applyColorMap(yChannelNormalized, image_colormap, cv::COLORMAP_JET);

      // 将彩色图像转换为 ROS 图像消息
      sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_colormap).toImageMsg();

      // 设置消息头
      color_msg->header.stamp = ros::Time::now();
      color_msg->header.seq = frame_count;
      color_msg->header.frame_id = "camera_init";

      // 发布彩色图像消息
      color_pub_.publish(color_msg);

      // 原始图像矫正
      cv::Mat rectify_raw;
      cv::undistort(image_rgb, rectify_raw, camera_matrix_, dist_coeffs_);

      // 将矫正后的图像转换为 ROS 图像消息
      sensor_msgs::ImagePtr rectify_rawmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectify_raw).toImageMsg();
      rectify_rawmsg->header.stamp = ros::Time::now();
      rectify_rawmsg->header.seq = frame_count;
      rectify_rawmsg->header.frame_id = "camera_init";
      // 发布矫正后的图像
      rectify_raw_pub_.publish(rectify_rawmsg);

      // 图像矫正
      cv::Mat rectify_image;
      cv::undistort(image_colormap, rectify_image, camera_matrix_, dist_coeffs_);

      // 将矫正后的图像转换为 ROS 图像消息
      sensor_msgs::ImagePtr rectify_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectify_image).toImageMsg();
      rectify_msg->header.stamp = ros::Time::now();
      rectify_msg->header.seq = frame_count;
      rectify_msg->header.frame_id = "camera_init";
      // 发布矫正后的图像
      rectify_pub_.publish(rectify_msg);
      ros::spinOnce();
      frame_count += 1;

      if (NULL != stOutFrame.pBufAddr)
      {
        nRet = MV_CC_FreeImageBuffer(camera_handle_, &stOutFrame);
        if (nRet != MV_OK)
        {
          ROS_ERROR("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        }
      }
    }

    CloseDevice();
  }

} // namespace mvs_camera