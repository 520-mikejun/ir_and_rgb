/* 
保留了一些对红外的处理
*/

#include "LIVMapper.h"
#include <std_msgs/String.h>
LIVMapper::~LIVMapper() {}
void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  ////初始化红外
  nh.param<bool>("ir/ir_enabled", ir_enabled, true);
  nh.param<double>("ir/ir_alpha", ir_alpha, 0.7);
  nh.param<int>("ir/integration_method", ir_integration_method, 0);
  //nh.param<double>("ir/heat_enhancement_factor", heat_enhancement_factor, 1.5);
  nh.param<double>("ir/heat_enhancement_factor", heat_enhancement_factor, 15.0); 
// 新增参数：调试信息输出频率
  nh.param<int>("ir/debug_frequency", ir_debug_frequency, 30);
    // 初始化红外相机内参矩阵
  K_ir = Eigen::Matrix3d::Identity();
  K_ir(0,0) = 371.28592631;
  K_ir(1,1) = 371.03828991;
  K_ir(0,2) = 321.7997769;
  K_ir(1,2) = 259.52376857;
    
    // 初始化红外到可见光的外参
  R_ir_to_cam = Eigen::Matrix3d::Identity();
  R_ir_to_cam << -0.57432029, -0.0489925, -0.81716335,
                  -0.10983116, -0.98457145, 0.13622109,
                  -0.81122951, 0.16798453, 0.56007845;
    
  T_ir_to_cam = Eigen::Vector3d::Zero();
  T_ir_to_cam << 812.46600029, 60.77312924, 313.87154682;
    
  if (ir_enabled) {
        std::cout << "ir have start" << std::endl;
        // std::cout << "neucan：\n" << K_ir << std::endl;
        // std::cout << "红外到可见光的旋转矩阵：\n" << R_ir_to_cam << std::endl;
        // std::cout << "红外到可见光的平移向量：\n" << T_ir_to_cam << std::endl;
    }
    ///////               
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}
////红外回调函数
void LIVMapper::ir_image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    if (!ir_enabled) return;
    cv::Mat heat_detection_map = ir_image_current.clone();
    int hot_points_in_frame = 0;
    ros::Time start_time = ros::Time::now();
    
    try {
        // 将ROS消息转换为OpenCV图像
        
        ir_frame_count++;
        
        // 新增：计算当前帧的温度统计
        float frame_max_temp = 0;
        float frame_avg_temp = 0;
        int hot_pixels = 0;
        int total_pixels = ir_image_current.rows * ir_image_current.cols;
        
        // 创建热力图可视化
        cv::Mat ir_visualization = ir_image_current.clone();
        //cv::imshow("heat picture:",ir_visualization);
        // 对热力图进行增强处理以突出热点区域
        for (int y = 0; y < ir_visualization.rows; y++) {
            for (int x = 0; x < ir_visualization.cols; x++) {
                cv::Vec3b color = ir_visualization.at<cv::Vec3b>(y, x);
                float temp_value = extract_temperature_from_color(color);
                frame_avg_temp += temp_value;
                
                if (temp_value > frame_max_temp) {
                    frame_max_temp = temp_value;
                }
                
                // 统计热区像素
                if (temp_value > ir_temperature_threshold) {
                    hot_pixels++;
                    
                    // 增强热区显示
                    float enhanced_factor = (temp_value - ir_temperature_threshold) / 
                                         (1.0 - ir_temperature_threshold);
                    enhanced_factor *= heat_enhancement_factor;  // 应用增强因子
                    
                    // 使红色更明显
                    ir_visualization.at<cv::Vec3b>(y, x)[0] = std::max(0, std::min(255, int(color[0] * (1.0 - enhanced_factor))));
                    ir_visualization.at<cv::Vec3b>(y, x)[1] = std::max(0, std::min(255, int(color[1] * (1.0 - enhanced_factor))));
                    ir_visualization.at<cv::Vec3b>(y, x)[2] = std::max(0, std::min(255, int(255 * enhanced_factor + color[2] * (1.0 - enhanced_factor))));
                }
            }
        }
        
        // 更新统计信息
        frame_avg_temp /= total_pixels;
        ir_hot_points_count += hot_pixels;
        if (frame_max_temp > ir_max_temp_value) {
            ir_max_temp_value = frame_max_temp;
        }
        
        // 显示统计信息
        std::stringstream ss;
        ss << "Max temp: " << std::fixed << std::setprecision(2) << frame_max_temp;
        cv::putText(ir_visualization, ss.str(), cv::Point(10, 20), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        ss.str("");
        ss << "Hot pixels: " << hot_pixels << " (" << std::fixed << std::setprecision(1) 
           << (hot_pixels * 100.0 / total_pixels) << "%)";
        cv::putText(ir_visualization, ss.str(), cv::Point(10, 40), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        ss.str("");
        ss << "Threshold: " << ir_temperature_threshold;
        cv::putText(ir_visualization, ss.str(), cv::Point(10, 60), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        
        // 发布处理后的红外图像用于调试
        if (publish_ir_image) {
            cv_bridge::CvImage out_msg;
            out_msg.header.stamp = msg->header.stamp;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = ir_visualization;
            pub_ir_image.publish(out_msg.toImageMsg());
        }
        
        // 定期发布统计信息
        ros::Time current_time = ros::Time::now();
        if ((current_time - ir_last_stats_time).toSec() > 1.0) {  // 每秒更新一次统计
            std_msgs::String stats_msg;
            std::stringstream stats_ss;
            
            stats_ss << "ir information:\n"
                     << "  frequent: " << ir_frame_count / (current_time - ir_last_stats_time).toSec() << " fps\n"
                     << "  heat ratio: " << (ir_hot_points_count * 100.0 / (ir_frame_count * total_pixels)) << "%\n"
                     << "  max temperture: " << ir_max_temp_value << "\n"
                     << "  average deal time: " << (ir_process_time_total * 1000 / ir_frame_count) << " ms\n";
                     
            // 添加参数调整建议
            if (ir_hot_points_count < 100) {
                stats_ss << "  lower temperture (temperature_threshold),now: " << ir_temperature_threshold << "\n";
            } else if (ir_hot_points_count > ir_frame_count * total_pixels * 0.4) {
                stats_ss << "  higher temperture (temperature_threshold), now : " << ir_temperature_threshold << "\n";
            }
            
            if (ir_max_temp_value < 0.7) {
                stats_ss << "  higher radio (heat_enhancement_factor),now: " << heat_enhancement_factor << "\n";
            }
            
            stats_msg.data = stats_ss.str();
            pub_ir_stats.publish(stats_msg);
            
            // 打印部分统计信息到控制台
            if (ir_frame_count % ir_debug_frequency == 0) {
                ROS_INFO("\n%s", stats_msg.data.c_str());
            }
            
            // 重置统计
            ir_frame_count = 0;
            ir_hot_points_count = 0;
            ir_process_time_total = 0;
            ir_max_temp_value = 0;
            ir_last_stats_time = current_time;
        }
        
        ir_data_ready = true;  // 标记红外数据已就绪
        
        // 输出调试信息（每隔一定帧数）
        if (ir_frame_count % ir_debug_frequency == 0) {
            ROS_DEBUG("ir image:size=%dx%d,heat pixel=%d,max temperture=%.2f", 
                    ir_image_current.cols, ir_image_current.rows, hot_pixels, frame_max_temp);
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("error ir image: %s", e.what());
    }
    
    // 计算处理时间
    ros::Time end_time = ros::Time::now();
    double process_duration = (end_time - start_time).toSec();
    ir_process_time_total += process_duration;
}
////归一化得到温度，也就是计算温度
float LIVMapper::extract_temperature_from_color(const cv::Vec3b &color)
{
    float heat_value = (color[2] * 0.7f + color[1] * 0.2f + color[0] * 0.1f) / 255.0f;
    
    // 类型统一，两边都用float
    return std::max(float(ir_temperature_threshold) + 0.1f, heat_value);
}
///HSv颜色转化
void LIVMapper::rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, float &h, float &s, float &v) 
{
    float rf = r / 255.0f;
    float gf = g / 255.0f;
    float bf = b / 255.0f;
    
    float cmax = std::max(rf, std::max(gf, bf));
    float cmin = std::min(rf, std::min(gf, bf));
    float delta = cmax - cmin;
    
    // 计算色调H
    if (delta < 0.00001f) {
        h = 0; // 灰色没有色调
    } else if (cmax == rf) {
        h = 60.0f * fmodf((gf - bf) / delta, 6.0f);
    } else if (cmax == gf) {
        h = 60.0f * ((bf - rf) / delta + 2.0f);
    } else {
        h = 60.0f * ((rf - gf) / delta + 4.0f);
    }
    
    if (h < 0) h += 360.0f;
    
    // 计算饱和度S
    s = (cmax < 0.00001f) ? 0 : (delta / cmax);
    
    // 计算明度V
    v = cmax;
}

void LIVMapper::hsv_to_rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) 
{
    float c = v * s;
    float x = c * (1 - fabsf(fmodf(h / 60.0f, 2) - 1));
    float m = v - c;
    
    float rf, gf, bf;
    
    if (h >= 0 && h < 60) {
        rf = c; gf = x; bf = 0;
    } else if (h >= 60 && h < 120) {
        rf = x; gf = c; bf = 0;
    } else if (h >= 120 && h < 180) {
        rf = 0; gf = c; bf = x;
    } else if (h >= 180 && h < 240) {
        rf = 0; gf = x; bf = c;
    } else if (h >= 240 && h < 300) {
        rf = x; gf = 0; bf = c;
    } else {
        rf = c; gf = 0; bf = x;
    }
    
    r = static_cast<uint8_t>(round((rf + m) * 255));
    g = static_cast<uint8_t>(round((gf + m) * 255));
    b = static_cast<uint8_t>(round((bf + m) * 255));
}
void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
   
    // 添加红外相机订阅器
    if (ir_enabled) {
        sub_ir = nh.subscribe("/rectify_hk", 10, &LIVMapper::ir_image_callback, this);
        pub_ir_image = it.advertise("/ir_processed", 1);
    pub_ir_stats = nh.advertise<std_msgs::String>("/ir_stats", 1);
    ROS_INFO("get ir topic : /rectify_hk,deal ir topic pub: /ir_processed, /ir_stats");
    ir_frame_count = 0;
    ir_hot_points_count = 0;
    ir_process_time_total = 0;
    ir_max_temp_value = 0;
    ir_last_stats_time = ros::Time::now();  
  }

}
void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;

  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  sensor_msgs::PointCloud2 laserCloudmsg;

  if (img_en)
  {
      static int pub_num = 1;
      static int ir_fusion_count = 0;
      static int point_count = 0;
      static int ir_applied_count = 0;
      
      *pcl_wait_pub += *pcl_w_wait_pub;
      if (pub_num == pub_scan_num)
      {
          ros::Time fusion_start = ros::Time::now();
          pub_num = 1;
          size_t size = pcl_wait_pub->points.size();
          laserCloudWorldRGB->reserve(size);
          cv::Mat img_rgb = vio_manager->img_rgb;
          
          // 检查红外数据是否准备好
          bool use_ir = ir_enabled && ir_data_ready && !ir_image_current.empty();
          
          // if (use_ir) {
          //     ROS_INFO("cloud fusion ready (point size: %zu)...", size);
          //     pointRGB.r = 255;  // 强制红色分量最大化
          //     pointRGB.g = pixel[1] * 0.5;  // 减少绿色
          //     pointRGB.b = pixel[0] * 0.5;  // 减少蓝色
          //     ir_fusion_count++;
          // }
          
          int ir_points_processed = 0;
          point_count = 0;
          ir_applied_count = 0;
          
          for (size_t i = 0; i < size; i++)
          {
              PointTypeRGB pointRGB;
              pointRGB.x = pcl_wait_pub->points[i].x;
              pointRGB.y = pcl_wait_pub->points[i].y;
              pointRGB.z = pcl_wait_pub->points[i].z;

              V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
              V3D pf(vio_manager->new_frame_->w2f(p_w));
              if (pf[2] < 0) continue;
              V2D pc(vio_manager->new_frame_->w2c(p_w));

              if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3))
              {
                  point_count++;
                  V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
                  //pointRGB.r = pixel[2];
                  pointRGB.r = 255;
                  pointRGB.g = pixel[1]*0.6;
                  pointRGB.b = pixel[0]*0.6;

                  if (use_ir)
                  {
                      ir_points_processed++;
                      Eigen::Vector3d p_cam(pf[0], pf[1], pf[2]);
                      Eigen::Vector3d p_ir = R_ir_to_cam * p_cam + T_ir_to_cam;
                      if (p_ir[2] > 0)
                      {
                          Eigen::Vector3d p_ir_homo = K_ir * (p_ir / p_ir[2]);
                          int ir_x = round(p_ir_homo[0]);
                          int ir_y = round(p_ir_homo[1]);
                          
                          // 改进：添加边界检查和插值获取更精确的红外像素值
                          if (ir_x >= 0 && ir_x < ir_image_current.cols - 1 &&
                              ir_y >= 0 && ir_y < ir_image_current.rows - 1)
                          {
                              // 获取周围4个像素点进行双线性插值
                              float dx = p_ir_homo[0] - ir_x;
                              float dy = p_ir_homo[1] - ir_y;
                              
                              cv::Vec3b ir_pixel_tl = ir_image_current.at<cv::Vec3b>(ir_y, ir_x);
                              cv::Vec3b ir_pixel_tr = ir_image_current.at<cv::Vec3b>(ir_y, ir_x+1);
                              cv::Vec3b ir_pixel_bl = ir_image_current.at<cv::Vec3b>(ir_y+1, ir_x);
                              cv::Vec3b ir_pixel_br = ir_image_current.at<cv::Vec3b>(ir_y+1, ir_x+1);
                              
                              // 插值计算温度值
                              float temp_tl = extract_temperature_from_color(ir_pixel_tl);
                              float temp_tr = extract_temperature_from_color(ir_pixel_tr);
                              float temp_bl = extract_temperature_from_color(ir_pixel_bl);
                              float temp_br = extract_temperature_from_color(ir_pixel_br);
                              
                              float normalized_temp = temp_tl * (1-dx) * (1-dy) + 
                                                    temp_tr * dx * (1-dy) + 
                                                    temp_bl * (1-dx) * dy + 
                                                    temp_br * dx * dy;
                              
                              // 强化高温区域的显示效果
                              if (normalized_temp > ir_temperature_threshold)
                              {
                                  ir_applied_count++;
                                  if (ir_integration_method == 0)
                                  {
                                      // 方法1：基于HSV的着色方法，保留亮度但改变色调
                                      float h, s, v;
                                      rgb_to_hsv(pointRGB.r, pointRGB.g, pointRGB.b, h, s, v);
                                      
                                      // 根据温度调整色调（越热越红）
                                      float factor = (normalized_temp - ir_temperature_threshold) / 
                                                    (1.0 - ir_temperature_threshold);
                                      factor *= heat_enhancement_factor;  // 应用增强因子
                                      
                                      h = h * (1.0f - factor) + 0.0f * factor;  // 向红色(0度)偏移
                                      s = s * (1.0f - factor) + 1.0f * factor;  // 增加饱和度
                                      
                                      hsv_to_rgb(h, s, v, pointRGB.r, pointRGB.g, pointRGB.b);
                                  }
                                  else
                                  {
                                      // 方法2：直接混合RGB值
                                      // 高温区域更加倾向于红色
                                      float factor = (normalized_temp - ir_temperature_threshold) / 
                                                   (1.0 - ir_temperature_threshold);
                                      factor *= ir_alpha * heat_enhancement_factor;  // 应用混合系数和增强因子
                                      
                                      // 增强红色分量，减弱蓝色分量
                                      pointRGB.r = std::min(255, int(pointRGB.r * (1.0f - factor) + 255 * factor));
                                      pointRGB.b = std::max(0, int(pointRGB.b * (1.0f - factor)));
                                  }
                              }
                          }
                      }
                  }
                  if (pf.norm() > blind_rgb_points)
                      laserCloudWorldRGB->push_back(pointRGB);
              }
          }
          
          // 输出红外融合统计信息
          if (use_ir && ir_fusion_count % ir_debug_frequency == 0) {
              ros::Time fusion_end = ros::Time::now();
              double fusion_duration = (fusion_end - fusion_start).toSec();
              
              ROS_INFO("about ir input:\n"
                      "  point size: %d \n"
                      "  point in field of view: %d \n"
                      "  ir treatment point: %d \n"
                      "  ir application point %d (%.1f%%)\n"
                      "  together use time: %.2f ms",
                      (int)size, point_count, ir_points_processed, ir_applied_count,
                      ir_points_processed > 0 ? (ir_applied_count * 100.0 / ir_points_processed) : 0.0,
                      fusion_duration * 1000);
              
              // 提供参数调整建议
              if (ir_applied_count < 10) {
                  ROS_WARN("not enough heat point ,wrong number：\n"
                          "  1. lower temperture (temperature_threshold)，now: %.2f\n"
                          "  2. the ir matrix wrong",
                          ir_temperature_threshold);
              }
              else if (ir_applied_count > ir_points_processed * 0.5) {
                  ROS_WARN("heat too high,wrong number：\n"
                          "  1. higher temperture (temperature_threshold)，now: %.2f\n"
                          "  2. lower factor (heat_enhancement_factor)，now: %.2f",
                          ir_temperature_threshold, heat_enhancement_factor);
              }
          }
          
          pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
      }
      else
      {
          pub_num++;
      }
  }
  else
  {
      pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
  }

  laserCloudmsg.header.stamp = ros::Time::now();
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes.publish(laserCloudmsg);

    /*** Save map ***/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        static int scan_wait_num = 0;

        if (img_en)
        {
            *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
            *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        scan_wait_num++;

        if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            if (img_en)
            {
                pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
                PointCloudXYZRGB().swap(*pcl_wait_save);
            }
            else
            {
                pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
                PointCloudXYZI().swap(*pcl_wait_save_intensity);
            }
            Eigen::Quaterniond q(_state.rot_end);
            fout_pcd_pos << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                         << " " << q.z() << " " << endl;
            scan_wait_num = 0;
        }
    }

    if (laserCloudWorldRGB->size() > 0)
        PointCloudXYZI().swap(*pcl_wait_pub);
    PointCloudXYZI().swap(*pcl_w_wait_pub);
}
