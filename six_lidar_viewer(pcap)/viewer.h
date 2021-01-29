#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <fstream>
#include <sstream>


// Include VelodyneCapture Header
#include "VelodyneCapture.h"

bool roll_enable(false);
bool pitch_enable(false);
bool yaw_enable(false);
float roll_offset = 0.0;
float pitch_offset = 0.0;
float yaw_offset = 0.0;

int center_front_lidar_flag = 1;
int left_front_lidar_flag = 1;
int right_front_lidar_flag = 1;
int left_side_lidar_flag = 1;
int right_side_lidar_flag = 1;
int center_rear_lidar_flag = 1;
int virtual_scan_flag = 0;
int viewer_pause = 0;


void KeyboardViz3d(const cv::viz::KeyboardEvent &w, void *t)
{
    cv::viz::Viz3d *viewer=(cv::viz::Viz3d*)t;
    if (w.action) {
        std::cout << "you pressed "<< w.code<<" = "<<w.symbol<< " in viz window "<<viewer->getWindowName()<<"\n";

        // remove previous point clouds drawing to keep only the enabled ones
        //viewer->removeAllWidgets();

        switch (w.code) {
            case '1':
                center_front_lidar_flag = !center_front_lidar_flag;
                std::cout << "center_front_lidar_flag " << center_front_lidar_flag << "\n";
                break;
            case '2':
                left_front_lidar_flag = !left_front_lidar_flag;
                std::cout << "left_front_lidar_flag " << left_front_lidar_flag << "\n";
                break;
            case '3':
                right_front_lidar_flag = !right_front_lidar_flag;
                std::cout << "right_front_lidar_flag " << right_front_lidar_flag << "\n";
                break;
            case '4':
                left_side_lidar_flag = !left_side_lidar_flag;
                std::cout << "left_side_lidar_flag " << left_side_lidar_flag << "\n";
                break;
            case '5':
                right_side_lidar_flag = !right_side_lidar_flag;
                std::cout << "right_side_lidar_flag " << right_side_lidar_flag << "\n";
                break;
            case '6':
                center_rear_lidar_flag = !center_rear_lidar_flag;
                std::cout << "center_rear_lidar_flag " << center_rear_lidar_flag << "\n";
                break;
            case '-':
                //roll_offset += -0.1;
                //std::cout << "roll_offset " << roll_offset << "\n";
                pitch_offset += -0.1;
                std::cout << "pitch_offset " << pitch_offset << "\n";
                //yaw_offset += -0.1;
                //std::cout << "yaw_offset " << yaw_offset << "\n";
                break;
            case '+':
                //roll_offset += 0.1;
                //std::cout << "roll_offset " << roll_offset << "\n";
                pitch_offset += 0.1;
                std::cout << "pitch_offset " << pitch_offset << "\n";
                //yaw_offset += 0.1;
                //std::cout << "yaw_offset " << yaw_offset << "\n";
                break;
            case 'a':
                center_front_lidar_flag = 1;
                left_front_lidar_flag = 1;
                right_front_lidar_flag = 1;
                left_side_lidar_flag = 1;
                right_side_lidar_flag = 1;
                center_rear_lidar_flag = 1;
                std::cout << "Enable all Lidars " << "\n";
                break;
            case 'q':
                viewer->close();
                std::cout << "Quit " << "\n";
                break;
            case 'p':
                viewer_pause = !viewer_pause;
                std::cout << "pause " << viewer_pause << "\n";
                break;
            case 'v':
                virtual_scan_flag = !virtual_scan_flag;
                std::cout << "virtual_scan_flag " << virtual_scan_flag << "\n";
                break;
        }
    }

}


namespace LidarViewer
{
void cvViz3dCallbackSetting(cv::viz::Viz3d &viewer, bool &pause)
{
    viewer.registerKeyboardCallback(KeyboardViz3d, &viewer);

    /*viewer.registerKeyboardCallback(
        []( const cv::viz::KeyboardEvent& event, void* cookie )
        {

          std::cout << "you pressed "<< event.code<<" = "<<event.symbol<<"\n";

          // remove previous point clouds drawing to keep only the enabled ones
          static_cast<cv::viz::Viz3d*>( cookie )->removeAllWidgets();

          // Close Viewer
          if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN )
          {
              static_cast<cv::viz::Viz3d*>( cookie )->close();
          }

          switch (event.code) {
              case '1':
                  center_front_lidar_flag = !center_front_lidar_flag;
                  std::cout << "center_front_lidar_flag " << center_front_lidar_flag << "\n";
                  break;
              case '2':
                  left_front_lidar_flag = !left_front_lidar_flag;
                  std::cout << "left_front_lidar_flag " << left_front_lidar_flag << "\n";
                  break;
              case '3':
                  right_front_lidar_flag = !right_front_lidar_flag;
                  std::cout << "right_front_lidar_flag " << right_front_lidar_flag << "\n";
                  break;
              case '4':
                  left_side_lidar_flag = !left_side_lidar_flag;
                  std::cout << "left_side_lidar_flag " << left_side_lidar_flag << "\n";
                  break;
              case '5':
                  right_side_lidar_flag = !right_side_lidar_flag;
                  std::cout << "right_side_lidar_flag " << right_side_lidar_flag << "\n";
                  break;
              case '6':
                  center_rear_lidar_flag = !center_rear_lidar_flag;
                  std::cout << "center_rear_lidar_flag " << center_rear_lidar_flag << "\n";
                  break;
              case 'a':
                  center_front_lidar_flag = 1;
                  left_front_lidar_flag = 1;
                  right_front_lidar_flag = 1;
                  left_side_lidar_flag = 1;
                  right_side_lidar_flag = 1;
                  center_rear_lidar_flag = 1;
                  std::cout << "Enable all Lidars " << "\n";
                  break;
            }

        }
        , &viewer);
    */

    /*viewer.registerKeyboardCallback(
        []( const cv::viz::KeyboardEvent& event, void* pause )
        {
          // Switch state of pause / resume when pressing s
          if( event.code == 'p' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN  )
          {
              bool* p = static_cast<bool*>( pause );
              *p = !(*p);
          }
        }
        , &pause);*/


}

void update_viewer(std::vector<std::vector<cv::Vec3f>> &buffers, std::vector<std::vector<bool>> &results, cv::viz::Viz3d &viewer) {
    // if (buffers[0].empty()) {return;}
    cv::viz::WCloudCollection collection;
    std::vector<cv::Vec3f> curbsBuffer;
    for (int i = 0; i < buffers.size(); i++) {
        int idx = 0;
        for (int j = 0; j < buffers[i].size(); j++) {
            if (results[i][j]) {
                curbsBuffer.push_back(buffers[i][j]);
            } else {
                buffers[i][idx++] = buffers[i][j];
            }
        }
        buffers[i].resize(idx);
    }
    buffers.push_back(curbsBuffer);

    for (int i = 0; i < buffers.size(); i++) {
        cv::Mat cloudMat = cv::Mat(static_cast<int>(buffers[i].size()), 1, CV_32FC3, &buffers[i][0]);
        if (i == buffers.size()-1) {
            collection.addCloud(cloudMat, cv::viz::Color::red());
        }
        else {
            collection.addCloud(cloudMat, cv::viz::Color::white());
        }
    }
    viewer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(2));
    viewer.showWidget("Cloud", collection);
    viewer.setRenderingProperty("Cloud", cv::viz::POINT_SIZE, 2.0);  // Set point size of the point cloud
    viewer.setBackgroundColor(cv::viz::Color::mlab());
    viewer.spinOnce();
}

void updateViewerFromBuffers(std::vector<std::vector<cv::Vec3f>> &buffers, std::vector<std::vector<bool>> &results, cv::viz::Viz3d &viewer, std::vector<std::vector<float>> &vscanRes, std::vector<cv::viz::WPolyLine> polyLine)
{
    // if (buffers[0].empty()) {return;}
    viewer.removeAllWidgets();
    cv::viz::WCloudCollection collection;
    std::vector<cv::Vec3f> curbsBuffer;
    for (int i = 0; i < buffers.size(); i++)
    {
        int idx = 0;
        for (int j = 0; j < buffers[i].size(); j++)
        {
            if (results[i][j])
            {
                curbsBuffer.push_back(buffers[i][j]);
            } else
            {
                buffers[i][idx++] = buffers[i][j];
            }
        }
        buffers[i].resize(idx);
    }
    buffers.push_back(curbsBuffer);

    if (virtual_scan_flag) {
        std::vector<cv::viz::WLine> lines;
        int cnt = 0;
        for (auto &res : vscanRes)
        {
            cv::viz::WLine line(cv::Point3f(res[0], res[1], res[2]), cv::Point3f(res[0], res[1], res[3]), cv::viz::Color::green());
            line.setRenderingProperty(cv::viz::LINE_WIDTH, 5.0);
            viewer.showWidget("Line Widget"+std::to_string(cnt++), line);
        }
    }

    for (int i = 0; i < buffers.size(); i++)
    {
        cv::Mat cloudMat = cv::Mat(static_cast<int>(buffers[i].size()), 1, CV_32FC3, &buffers[i][0]);
        if (i == buffers.size()-1)
        {
            collection.addCloud(cloudMat, cv::viz::Color::red());
        }
        else
        {
            if(i == 0 && center_front_lidar_flag)
                collection.addCloud(cloudMat, cv::viz::Color::yellow());
            if(i == 1 && center_rear_lidar_flag)
                collection.addCloud(cloudMat, cv::viz::Color::yellow());
            if(i == 2 && left_front_lidar_flag)
                collection.addCloud(cloudMat, cv::viz::Color::red());
            if(i == 3 && left_side_lidar_flag)
                collection.addCloud(cloudMat, cv::viz::Color::white());
            if(i == 4 && right_front_lidar_flag)
                collection.addCloud(cloudMat, cv::viz::Color::blue());
            if(i == 5 && right_side_lidar_flag)
                collection.addCloud(cloudMat, cv::viz::Color::white());
        }
    }
    if (polyLine.size() == 2) {
        viewer.showWidget("Poly Left", polyLine[0]);
        viewer.showWidget("Poly Right", polyLine[1]);
    }

    #if 0 // draw ground plane
    cv::Size2d size = cv::Size2d(20.0, 40.0);
    cv::Point3d front_lidar_z_point = cv::Point3d(0.00f, 20.00f, -0.33f - 0.45);
    cv::viz::WPlane plane(front_lidar_z_point, cv::Vec3d(0,0,1), cv::Vec3d(0,1,0) ,size, cv::viz::Color::black());
    viewer.showWidget("Plane Widget", plane);
    #endif

    viewer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(2));
    viewer.showWidget("Cloud", collection);
    viewer.setRenderingProperty("Cloud", cv::viz::POINT_SIZE, 2.0);  // Set point size of the point cloud
    viewer.setBackgroundColor(cv::viz::Color::mlab());
    viewer.spinOnce();
}


void updateViewerFromBuffers(std::vector<std::vector<cv::Vec3f>> &buffers, std::vector<std::vector<int>> &results, cv::viz::Viz3d &viewer, std::vector<std::vector<float>> &vscanRes)
{
    // if (buffers[0].empty()) {return;}
    cv::viz::WCloudCollection collection;

    std::vector<cv::Vec3f> curbsBuffer_1;
    std::vector<cv::Vec3f> curbsBuffer_2;
    for (int i = 0; i < buffers.size(); i++)
    {
        int idx = 0;
        for (int j = 0; j < buffers[i].size(); j++)
        {
            if (results[i][j] == 1)
            {
                curbsBuffer_1.push_back(buffers[i][j]);
            }
            else if (results[i][j] == -1)
            {
                curbsBuffer_2.push_back(buffers[i][j]);
            }
            else
            {
                buffers[i][idx++] = buffers[i][j];
            }
        }
        buffers[i].resize(idx);
    }
    buffers.push_back(curbsBuffer_1);
    buffers.push_back(curbsBuffer_2);

    for (auto &res : vscanRes)
    {
        std::cout << res[0] << " " << res[1] << " " << res[2] << " " << res[3] << std::endl;
        cv::viz::WLine line(cv::Point3f(res[0], res[1], res[2]), cv::Point3f(res[0], res[1], res[3]), cv::viz::Color::green());
        line.setRenderingProperty(cv::viz::LINE_WIDTH, 5.0);
        viewer.showWidget("Line Widget", line);
    }

    for (int i = 0; i < buffers.size(); i++)
    {
        cv::Mat cloudMat = cv::Mat(static_cast<int>(buffers[i].size()), 1, CV_32FC3, &buffers[i][0]);
        if (i == buffers.size()-1)
        {
            collection.addCloud(cloudMat, cv::viz::Color::red());
        }
        else if (i == buffers.size()-2)
        {
            collection.addCloud(cloudMat, cv::viz::Color::green());
        }
        else
        {
            collection.addCloud(cloudMat, cv::viz::Color::white());
        }
    }

    viewer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(2));
    viewer.showWidget("Cloud", collection);
    viewer.setRenderingProperty("Cloud", cv::viz::POINT_SIZE, 2.0);  // Set point size of the point cloud
    viewer.setBackgroundColor(cv::viz::Color::mlab());
    viewer.spinOnce();
}

void updateViewerFromSingleBuffer(std::vector<cv::Vec3f> &buffer, cv::viz::Viz3d &viewer)
{
    cv::Mat cloudMat = cv::Mat(static_cast<int>(buffer.size()), 1, CV_32FC3, &buffer[0]);
    cv::viz::WCloud cloud( cloudMat );
    viewer.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(2));
    viewer.showWidget("Cloud", cloud);
    viewer.spinOnce();
}

void laser_to_cartesian(std::vector<velodyne::Laser> &lasers, std::vector<std::vector<float>> &pointcloud, float theta, cv::Mat &rot, cv::Mat &trans) {
    pointcloud.clear();
    pointcloud.resize(lasers.size());
    int idx = 0;
    for (int i = 0; i < lasers.size(); i++) {
        //double azimuth_rot = lasers[i].azimuth + theta;
        double azimuth_rot = lasers[i].azimuth;
        if (azimuth_rot >= 360.0) {
            azimuth_rot -= 360.0;
        }
        else if (azimuth_rot < 0.0) {
            azimuth_rot += 360.0;
        }
        const double distance = static_cast<double>( lasers[i].distance );
        const double azimuth  = azimuth_rot  * CV_PI / 180.0;
        const double vertical = lasers[i].vertical * CV_PI / 180.0;
        float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
        float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
        float z = static_cast<float>( ( distance * std::sin( vertical ) ) );

        if( x == 0.0f && y == 0.0f && z == 0.0f ) continue;

        x /= 100.0, y /= 100.0, z /= 100.0;
        float intensity = static_cast<float>(lasers[i].intensity);
        float ring = static_cast<float>(lasers[i].id);
        float dist = std::sqrt(x * x + y * y + z * z);
        if (dist < 0.9f) continue;

        // testing : front LiDAR
        //if (azimuth_rot >= 130.0 && azimuth_rot <= 260.0) continue;

        pointcloud[idx] = {x, y, z, intensity, ring, dist, static_cast<float>(azimuth_rot)}; // Write to pointcloud
        idx++;
    }
    pointcloud.resize(idx);
}

void pushToBuffer(std::vector<cv::Vec3f> &buffer, const std::vector<std::vector<float>> &pointcloud)
{
    buffer.resize(pointcloud.size());
    for (int i = 0; i < pointcloud.size(); i++)
    {
        buffer[i] = cv::Vec3f( pointcloud[i][0], pointcloud[i][1], pointcloud[i][2] );
    }
}

void push_result_to_buffer(std::vector<cv::Vec3f> &buffer, const std::vector<std::vector<float>> &pointcloud, cv::Mat &rot, cv::Mat &trans)
{
    buffer.resize(pointcloud.size());
    for (int i = 0; i < pointcloud.size(); i++)
    {
        buffer[i] = cv::Vec3f( pointcloud[i][0], pointcloud[i][1], pointcloud[i][2] );
        cv::Mat p(buffer[i]);
        p = rot * p; // Rotation
        float temp_x = p.at<float>(0,0);
        float temp_y = p.at<float>(1,0);
        float temp_z = p.at<float>(2,0);
        buffer[i][0] = temp_y + trans.at<float>(0,0);
        buffer[i][1] = temp_x + trans.at<float>(1,0);
        //buffer[i][2] = -buffer[i][2] + trans.at<float>(2,0);
        buffer[i][2] = -temp_z + trans.at<float>(2,0);
    //files[0] = "/home/rtml/lidar_view_work/one_lidar_fov_1.pcap";
        float tmp_x = buffer[i][0];
        buffer[i][0] = buffer[i][1];
        buffer[i][1] = tmp_x;
        buffer[i][2] = -buffer[i][2] - 0.5; // adjust sensor height
    }
}
}

namespace SensorConfig
{
std::vector<std::string> getPcapFiles()
{
    std::vector<std::string> files(6);

    //files[0] = "/home/rtml/lidar_view_work/snow_data/snow_no_traffic_1.pcap";
    //files[0] = "/home/rtml/lidar_view_work/one_lidar_fov_1.pcap";

    /*files[0] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_2/mil19_road_2019.9.13_center_front_2.pcap";
    files[1] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_2/mil19_road_2019.9.13_center_rear_2.pcap";
    files[2] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_2/mil19_road_2019.9.13_left_front_2.pcap";
    files[3] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_2/mil19_road_2019.9.13_left_side_2.pcap";
    files[4] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_2/mil19_road_2019.9.13_right_front_2.pcap";
    files[5] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_2/mil19_road_2019.9.13_right_side_2.pcap";*/


    /*files[0] = "/home/rtml/lidar_view_work/GM_Test/gm_test_center_front.pcap";
    files[1] = "/home/rtml/lidar_view_work/GM_Test/gm_test_center_rear.pcap";
    files[2] = "/home/rtml/lidar_view_work/GM_Test/gm_test_left_front.pcap";
    files[3] = "/home/rtml/lidar_view_work/GM_Test/gm_test_left_side.pcap";
    files[4] = "/home/rtml/lidar_view_work/GM_Test/gm_test_right_front.pcap";
    files[5] = "/home/rtml/lidar_view_work/GM_Test/gm_test_right_side.pcap";
    */

    /*files[0] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_5/mil19_road_2019.9.13_center_front_5.pcap";
    files[1] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_5/mil19_road_2019.9.13_center_rear_5.pcap";
    files[2] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_5/mil19_road_2019.9.13_left_front_5.pcap";
    files[3] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_5/mil19_road_2019.9.13_left_side_5.pcap";
    files[4] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_5/mil19_road_2019.9.13_right_front_5.pcap";
    files[5] = "/home/rtml/lidar_view_work/mil19_road_2019.9.13_5/mil19_road_2019.9.13_right_side_5.pcap";*/

    /*files[0] = "/home/rtml/lidar_view_work/060320_indoor/center_front_060320.pcap";
    files[1] = "/home/rtml/lidar_view_work/060320_indoor/center_rear_060320.pcap";
    files[2] = "/home/rtml/lidar_view_work/060320_indoor/left_front_060320.pcap";
    files[3] = "/home/rtml/lidar_view_work/060320_indoor/left_side_060320.pcap";
    files[4] = "/home/rtml/lidar_view_work/060320_indoor/right_front_060320.pcap";
    files[5] = "/home/rtml/lidar_view_work/060320_indoor/right_side_060320.pcap";*/

    files[0] = "/home/rtml/lidar_view_work/road_test_July09_2020/center_front_070920_set5.pcap";
    files[1] = "/home/rtml/lidar_view_work/road_test_July09_2020/center_rear_070920_set5.pcap";
    files[2] = "/home/rtml/lidar_view_work/road_test_July09_2020/left_front_070920_set5.pcap";
    files[3] = "/home/rtml/lidar_view_work/road_test_July09_2020/left_side_070920_set5.pcap";
    files[4] = "/home/rtml/lidar_view_work/road_test_July09_2020/right_front_070920_set5.pcap";
    files[5] = "/home/rtml/lidar_view_work/road_test_July09_2020/right_side_070920_set5.pcap";

    /*files[0] = "/home/rtml/lidar_view_work/mil19_snowstorm/snow_storm_mil19_dec_16_2_center_front.pcap";
    files[1] = "/home/rtml/lidar_view_work/mil19_snowstorm/snow_storm_mil19_dec_16_2_center_rear.pcap";
    files[2] = "/home/rtml/lidar_view_work/mil19_snowstorm/snow_storm_mil19_dec_16_2_center_rear.pcap";
    files[3] = "/home/rtml/lidar_view_work/mil19_snowstorm/snow_storm_mil19_dec_16_2_left_side.pcap";
    files[4] = "/home/rtml/lidar_view_work/mil19_snowstorm/snow_storm_mil19_dec_16_2_right_front.pcap";
    files[5] = "/home/rtml/lidar_view_work/mil19_snowstorm/snow_storm_mil19_dec_16_2_right_side.pcap";*/

    // files[0] = "/home/rtml/LiDAR_camera_calibration_work/data/mil19_indoor_2/mil19_indoor_2_front_center.pcap";
    // files[1] = "/home/rtml/LiDAR_camera_calibration_work/data/mil19_indoor_2/mil19_indoor_2_rear_center.pcap";
    // files[2] = "/home/rtml/LiDAR_camera_calibration_work/data/mil19_indoor_2/mil19_indoor_2_front_left.pcap";
    // files[3] = "/home/rtml/LiDAR_camera_calibration_work/data/mil19_indoor_2/mil19_indoor_2_side_left.pcap";
    // files[4] = "/home/rtml/LiDAR_camera_calibration_work/data/mil19_indoor_2/mil19_indoor_2_front_right.pcap";
    // files[5] = "/home/rtml/LiDAR_camera_calibration_work/data/mil19_indoor_2/mil19_indoor_2_side_right.pcap";


    /*files[0] = "/home/rtml/lidar_view_work/mil19_indoor_1/mil19_indoor_1_front_center.pcap";
    files[1] = "/home/rtml/lidar_view_work/mil19_indoor_1/mil19_indoor_1_rear_center.pcap";
    files[2] = "/home/rtml/lidar_view_work/mil19_indoor_1/mil19_indoor_1_front_left.pcap";
    files[3] = "/home/rtml/lidar_view_work/mil19_indoor_1/mil19_indoor_1_side_left.pcap";
    files[4] = "/home/rtml/lidar_view_work/mil19_indoor_1/mil19_indoor_1_front_right.pcap";
    files[5] = "/home/rtml/lidar_view_work/mil19_indoor_1/mil19_indoor_1_side_right.pcap";*/

    return files;
}

std::string getBinaryFile(int frame_idx, std::string root_dir="autoware-20190828124709/")
{
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(10) << frame_idx;
    std::string filename = "/home/rtml/LiDAR_camera_calibration_work/data/data_raw/synced/" + root_dir + "velodyne_points/data/" + ss.str() + ".bin";
    return filename;
}

std::vector<float> getRotationParams()
{
    std::vector<float> rot_params;
    rot_params.push_back(-14.0f);  // center front
    rot_params.push_back(-119.0f);  // center rear
    rot_params.push_back(-43.65f);  // driver front (left front)
    rot_params.push_back(-125.0f); // driver side (left side)
    rot_params.push_back(38.5f);   // passenger front (right front)
    rot_params.push_back(40.0f);  // passenger side (right side)
    return rot_params;
}

cv::Mat getRotationMatrixFromTheta_X(float theta_deg)
{
    cv::Mat rot = cv::Mat::zeros(3, 3, CV_32FC1);
    float theta_rad = -theta_deg * CV_PI / 180.;  // Notice the sign of theta!
    rot.at<float>(0,0) = 1.0f;
    rot.at<float>(1,0) = 0.0f;
    rot.at<float>(2,0) = 0.0f;
    rot.at<float>(0,1) = 0.0f;
    rot.at<float>(1,1) = std::cos(theta_rad);
    rot.at<float>(2,1) = std::sin(theta_rad);
    rot.at<float>(0,2) = 0.0f;
    rot.at<float>(1,2) = -std::sin(theta_rad);
    rot.at<float>(2,2) = std::cos(theta_rad);
    return rot;
}

cv::Mat getRotationMatrixFromTheta_Y(float theta_deg)
{
    cv::Mat rot = cv::Mat::zeros(3, 3, CV_32FC1);
    float theta_rad = -theta_deg * CV_PI / 180.;  // Notice the sign of theta!
    rot.at<float>(0,0) = std::cos(theta_rad);
    rot.at<float>(1,0) = 0.0f;
    rot.at<float>(2,0) = -std::sin(theta_rad);
    rot.at<float>(0,1) = 0.0f;
    rot.at<float>(1,1) = 1.0f;
    rot.at<float>(2,1) = 0.0f;
    rot.at<float>(0,2) = std::sin(theta_rad);
    rot.at<float>(1,2) = 0.0f;
    rot.at<float>(2,2) = std::cos(theta_rad);
    return rot;
}

cv::Mat getRotationMatrixFromTheta_Z(float theta_deg)
{
    cv::Mat rot = cv::Mat::zeros(3, 3, CV_32FC1);
    float theta_rad = -theta_deg * CV_PI / 180.;  // Notice the sign of theta!
    rot.at<float>(0,0) = std::cos(theta_rad);
    rot.at<float>(1,0) = std::sin(theta_rad);
    rot.at<float>(2,0) = 0.0f;
    rot.at<float>(0,1) = -std::sin(theta_rad);
    rot.at<float>(1,1) = std::cos(theta_rad);
    rot.at<float>(2,1) = 0.0f;
    rot.at<float>(0,2) = 0.0f;
    rot.at<float>(1,2) = 0.0f;
    rot.at<float>(2,2) = 1.0f;
    return rot;
}

cv::Mat getEye()
{
    cv::Mat rot = cv::Mat::zeros(3, 3, CV_32FC1);
    rot.at<float>(0,0) = 1.0f;
    rot.at<float>(1,0) = 0.0f;
    rot.at<float>(2,0) = 0.0f;
    rot.at<float>(0,1) = 0.0f;
    rot.at<float>(1,1) = 1.0f;
    rot.at<float>(2,1) = 0.0f;
    rot.at<float>(0,2) = 0.0f;
    rot.at<float>(1,2) = 0.0f;
    rot.at<float>(2,2) = 1.0f;
    return rot;
}

std::vector<cv::Mat> getRotationMatrices(const std::vector<float> &rot_params)
{
    // rotation matrix along xyz axis
    std::vector<cv::Mat> rotation_matrices;

    // GM CT6
    /*
    std::vector<float> rot_params_z;  // yaw
    rot_params_z.push_back(-48.0f);  // center front
    rot_params_z.push_back(-235.0f);  // center rear
    rot_params_z.push_back(-33.7f);  // driver front (left front)
    rot_params_z.push_back(-54.0f); // driver side (left side)
    rot_params_z.push_back(-30.5f);   // passenger front (right front)
    rot_params_z.push_back(75.0f);  // passenger side (right side)

    std::vector<float> rot_params_x; // pitch
    rot_params_x.push_back(0.1f);  // center front
    rot_params_x.push_back(0.6f);  // center rear
    rot_params_x.push_back(-0.5f);  // driver front (left front)
    rot_params_x.push_back(-1.2f); // driver side (left side)
    rot_params_x.push_back(-0.2f);   // passenger front (right front)
    rot_params_x.push_back(-2.3f);  // passenger side (right side)

    std::vector<float> rot_params_y; // roll
    rot_params_y.push_back(1.2f);  // center front
    rot_params_y.push_back(0.5f);  // center rear
    rot_params_y.push_back(-1.5f);  // driver front (left front)
    rot_params_y.push_back(1.3f); // driver side (left side)
    rot_params_y.push_back(0.5f);   // passenger front (right front)
    rot_params_y.push_back(-0.8f);  // passenger side (right side)
    */

    // CMU CT6
    std::vector<float> rot_params_z;  // yaw
    rot_params_z.push_back(-14.0f);  // center front
    rot_params_z.push_back(-119.0f);  // center rear
    rot_params_z.push_back(-43.65f);  // driver front (left front)
    rot_params_z.push_back(-125.0f); // driver side (left side)
    rot_params_z.push_back(38.5f);   // passenger front (right front)
    rot_params_z.push_back(40.0f);  // passenger side (right side)

    std::vector<float> rot_params_x; // pitch
    rot_params_x.push_back(-1.6f);  // center front
    rot_params_x.push_back(0.6f+pitch_offset);  // center rear
    rot_params_x.push_back(-0.3f+pitch_offset);  // driver front (left front)
    rot_params_x.push_back(-1.2f+pitch_offset); // driver side (left side)
    rot_params_x.push_back(-0.2f+pitch_offset);   // passenger front (right front)
    rot_params_x.push_back(0.0f+pitch_offset);  // passenger side (right side)

    std::vector<float> rot_params_y; // roll
    rot_params_y.push_back(0.7);  // center front
    rot_params_y.push_back(0.5f+roll_offset);  // center rear
    rot_params_y.push_back(1.0f+roll_offset);  // driver front (left front)
    rot_params_y.push_back(1.3f+roll_offset); // driver side (left side)
    rot_params_y.push_back(0.5f+roll_offset);   // passenger front (right front)
    rot_params_y.push_back(-0.2f+roll_offset);  // passenger side (right side)


    for (int i = 0; i < rot_params.size(); i++)
    {
        // Multiply the rotation matrix if rotated along more than one axis
        // For example
        // cv::Mat rot = getRotationMatrixFromTheta_Z(rot_params[i]) * getRotationMatrixFromTheta_Z(rot_params[i]);
        // rotation_matrices.push_back(getRotationMatrixFromTheta_Z(rot_params[i]));
        //rotation_matrices.push_back(getEye());
		cv::Mat rot = getRotationMatrixFromTheta_Z(rot_params_z[i]) * getRotationMatrixFromTheta_X(rot_params_x[i]) * getRotationMatrixFromTheta_Y(rot_params_y[i]);
        //rotation_matrices.push_back(getRotationMatrixFromTheta_Z(rot_params[i]));
        rotation_matrices.push_back(rot);
    }
    return rotation_matrices;
}


std::vector<cv::Mat> getRotationMatrices_tuning(const std::vector<float> &rot_params, float roll_offset, float pitch_offset, float yaw_offset)
{
    // rotation matrix along xyz axis
    std::vector<cv::Mat> rotation_matrices;

    std::ifstream infile("calibration.txt");
    std::string line;
    float calVal = 0.0f;

    std::vector<float> rot_params_z;  // yaw
    std::getline(infile, line); calVal = atof(line.c_str()) + yaw_offset; rot_params_z.push_back(calVal);  // center front
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + yaw_offset; rot_params_z.push_back(calVal);  // center rear
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + yaw_offset; rot_params_z.push_back(calVal);  // driver front (left front)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + yaw_offset; rot_params_z.push_back(calVal); // driver side (left side)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + yaw_offset; rot_params_z.push_back(calVal);   // passenger front (right front)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + yaw_offset; rot_params_z.push_back(calVal);  // passenger side (right side)
    std::cout << calVal << '\n';

    std::vector<float> rot_params_x; // pitch
    std::getline(infile, line); calVal = atof(line.c_str()) + pitch_offset; rot_params_x.push_back(calVal);  // center front
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + pitch_offset; rot_params_x.push_back(calVal);  // center rear
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + pitch_offset; rot_params_x.push_back(calVal);  // driver front (left front)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + pitch_offset; rot_params_x.push_back(calVal); // driver side (left side)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + pitch_offset; rot_params_x.push_back(calVal);   // passenger front (right front)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + pitch_offset; rot_params_x.push_back(calVal);  // passenger side (right side)
    std::cout << calVal << '\n';

    std::vector<float> rot_params_y; // roll
    std::getline(infile, line); calVal = atof(line.c_str()) + roll_offset; rot_params_y.push_back(calVal);  // center front
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + roll_offset; rot_params_y.push_back(calVal);  // center rear
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + roll_offset; rot_params_y.push_back(calVal);  // driver front (left front)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + roll_offset; rot_params_y.push_back(calVal); // driver side (left side)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + roll_offset; rot_params_y.push_back(calVal);   // passenger front (right front)
    std::cout << calVal << '\n';
    std::getline(infile, line); calVal = atof(line.c_str()) + roll_offset; rot_params_y.push_back(calVal);  // passenger side (right side)
    std::cout << calVal << '\n';



    for (int i = 0; i < rot_params.size(); i++)
    {
        // Multiply the rotation matrix if rotated along more than one axis
        // For example
        // cv::Mat rot = getRotationMatrixFromTheta_Z(rot_params[i]) * getRotationMatrixFromTheta_Z(rot_params[i]);
        // rotation_matrices.push_back(getRotationMatrixFromTheta_Z(rot_params[i]));
        //rotation_matrices.push_back(getEye());
		    cv::Mat rot = getRotationMatrixFromTheta_Z(rot_params_z[i]) * getRotationMatrixFromTheta_X(rot_params_x[i]) * getRotationMatrixFromTheta_Y(rot_params_y[i]);
        //rotation_matrices.push_back(getRotationMatrixFromTheta_Z(rot_params[i]));
        rotation_matrices.push_back(rot);
    }
    return rotation_matrices;
}


std::vector<cv::Mat> getTranslationMatrices() {
    std::vector<std::vector<float>> trans_params;
    // GM CT6
    /*
    trans_params.push_back({3.98f, 0.00f, -0.26f});  // center front
    trans_params.push_back({-1.19f, 0.00f, -0.15f});  // center rear
    trans_params.push_back({3.82f, -0.57f, -0.02f});  // driver front (left front)
    trans_params.push_back({2.70f, -0.90f, -0.57f}); // driver side (left side)
    trans_params.push_back({3.88f, 0.57f, -0.02f});   // passenger front (right front)
    trans_params.push_back({2.70f, 0.90f, -0.55f});  // passenger side (right side)
    */
    // CMU CT6
    trans_params.push_back({3.98f, 0.00f, -0.34f});  // center front
    trans_params.push_back({-1.19f, 0.00f, -0.15f});  // center rear
    trans_params.push_back({3.87f, -0.60f, -0.02f});  // driver front (left front)
    trans_params.push_back({2.70f, -0.90f, -0.57f}); // driver side (left side)
    trans_params.push_back({3.85f, 0.61f, -0.02f});   // passenger front (right front)
    trans_params.push_back({2.70f, 0.90f, -0.55f});  // passenger side (right side)
    // trans_params.push_back({3.98f, 0.00f, -0.22f});  // center front
    // trans_params.push_back({-1.19f, 0.00f, 0.14f});  // center rear
    // trans_params.push_back({3.91f, -0.60f, 0.00f});  // driver front (left front)
    // trans_params.push_back({2.70f, -0.90f, -0.55f}); // driver side (left side)
    // trans_params.push_back({3.83f, 0.61f, 0.00f});   // passenger front (right front)
    // trans_params.push_back({2.70f, 0.90f, -0.50f});  // passenger side (right side

    std::vector<cv::Mat> translation_matrices;
    for (int i = 0; i < trans_params.size(); i++)
    {
        cv::Mat trans = cv::Mat::zeros(3, 1, CV_32FC1);
        trans.at<float>(0,0) = trans_params[i][0];
        trans.at<float>(1,0) = trans_params[i][1];
        trans.at<float>(2,0) = trans_params[i][2];
        translation_matrices.push_back(trans);
    }
    return translation_matrices;
}
}
