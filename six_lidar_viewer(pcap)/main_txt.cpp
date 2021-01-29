std::vector<std::string> binaryFiles;

// For TXT files
LidarDataReader(std::string rootPath, int start, int end, cv::Mat &rot, cv::Mat &trans) {
    // std::cout<<"[Data Reader] TXT file : " << rootPath << std::endl;
    this->rot = rot;
    this->trans = trans;
    type = DataType::TXT;
    for(int i=start; i<=end; i++) {
        binaryFiles.push_back(rootPath + "/" + std::to_string(i) + ".txt");
    }
}

pointcloud = readFromTxt(binaryFiles[currentFrame++]);

std::vector<std::vector<float>> readFromTxt(std::string &filename) {
    std::vector<std::vector<float>> pointcloud;
    std::ifstream pointcloud_file(filename);
    float x;
    float y;
    float z;
    std::string current_number;
    while(getline(pointcloud_file, current_number, ' ')) {
        x = std::stof(current_number);
        getline(pointcloud_file, current_number, ' ');
        y = std::stof(current_number);
        getline(pointcloud_file, current_number);
        z = std::stof(current_number);
        x /= 100.0, y /= 100.0, z /= 100.0;
        pointcloud.push_back({x, y, z});
    }
    return pointcloud;
}
