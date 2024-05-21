#include <opencv2/opencv.hpp>

void print_grayscale_image(cv::Mat _img){
    int width = _img.cols; // lie
    int height = _img.rows; //hang

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            std::cout << _img.at<float>(i, j) << " ";
        }
        std::cout << std::endl;
    }
    
};

void Compression_grayscale(cv::Mat *img_input, cv::Mat *img_output){
    int x_range = img_output->cols;
    int y_range = img_output->rows;
    int cell_width = (img_input->cols) /  (img_output->cols);
    int cell_height = (img_input->rows) /  (img_output->rows);

    // printf("%d %d %d %d ",x_range,y_range,cell_width,cell_height);

    for (int i = 0; i < x_range; i++)//x
    {
        for (int j = 0; j < y_range; j++)//y
        {
            // 定义区域
            cv::Rect roi(i*cell_width, j*cell_height, 10, 10);  // 在 (100, 100) 处起始，宽度为 200，高度为 200 的矩形区域
            // 提取区域
            cv::Mat roiImage = (*img_input)(roi);
            
            // 计算平均值
            cv::Scalar meanValue = mean(roiImage);
            
            // 输出结果
            double average = meanValue[0];  // 提取灰度图像中的平均值
            // std::cout << "Average: " << average << std::endl;

            img_output->at<float>(cv::Point2d(i,j)) = average ;
        }
    }
};
