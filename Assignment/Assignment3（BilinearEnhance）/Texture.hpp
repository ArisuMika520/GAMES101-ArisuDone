//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;

        //纹理采样加倍
        width = image_data.cols / 2;
        height = image_data.rows / 2;
        cv::pyrDown(image_data, image_data, cv::Size(width, height));

    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        int u_img = static_cast<int>(u * width);
        int v_img = static_cast<int>((1 - v) * height);
        if (u_img < 0) u_img = 0;
        if (u_img >= width) u_img = width - 1;
        if (v_img < 0) v_img = 0;
        if (v_img >= height) v_img = height - 1;


        cv::Vec3b color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    // 边界判定
    int rangeSafe(int x, bool isU) 
    {

        if (x < 0) return 0;

        if (isU && x >= width) 
        {
            return width - 1;
        }
        else if (!isU && x >= height) 
        {
            return height - 1;
        }
        return x;
    }
    //双线性纹理插值
    Eigen::Vector3f getColorBilinear(float u, float v) 
    {

        float u_img = u * width;
        float v_img = (1 - v) * height;

        int u_min = rangeSafe(floor(u_img), true);
        int u_max = rangeSafe(ceil(u_img), true);
        int v_min = rangeSafe(floor(v_img), false);
        int v_max = rangeSafe(ceil(v_img), false);

        cv::Vec3b U00 = image_data.at<cv::Vec3b>(v_max, u_min);
        cv::Vec3b U10 = image_data.at<cv::Vec3b>(v_max, u_max);
        cv::Vec3b U01 = image_data.at<cv::Vec3b>(v_min, u_min);
        cv::Vec3b U11 = image_data.at<cv::Vec3b>(v_min, u_max);

        float lerp_s = (u_img - u_min) / (u_max - u_min);
        float lerp_t = (v_img - v_min) / (v_max - v_min);
        cv::Vec3b cTop = (1 - lerp_s) * U01 + lerp_s * U11;
        cv::Vec3b cBot = (1 - lerp_s) * U00 + lerp_s * U10;

        cv::Vec3b color = (1 - lerp_t) * cTop + lerp_t * cBot;
        return Eigen::Vector3f(color[0], color[1], color[2]);

    }

};
#endif //RASTERIZER_TEXTURE_H
