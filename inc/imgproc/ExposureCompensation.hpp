/***********************************************************************
 *
 * Mario Created on   : 2019-04-19 14:11
 * Filename      : smooth.hpp
 * Function      : 亮度均衡实现代码
 *
 *************************************************************************/

#pragma once

#include <iostream>
#include <opencv2/core.hpp>
#include <math.h>
#include "LUT.h"
#include <vector>
#include "LuminanceEqualize.hpp"
#include <numeric>
#include <functional>

#include <chrono>
#include <toml11/toml.hpp>
#include "utils/Timer.hpp"

class ExposureCompensation
{
private:

	// 融合像素灰度阈值(防止在看到有高度的物体时,两个相机看到的图像不一致的情况)
	int intensityThreshold = 30;

	// 滞后滤波系数
	float lagAlpha = 0.4;

	// 亮度均衡gains阈值(防止产生过度均衡效果)
	float minGain = 0.7;
	float maxGain = 1.3;

	// 前一帧的gains
	cv::Mat_<double> preGains{4, 1};

	std::array<int, 2> frontLeftRadius, frontRightRadius, rearLeftRadius, rearRightRadius;

public:
	ExposureCompensation(const ExposureCompensation&) = default;

	/**
	 * @param intensityThreshold 融合像素灰度阈值
	 * @param lagAlpha 滞后滤波系数
	 * @param minGain 亮度均衡gains阈值
	 * @param maxGain 亮度均衡gains阈值
	 */
	ExposureCompensation(float intensityThreshold = 30, float lagAlpha = 0.4, float minGain = 0.7, float maxGain = 1.3)
		: intensityThreshold(intensityThreshold), lagAlpha(lagAlpha), minGain(minGain), maxGain(maxGain)
	{
	}

	/**
	 * 从配置文件初始化
	 * @param configPath
	 */
	ExposureCompensation(const char* configPath)
	{
		const auto config = toml::parse(configPath);
		intensityThreshold = toml::find<int>(config, "intensity_threshold");
		lagAlpha = toml::find<float>(config, "lag_alpha");
		minGain = toml::find<float>(config, "min_gain");
		maxGain = toml::find<float>(config, "max_gain");

		frontLeftRadius = toml::find<std::array<int, 2>>(config, "front_left_radius");
		frontRightRadius = toml::find<std::array<int, 2>>(config, "front_right_radius");
		rearLeftRadius = toml::find<std::array<int, 2>>(config, "rear_left_radius");
		rearRightRadius = toml::find<std::array<int, 2>>(config, "rear_right_radius");
	}


	static inline float calcBGRIntensity(const cv::Vec3b& bgr)
	{
		static cv::Vec3f bgr2y{0.114, 0.587, 0.299};
		float intensity = 0;
		for (int i = 0; i < 3; ++i)
		{
			intensity += bgr[i] * bgr2y[i];
		}
		return intensity;
	}

	/**
	 * 计算灰度和
	 * @tparam TPixel 像素类型
	 * @tparam IntensityFunc 计算单个像素灰度值的函数类型
	 *
	 * @param images 四路图片
	 * @param camId1 有效值为 const_value::FRONT_CAM 或 const_value::REAR_CAM
	 * @param camId2 有效值为 const_value::LEFT_CAM 或 const_value::RIGHT_CAM
	 * @param innerRadius 选点扇形区域内部的半径
	 * @param outerRadius 选点扇形区域外部的半径
	 * @param lut 查找表
	 * @param sum1 camId1 对应的灰度和
	 * @param sum2 camId2 对应的灰度和
	 * @param numPixel 选点区域的像素个数
	 * @param intensityFunc 计算单个像素灰度值的函数
	 */
	template<class TPixel, class IntensityFunc>
	static void calcIntensitySum(cv::Mat* images, int camId1, int camId2, int innerRadius, int outerRadius, const CalibLUT& lut,
		double& sum1, double& sum2, int& numPixel,
		IntensityFunc intensityFunc)
	{
		assert((camId1 == 0 || camId1 == 1) && (camId2 == 2 || camId2 == 3));

		int bevWidth = lut.header.bev_img_width;
		int bevHeight = lut.header.bev_img_height;
		const auto& carRect = lut.header.car_Icon_Rect;

		sum1 = sum2 = 0;
		numPixel = 0;

		float innerRadius2 = innerRadius * innerRadius;
		float outerRadius2 = outerRadius * outerRadius;

		// 圆心坐标，cxs 的索引为 camId2，cys 的索引为 camId1
		static int cxs[4] = { 0, 0, carRect.x, carRect.br().x };
		static int cys[4] = { carRect.y, carRect.br().y, 0, 0 };

		int cx = cxs[camId2], cy = cys[camId1];

		// camId2 as index
		static int stepX[4] = { 0, 0, -1, +1 };

		// camId1 as index
		static int stepY[4] = { -1, +1, 0, 0 };

		for (int y = cy;
			(camId1 == 0
				? y > cy - outerRadius
				: y < cy + outerRadius);
			y += stepY[camId1])
		{
			for (int x = cx;
			     (camId2 == 2
			        ? x > cx - outerRadius
			        : x < cx + outerRadius);
			     x += stepX[camId2])
			{
				if (lut(y, x).wt_fusion != 0 && lut(y, x).wt_fusion != 255)
				{
					float dy = y - cy, dx = x - cx;
					float r = dy * dy + dx * dx;

					// 当点落在 innerRadius 和 outerRadius 构成的扇形中，
					// 并且不是非融合区域点时才计算。
					if (r >= innerRadius2 && r <= outerRadius2)
					{
						const auto& point = lut(y, x);
						int pic1 = point.pic1;
						int pic2 = point.pic2;
						const auto& pixel1 = images[pic1].at<TPixel>(point.point_pos1.y, point.point_pos1.x);
						const auto& pixel2 = images[pic2].at<TPixel>(point.point_pos2.y, point.point_pos2.x);

						// 灰度求和
						sum1 += intensityFunc(pixel1);
						sum2 += intensityFunc(pixel2);

						++numPixel;
					}
				}
			}
		}
	}
	/**
	 * @brief 亮度均衡代码具体实现
	 *
	 * @tparam TPixel
	 * @tparam IterationSumFunc
	 * @param images                4张图像[Input]
	 * @param lut                   Lut表[Input]
	 * @param gains                 增益[Output]
	 * @param iterationSum          图像遍历函数
	 */
	template<class TPixel, class IntensityFunc>
	void calcExposureGain(cv::Mat *images, const CalibLUT& lut, float *gains, IntensityFunc intensityFunc)
	{

		// 表示融合区域长宽 [305, 313]
		int mergeHeight = lut.header.car_Icon_Rect.y;
		int mergeWidth = lut.header.car_Icon_Rect.x;

		// 表示原图长宽 [1280, 720]
		int srcWidth = lut.header.src_img_width;
		int srcHeight = lut.header.src_img_height;

		// 表示俯视图长宽 [810, 1080]
		int birdviewWidth = lut.header.bev_img_width;
		int birdviewHeight = lut.header.bev_img_height;

		// 表示待拼接的图像数量
		const int num_images = 4;

		// ** IMPORTANT **:
		// N: 4*4
		//  表示两幅图像间重叠部分的像素数量
		cv::Mat_<int> N(num_images, num_images);
		N.setTo(0);

		// ** IMPORTANT **:
		// I: 4*4
		//  表示两幅图像间重叠部分的(第一个元素所代表的图像的)亮度和
		cv::Mat_<double> I(num_images, num_images);
		I.setTo(0);

		// ************ 计算I与N ************
		auto t1 = std::chrono::high_resolution_clock::now();
		double sum1;
		double sum2;
		int mergePixelNum;

		/**
		区域划分:
				 0
			 ---------
			 |       |
			 |       |
		 2   |       |   3
			 |       |
			 |       |
			 ---------
				 1
		*/
		Timer<> timer;
		timer.start();
		#pragma omp parallel sections num_threads(4)
		{
			#pragma omp section
			{
				// 左上角 对应0,2区域
				calcIntensitySum<TPixel>(images, 0, 2, frontLeftRadius[0], frontLeftRadius[1], lut, sum1, sum2, mergePixelNum, intensityFunc);

				N(0, 2) = N(2, 0) = mergePixelNum;
				if (N(0, 2) != 0)
				{
					I(0, 2) = sum1 / N(0, 2);
					I(2, 0) = sum2 / N(2, 0);
				}
			}
			
			#pragma omp section
			{
				// 右上角
				calcIntensitySum<TPixel>(images, 0, 3, frontRightRadius[0], frontRightRadius[1], lut, sum1, sum2, mergePixelNum, intensityFunc);

				N(0, 3) = N(3, 0) = mergePixelNum;
				if (N(0, 3) != 0)
				{
					I(0, 3) = sum1 / N(0, 3);
					I(3, 0) = sum2 / N(3, 0);
				}
			}

			#pragma omp section
			{
				// 左下角
				calcIntensitySum<TPixel>(images, 1, 2, rearLeftRadius[0], rearLeftRadius[1], lut, sum1, sum2, mergePixelNum, intensityFunc);
				N(1, 2) = N(2, 1) = mergePixelNum;
				if (N(1, 2) != 0)
				{
					I(1, 2) = sum1 / N(1, 2);
					I(2, 1) = sum2 / N(2, 1);
				}
			}

			#pragma omp section
			{
				// 右下角
				calcIntensitySum<TPixel>(images, 1, 3, rearRightRadius[0], rearRightRadius[1], lut, sum1, sum2, mergePixelNum, intensityFunc);
				N(1, 3) = N(3, 1) = mergePixelNum;
				if (N(1, 3) != 0)
				{
					I(1, 3) = sum1 / N(1, 3);
					I(3, 1) = sum2 / N(3, 1);
				}
			}
		}
		timer.stop();
		std::cout << "calc intensity sum: " << timer.get() << std::endl;

		// ************ 最小二乘 ************
		double alpha = 0.01;    //表示σN的平方的倒数
		double beta = 100;    //表示σg的平方的倒数

		cv::Mat_<double> A(num_images, num_images);
		A.setTo(0);
		cv::Mat_<double> b(num_images, 1);
		b.setTo(0);

		for (int i = 0; i < num_images; ++i)
		{
			for (int j = 0; j < num_images; ++j)
			{
				b(i, 0) += beta * N(i, j);
				A(i, i) += beta * N(i, j);
				if (j == i) continue;
				A(i, i) += 2 * alpha * I(i, j) * I(i, j) * N(i, j);
				A(i, j) -= 2 * alpha * I(i, j) * I(j, i) * N(i, j);
			}
		}

		cv::Mat_<double> gains_;
		timer.start();
		bool ret = cv::solve(A, b, gains_);
		timer.stop();
		std::cout << "solve: " << timer.get() << std::endl;
#if 0
		std::cout << "solve return " << std::boolalpha << ret << std::endl;

		auto t2 = std::chrono::high_resolution_clock::now();
		auto timeUsed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		std::cout << " !!!! allExposureComposition solve time cost = " << timeUsed << " ms. " << std::endl;
		std::cout << "!!! A: " << A << std::endl;
		std::cout << "!!! b: " << b << std::endl;
		std::cout << "!!! N: " << N << std::endl;
		std::cout << "!!! I: " << I << std::endl;
		std::cout << "!!!gains = " << gains_ << std::endl;
#endif

#if 0
		// 滞后gains
		for (int i = 0; i < 4; ++i)
		{

			// 阈值操作
			gains_(i, 0) = (gains_(i, 0) > maxGain) ? maxGain : gains_(i, 0);
			gains_(i, 0) = (gains_(i, 0) < minGain) ? minGain : gains_(i, 0);

			gains[i] = lagAlpha * preGains(i, 0) +
			           (1 - lagAlpha) * gains_(i, 0);

			preGains(i, 0) = gains[i];
		}
#else
		for (int i = 0; i < 4; ++i)
		{
			gains[i] = gains_(i, 0);
		}
#endif
	}


	/**
	 * @brief 从BGR图片计算亮度均衡增益gains接口
	 *
	 * @tparam TPixel
	 * @param images        输入4张图片[Input]
	 * @param lut           Lut表[Input]
	 * @param gains         亮度均衡增益[Output]
	 */
	inline void calcExposureGainBGR(cv::Mat *images, const CalibLUT& lut, float *gains)
	{
		calcExposureGain<cv::Vec3b>(images, lut, gains, calcBGRIntensity);
	}

	inline void calcExposureGainUYVY(cv::Mat *images, const CalibLUT& lut, float *gains)
	{
		calcExposureGain<cv::Vec2b>(images, lut, gains,
			[](const cv::Vec2b& pixel) { return pixel[1]; });
	}

	inline void calcExposureGainYUYV(cv::Mat *images, const CalibLUT& lut, float *gains)
	{
		calcExposureGain<cv::Vec2b>(images, lut, gains,
		                            [](const cv::Vec2b& pixel) { return pixel[0]; });
	}

};