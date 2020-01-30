#pragma once


#pragma once

#include <memory>

#include <opencv2/core.hpp>
#include <nanovg.h>
#include <glm/glm.hpp>
#include <functional>

#include "LUT.h"
#include "Quad.hpp"
#include "toml11/toml.hpp"

// 1-D 2-R
class TrailView
{
public:
	TrailView() = default;
	TrailView(int camId, const char* configPath, const char* vehicleConfigPath, std::shared_ptr<NVGcontext> vgCtx, std::shared_ptr<CalibLUT> lut);

	void draw(int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawBEV(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawBEV0(int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawSV(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawFisheye(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawFisheyeStaticLine(int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawBEV2(float degree, int x, int y, int width, int height, const glm::mat4& transform);
	void drawDistanceLine(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawDistanceLine2(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));

	void update(float degree);

	void drawWarningLine(int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawWheelTrail(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawWheelTrailBEV(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawCarTrail(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void drawFilledTrail(float degree, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));

private:

	struct TrailStatus
	{
		float arcCX, arcCY;
		float arcStartX, arcStartY;
		float arcEndX, arcEndY;
		float arcR;
		float arcStartAngle, arcEndAngle;
		int arcDir;

		NVGpaint paint;
	};

public:

	void dumpTrailStatus(const TrailStatus& trail);

private:
	void drawArc(const TrailStatus& trail);
	void drawArcWithBorder(const TrailStatus& wheelTrail);
	void drawArcWithBorder2(float degree, const TrailStatus& wheelTrail);
	void drawArcWithBorder2(const TrailStatus& innerTrail, const TrailStatus& outerTrail);
	void loadConfig(const char* path);
	void loadExpandConfig(const char* path);
	void loadVehicleConfig(const char* path);

	Quad calcQuad(const Quad& q);

	cv::Point3f bev2world(const cv::Point2f& p);
	cv::Point2f world2fisheye(const cv::Point3f& p);
	std::tuple<float, float> bev2fev(float x, float y);

	std::shared_ptr<NVGcontext> vgCtx;
	std::shared_ptr<CalibLUT> lut;

	int camId;

	int srcWidth, srcHeight;
	int resultWidth, resultHeight;

	

	// 车辆参数，以像素计
	float wheelBase;    // 轴距
	float frontWheelTread, rearWheelTread;  // 前后轮距
	float frontOverhangLength, rearOverhangLength;  // 前后轴离前后保险杠（即车框的上下边）的距离
	float vehicleWidthWithMirror;
	cv::Rect cropROI;
	WarningLineParam warningLineParam;
	WheelTrailParam wheelTrailParam;
	CarTrailParam carTrailParam;
	BEVTrailParam bevTrailParam;
	DistanceLineParam distanceLineParam;
	DistanceLineParam frontDisLineParam, rearDisLineParam;

	NVGcolor filledTrailColor;
};