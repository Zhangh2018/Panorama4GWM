#include <cmath>
#include <numeric>
#include "TrailView.hpp"
#include "camera_model.h"

#include <nanovg.h>
#include <toml.hpp>
#include <glm/ext.hpp>
#include <functional>

#include "const_value.h"
#include "Quad.hpp"


static inline int invDir(int dir)
{
	assert(dir == NVG_CW || dir == NVG_CCW);
	return (dir == NVG_CW ? NVG_CCW : NVG_CW);
}

template<class T>
static inline typename std::enable_if<std::is_floating_point<T>::value, NVGcolor>::type
vector2Color(const std::vector<T>& vec)
{
	assert(vec.size() >= 3);

	NVGcolor color;
	if (vec.size() > 3)
	{
		color = nvgRGBAf(vec[0], vec[1], vec[2], vec[3]);
	}
	else
	{
		color = nvgRGBf(vec[0], vec[1], vec[2]);
	}
	return color;
}

template<class T>
static inline typename std::enable_if<std::is_integral<T>::value, NVGcolor>::type
vector2Color(const std::vector<T>& vec)
{
	assert(vec.size() >= 3);

	NVGcolor color;
	if (vec.size() > 3)
	{
		color = nvgRGBA(vec[0], vec[1], vec[2], vec[3]);
	}
	else
	{
		color = nvgRGB(vec[0], vec[1], vec[2]);
	}
	return color;
}


void TrailView::WarningLineParam::from_toml(const toml::value& v)
{
	width = toml::find<float>(v, "width");
	color = vector2Color(toml::find<std::vector<int>>(v, "color"));
	distanceToBumper = toml::find<float>(v, "distance_to_rear_bumper");
}

void TrailView::WheelTrailParam::from_toml(const toml::value& v)
{
	width = toml::find<float>(v, "width");
	length = toml::find<float>(v, "length");
	borderWidth = toml::find<float>(v, "border_width");
	borderColor = vector2Color(toml::find<std::vector<int>>(v, "border_color"));
	fillColor = vector2Color(toml::find<std::vector<int>>(v, "fill_color"));
	distanceToBumper = toml::find<float>(v, "distance_to_rear_bumper");
	borderAlphaRange = toml::find<std::array<int, 2>>(v, "border_alpha_range");
}

void TrailView::CarTrailParam::from_toml(const toml::value& v)
{
	width = toml::find<float>(v, "width");
	length = toml::find<float>(v, "length");
	dashes = toml::find<std::vector<float>>(v, "dashes");
	dashUnit = length / std::accumulate(dashes.begin(), dashes.end(), 0.0);
	distancePointToBumper = toml::find<float>(v, "distance_point_to_rear_bumper");
	distanceToBumper = toml::find<float>(v, "distance_to_rear_bumper");
	color = vector2Color(toml::find<std::vector<int>>(v, "color"));
	alphaRange = toml::find<std::array<int, 2>>(v, "alpha_range");
}

void TrailView::BEVTrailParam::from_toml(const toml::value& v)
{
	width = toml::find<float>(v, "width");
	length = toml::find<float>(v, "length");
	color = vector2Color(toml::find<std::vector<int>>(v, "color"));
	alphaRange = toml::find<std::array<int, 2>>(v, "alpha_range");
}

void TrailView::DistanceLineParam::from_toml(const toml::value& v)
{
	distanceToBumper = toml::find<float>(v, "distance_to_bumper");
	horizontalLength = toml::find<float>(v, "horizontal_part_length");
	length = toml::find<float>(v, "length");
	width = toml::find<float>(v, "width");
	color = vector2Color(toml::find<std::vector<int>>(v, "color"));
}

void TrailView::loadConfig(const char *configPath)
{
	float mmPerPixelH = lut->header.mm_per_pixel_h,
		mmPerPixelW = lut->header.mm_per_pixel_w;

	const auto config = toml::parse(configPath);
	warningLineParam = toml::find<WarningLineParam>(config, "warning_line");
	wheelTrailParam = toml::find<WheelTrailParam>(config, "wheel_trail");
	carTrailParam = toml::find<CarTrailParam>(config, "car_trail");
	bevTrailParam = toml::find<BEVTrailParam>(config, "bev_trail");
	filledTrailColor = vector2Color(
		toml::find<std::vector<int>>(
			toml::find<toml::value>(config, "fill"),
			    "color"
		)
	);

	frontDisLineParam = toml::find<DistanceLineParam>(config, "front_distance_line");
	rearDisLineParam = toml::find<DistanceLineParam>(config, "rear_distance_line");
}

void TrailView::loadVehicleConfig(const char *vehicleConfigPath)
{
	float mmPerPixelH = lut->header.mm_per_pixel_h,
		mmPerPixelW = lut->header.mm_per_pixel_w;

	const auto config = toml::parse(vehicleConfigPath);
	wheelBase = toml::find<float>(config, "wheel_base") / mmPerPixelH;
	frontWheelTread = toml::find<float>(config, "front_wheel_tread") / mmPerPixelW;
	rearWheelTread = toml::find<float>(config, "rear_wheel_tread") / mmPerPixelW;
	frontOverhangLength = toml::find<float>(config, "front_overhang_length") / mmPerPixelH;
	rearOverhangLength = toml::find<float>(config, "rear_overhang_length") / mmPerPixelH;
	vehicleWidthWithMirror = toml::find<float>(config, "vehicle_width_with_mirrors") / mmPerPixelW;
	vehicleWidth = toml::find<float>(config, "vehicle_width") / mmPerPixelW;
}


TrailView::TrailView(int camId, const char *configPath, const char *vehicleConfigPath,
                     std::shared_ptr<NVGcontext> vgCtx, std::shared_ptr<CalibLUT> lut)
	: camId(camId), vgCtx(vgCtx), lut(lut)
{
	loadConfig(configPath);
	loadVehicleConfig(vehicleConfigPath);
	ptrailView = PseudoTrailView(
		-45.0, 45.0, 1.0,
		filledIndices, filledVertices,
		leftWheelIndices, leftWheelVertices,
		rightWheelIndices, rightWheelVertices
		);
}


void drawQuad(NVGcontext *vg, const Quad& q, const NVGcolor& color)
{
	nvgBeginPath(vg);
	nvgMoveTo(vg, q.topLeft().x, q.topLeft().y);
	nvgLineTo(vg, q.topRight().x, q.topRight().y);
	nvgLineTo(vg, q.bottomRight().x, q.bottomRight().y);
	nvgLineTo(vg, q.bottomLeft().x, q.bottomLeft().y);
	nvgClosePath(vg);
	nvgFillColor(vg, color);
	nvgFill(vg);
}

void drawQuad(NVGcontext *vg, const Quad& q, const NVGpaint& paint)
{
	nvgBeginPath(vg);
	nvgMoveTo(vg, q.topLeft().x, q.topLeft().y);
	nvgLineTo(vg, q.topRight().x, q.topRight().y);
	nvgLineTo(vg, q.bottomRight().x, q.bottomRight().y);
	nvgLineTo(vg, q.bottomLeft().x, q.bottomLeft().y);
	nvgClosePath(vg);
	nvgFillPaint(vg, paint);
	nvgFill(vg);
}

/**
 * BEV 图上像素坐标到世界坐标 （假设所有点都在地面上）
 * @param p
 * @return
 */
cv::Point3f TrailView::bev2world(const cv::Point2f& p)
{
	return cv::Point3f{(p.x - lut->header.cx) * lut->header.mm_per_pixel_w,
	                   (lut->header.cy - p.y) * lut->header.mm_per_pixel_h,
	                   0};
}

cv::Point2f TrailView::world2fisheye(const cv::Point3f& p)
{
	double worldRay[3] = {p.x, p.y, p.z};
	double imagePoint[2] = {0};
	World_Ray_To_Image_Point(imagePoint, worldRay, Camera_Model[camId]);
	return cv::Point2f(imagePoint[1], imagePoint[0]);
}

void TrailView::drawWarningLine(int x, int y, int width, int height, const glm::mat4& transform)
{
	if (camId != const_value::REAR_CAM)
	{
		return;
	}
	auto vg = vgCtx.get();
	glViewport(x, y, width, height);
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);
	nvgSave(vg);

	float startY = lut->header.carRealRect.br().y + warningLineParam.distanceToBumper / lut->header.mm_per_pixel_h;
	float startX = lut->header.carRealRect.br().x + (vehicleWidthWithMirror - lut->header.carRealRect.width) / 2;
	float length = vehicleWidthWithMirror;

	int segments = 20;
	float stepX = -length / segments, stepY = 0;

	auto callback = [](void *ctx, NVGvertex *vert)
	{
		auto tv = reinterpret_cast<TrailView *>(ctx);

		cv::Vec3f worldCoord = tv->bev2world({vert->x, vert->y});
		Float64_t worldRay[3] = {worldCoord[0], worldCoord[1], 0};

		Float64_t camRay[3] = {0};
		World_Ray_To_Cam_Ray(camRay, worldRay, &(Camera_Model[tv->camId]->camera_model_ext));

		Float64_t imagePoint[3] = {0};
		Cam_Ray_To_Image_Point(imagePoint, camRay, &(Camera_Model[tv->camId]->camera_model_int));

		vert->x = imagePoint[1];
		vert->y = imagePoint[0];
	};

	nvgVertexTransformCallback(vg, static_cast<NVGVertexTransformCallback>(callback), static_cast<void *>(this));
	nvgTransform4(vg, glm::value_ptr(transform));

	nvgBeginPath(vg);
	nvgMoveTo(vg, startX, startY);
	for (int i = 1; i <= segments; ++i)
	{
		nvgLineTo(vg, startX + stepX * i, startY + stepY * i);
	}
	nvgStrokeWidth(vg, warningLineParam.width / lut->header.mm_per_pixel_h);
	nvgStrokeColor(vg, warningLineParam.color);
	nvgStroke(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);
}

static inline int invertArcDir(int dir)
{
	if (dir == NVG_CW)
	{
		return NVG_CCW;
	}
	else if (dir == NVG_CCW)
	{
		return NVG_CW;
	}
	else
	{
		return -1;
	}
}

void drawArcWithBorder(NVGcontext *vg, float cx, float cy, float r, float a0, float a1, int dir, float width,
                       float borderWidth, const NVGcolor& color, const NVGcolor& fillColor)
{
	assert(dir == NVG_CW || dir == NVG_CCW);

	float innerR = r - width / 2,
		outerR = r + width / 2;
	float a0p = a0 + (dir == NVG_CW ? -1 : 1) * (a0 * width) / outerR;
	if (dir == NVG_CW)
	{
		a0p = a0 - (a0 * width) / outerR;
	}
	if (dir == NVG_CCW)
	{
		a0p = a0 + ((M_PI - a0) * width) / outerR;
	}
	float fillWidth = width - 2 * borderWidth;
	float innerStartX = cx + innerR * std::cos(a0),
		innerStartY = cy + innerR * std::sin(a0),
		innerEndX = cx + innerR * std::cos(a1),
		innerEndY = cy + innerR * std::sin(a1);

	float outerStartX = cx + outerR * std::cos(a0p),
		outerStartY = innerStartY,
		outerEndX = cx + outerR * std::cos(a1),
		outerEndY = cy + outerR * std::sin(a1);

	nvgSave(vg);

	nvgBeginPath(vg);

	if (dir == NVG_CW)
	{
		nvgScissor(vg, innerEndX, innerStartY, outerStartX, outerEndY);
	}
	else
	{
		nvgScissor(vg, outerStartX, outerStartY, innerEndX, outerEndY);
	}
	nvgArc(vg, cx, cy, r, a0p, a1, dir);
	nvgStrokeWidth(vg, fillWidth);
	nvgStrokeColor(vg, nvgRGBAf(1, 0, 0, 1));
	nvgStroke(vg);

	nvgRestore(vg);

#if 1
	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	nvgArc(vg, cx, cy, innerR, a0, a1, dir);
	nvgLineTo(vg, outerEndX, outerEndY);
	nvgArc(vg, cx, cy, outerR, a1, a0p, invertArcDir(dir));

	nvgStrokeWidth(vg, borderWidth);
	//nvgStrokePaint(vg, borderPaint);
	nvgStrokeColor(vg, color);
	nvgStroke(vg);

//	nvgBeginPath(vg);
//	nvgMoveTo(vg, innerStartX, innerStartY);
//	nvgLineTo(vg, innerEndX, innerEndY);
//	nvgLineTo(vg, outerEndX, outerEndY);
//	nvgLineTo(vg, outerStartX, outerStartY);
//	nvgClosePath(vg);
//	nvgStrokeWidth(vg, 2);
//	nvgStrokeColor(vg, nvgRGBAf(0, 0, 1, 1));
//	nvgStroke(vg);
#endif
}


void drawArcWithBorder2(NVGcontext *vg, float cx, float cy, float r, float a0, float a1, int dir, float width,
                       float borderWidth, const NVGcolor& color, const NVGcolor& fillColor)
{
	assert(dir == NVG_CW || dir == NVG_CCW);

	float innerR = r - width / 2,
		outerR = r + width / 2;
	float a0p = a0 + (dir == NVG_CW ? -1 : 1) * (a0 * width) / outerR;
	if (dir == NVG_CW)
	{
		a0p = a0 - (a0 * width) / outerR;
	}
	if (dir == NVG_CCW)
	{
		a0p = a0 + ((M_PI - a0) * width) / outerR;
	}

	float fillWidth = width - 2 * borderWidth;


	float innerStartX = cx + innerR * std::cos(a0),
		innerStartY = cy + innerR * std::sin(a0),
		innerEndX = cx + innerR * std::cos(a1),
		innerEndY = cy + innerR * std::sin(a1);

	float outerStartX = cx + outerR * std::cos(a0p),
		outerStartY = innerStartY,
		outerEndX = cx + outerR * std::cos(a1),
		outerEndY = cy + outerR * std::sin(a1);

	nvgBeginPath(vg);

	// 这里有点问题，现在的问题是在画车轮轨迹线的边框时，startX，startY 会超出 warningLine。
	// 但这里的坐标都已经认真算过，确认是落在 warningLine 上的。

	// nvgMoveTo(vg, innerStartX, innerStartY);
	// nvgArc(vg, cx, cy, innerR, a0, a1, dir);
	// nvgLineTo(vg, outerEndX, outerEndY);
	// nvgArc(vg, cx, cy, outerR, a1, a0p, invertArcDir(dir));

	// nvgClosePath(vg);
	// nvgFillColor(vg, nvgRGBAf(1, 0, 0, 1));
	// nvgFill(vg);

#if 1
	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	nvgArc(vg, cx, cy, innerR, a0, a1, dir);
	nvgLineTo(vg, outerEndX, outerEndY);
	nvgArc(vg, cx, cy, outerR, a1, a0p, invertArcDir(dir));

	nvgStrokeWidth(vg, borderWidth);
	//nvgStrokePaint(vg, borderPaint);
	nvgStrokeColor(vg, color);
	nvgStroke(vg);
#else
	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	nvgLineTo(vg, innerEndX, innerEndY);
	nvgLineTo(vg, outerEndX, outerEndY);
	nvgLineTo(vg, outerStartX, outerStartY);
	nvgClosePath(vg);
	nvgStrokeWidth(vg, 2);
	nvgStrokeColor(vg, nvgRGBAf(0, 0, 1, 1));
	nvgStroke(vg);
#endif
}


void TrailView::drawArcWithBorder(const TrailView::TrailStatus& wheelTrail)
{
	auto vg = vgCtx.get();

	float arcWidth = wheelTrailParam.width / lut->header.mm_per_pixel_w,
		borderWidth = wheelTrailParam.borderWidth / lut->header.mm_per_pixel_w;
	float r = wheelTrail.arcR;
	float innerR = r - arcWidth / 2 + borderWidth / 2;
	float outerR = r + arcWidth / 2 - borderWidth / 2;

	float cx = wheelTrail.arcCX, cy = wheelTrail.arcCY;
	int dir = wheelTrail.arcDir;
	float a0 = wheelTrail.arcStartAngle, a1 = wheelTrail.arcEndAngle;
	float a0p = 0;
	if (dir == NVG_CW)
	{
		a0p = a0 - (a0 * arcWidth) / outerR;
	}
	if (dir == NVG_CCW)
	{
		a0p = a0 + ((M_PI - a0) * arcWidth) / outerR;
	}

	float fillWidth = arcWidth - 2 * borderWidth;

//	float innerStartY = lut->header.carRealRect.br().y + warningLineParam.distanceToBumper / lut->header.mm_per_pixel_h;
//	float innerStartX = cx + std::sqrt(std::pow(innerR, 2) - std::pow(innerStartY - cy, 2));

	float innerStartX = cx + innerR * std::cos(a0),
		innerStartY = cy + innerR * std::sin(a0);

	float outerStartX = cx + outerR * std::cos(a0p),
		outerStartY = innerStartY,
		outerEndX = cx + outerR * std::cos(a1),
		outerEndY = cy + outerR * std::sin(a1);

	nvgBeginPath(vg);

	// 这里有点问题，现在的问题是在画车轮轨迹线的边框时，startX，startY 会超出 warningLine。
	// 但这里的坐标都已经认真算过，确认是落在 warningLine 上的。

	nvgMoveTo(vg, innerStartX, innerStartY);
	nvgArc(vg, cx, cy, innerR, a0, a1, dir);
	nvgLineTo(vg, outerEndX, outerEndY);
	nvgArc(vg, cx, cy, outerR, a1, a0p, invertArcDir(dir));

	nvgClosePath(vg);
	nvgFillColor(vg, wheelTrailParam.fillColor);
	nvgFill(vg);

#if 1
	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	nvgArc(vg, cx, cy, innerR, a0, a1, dir);
	nvgLineTo(vg, outerEndX, outerEndY);
	nvgArc(vg, cx, cy, outerR, a1, a0p, invertArcDir(dir));

	nvgStrokeWidth(vg, borderWidth);
	//nvgStrokePaint(vg, borderPaint);
	nvgStrokeColor(vg, wheelTrailParam.borderColor);
	nvgStroke(vg);
#else
	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	nvgLineTo(vg, innerEndX, innerEndY);
	nvgLineTo(vg, outerEndX, outerEndY);
	nvgLineTo(vg, outerStartX, outerStartY);
	nvgClosePath(vg);
	nvgStrokeWidth(vg, 2);
	nvgStrokeColor(vg, nvgRGBAf(0, 0, 1, 1));
	nvgStroke(vg);
#endif

}

void TrailView::drawArcWithBorder2(float degree, const TrailStatus& wheelTrail)
{
	auto vg = vgCtx.get();

	float startX = wheelTrail.arcStartX, startY = wheelTrail.arcStartY;
	float apexX = wheelTrail.arcCX, apexY = wheelTrail.arcCY;
	float innerStartX = startX - wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
		innerStartY = startY;
	float outerStartX = startX + wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
		outerStartY = startY;

	float dxInnerStartApex = innerStartX - apexX,
		dyInnerStartApex = innerStartY - apexY;
	float dxOuterStartApex = outerStartX - apexX,
		dyOuterStartApex = outerStartY - apexY;

	float innerR = std::sqrt(dyInnerStartApex * dyInnerStartApex + dxInnerStartApex * dxInnerStartApex);
	float outerR = std::sqrt(dyOuterStartApex * dyOuterStartApex + dxOuterStartApex * dxOuterStartApex);

	float innerStartAngle = std::atan(std::abs(dyInnerStartApex / dxInnerStartApex)),
		innerEndAngle = innerStartAngle + wheelTrailParam.length / lut->header.mm_per_pixel_h / innerR;
	float outerStartAngle = std::atan(std::abs(dyOuterStartApex / dxOuterStartApex)),
		outerEndAngle = outerStartAngle + wheelTrailParam.length / lut->header.mm_per_pixel_h / outerR;

	if (degree < 0)
	{
		innerStartAngle = M_PI - innerStartAngle;
		innerEndAngle = M_PI - innerEndAngle;
		outerStartAngle = M_PI - outerStartAngle;
		outerEndAngle = M_PI - outerEndAngle;
	}

	float innerEndX = apexX + std::cos(innerEndAngle) * innerR,
		innerEndY = apexY + std::sin(innerEndAngle) * innerR;
	float outerEndX = apexX + std::cos(outerEndAngle) * outerR,
		outerEndY = apexY + std::sin(outerEndAngle) * outerR;

	int dir = (degree > 0 ? NVG_CW : NVG_CCW);

	// nvgBeginPath(vg);
	// nvgMoveTo(vg, innerStartX, innerStartY);
	// nvgArc(vg, apexX, apexY, innerR, innerStartAngle, innerEndAngle, dir);
	// nvgLineTo(vg, outerEndX, outerEndY);
	// nvgArc(vg, apexX, apexY, outerR, outerEndAngle, outerStartAngle, invertArcDir(dir));

	// nvgClosePath(vg);
	// nvgFillColor(vg, wheelTrailParam.fillColor);
	// nvgFill(vg);

	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	nvgArc(vg, apexX, apexY, innerR, innerStartAngle, innerEndAngle, dir);
	nvgLineTo(vg, outerEndX, outerEndY);
	nvgArc(vg, apexX, apexY, outerR, outerEndAngle, outerStartAngle, invertArcDir(dir));

	nvgStrokeWidth(vg, wheelTrailParam.borderWidth / lut->header.mm_per_pixel_w);
	//nvgStrokeColor(vg, wheelTrailParam.borderColor);
	nvgStrokePaint(vg, wheelTrail.paint);
	nvgStroke(vg);
}

void TrailView::drawArcWithBorder2(const TrailStatus& innerTrail, const TrailStatus& outerTrail)
{
	auto vg = vgCtx.get();

	nvgBeginPath(vg);
	nvgMoveTo(vg, innerTrail.arcStartX, innerTrail.arcStartY);
	nvgArc(vg, innerTrail.arcCX, innerTrail.arcCY, innerTrail.arcR, innerTrail.arcStartAngle, innerTrail.arcEndAngle, innerTrail.arcDir);
	nvgMoveTo(vg, outerTrail.arcEndX, outerTrail.arcEndY);
	nvgArc(vg, innerTrail.arcCX, innerTrail.arcCY, innerTrail.arcR, innerTrail.arcStartAngle, innerTrail.arcEndAngle, invertArcDir(innerTrail.arcDir));

	nvgStrokeWidth(vg, wheelTrailParam.borderWidth / lut->header.mm_per_pixel_w);
	nvgStrokeColor(vg, wheelTrailParam.borderColor);
	nvgStroke(vg);
}

void TrailView::drawWheelTrail(float degree, int x, int y, int width, int height, const glm::mat4& transform)
{
	glm::mat4 transformMatrix = transform;
//	if (degree < 0)
//	{
//		// 转弯角为负时，即向右转时，轨迹是左转当前角度绝对值时的镜像
//		transformMatrix = glm::scale(transformMatrix, glm::vec3(-1, 1, 1));
//		degree = std::abs(degree);
//	}

	float radian = nvgDegToRad(degree);

	auto carRect = lut->header.carRealRect;
	// 前后轴中心
	float rearAxleCenterX = carRect.x + carRect.width / 2.0,
		rearAxleCenterY = carRect.br().y - rearOverhangLength;

	// 弯心坐标
	float apexX = rearAxleCenterX - wheelBase / std::tan(radian),
		apexY = rearAxleCenterY;

	auto vg = vgCtx.get();
	glViewport(x, y, width, height);
	//nvgBeginFrame(vg, resultWidth, resultHeight, 1.0);
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);

	nvgSave(vg);
	nvgReset(vg);


	auto callback = [](void *ctx, NVGvertex *vert)
	{
		auto tv = reinterpret_cast<TrailView *>(ctx);

		cv::Vec3f worldCoord = tv->bev2world({vert->x, vert->y});
		Float64_t worldRay[3] = {worldCoord[0], worldCoord[1], 0};
		Float64_t imagePoint[3] = {0};
		World_Ray_To_Image_Point(imagePoint, worldRay, Camera_Model[tv->camId]);

		vert->x = imagePoint[1];
		vert->y = imagePoint[0];
	};

	nvgVertexTransformCallback(vg, static_cast<NVGVertexTransformCallback>(callback), static_cast<void *>(this));
	nvgTransform4(vg, glm::value_ptr(transformMatrix));

	float arcWidth = wheelTrailParam.width / lut->header.mm_per_pixel_w,
		borderWidth = wheelTrailParam.borderWidth / lut->header.mm_per_pixel_w;

#if 1
//	drawArcWithBorder2(vg, apexX, apexY, rearLeftWheelTrail.arcR,
//	                  rearLeftWheelTrail.arcStartAngle, rearLeftWheelTrail.arcEndAngle,
//	                  rearLeftWheelTrail.arcDir,
//	                  arcWidth,
//	                  borderWidth,
//	                  wheelTrailParam.borderColor,
//	                  wheelTrailParam.fillColor);
//
//	drawArcWithBorder2(vg, apexX, apexY, rearRightWheelTrail.arcR,
//	                  rearRightWheelTrail.arcStartAngle,  rearRightWheelTrail.arcEndAngle,
//	                  rearRightWheelTrail.arcDir,
//	                  arcWidth,
//	                  borderWidth,
//	                  wheelTrailParam.borderColor,
//	                  wheelTrailParam.fillColor);

	drawArcWithBorder2(degree, rearLeftWheelTrail);
	drawArcWithBorder2(degree, rearRightWheelTrail);

//	drawArcWithBorder2(rearLeftWheelInnerTrail, rearLeftWheelOuterTrail);
//	drawArcWithBorder2(rearRightWheelInnerTrail, rearRightWheelOuterTrail);

//	nvgBeginPath(vg);
//
//	nvgCircle(vg, rearLeftWheelTrail.arcStartX, rearLeftWheelTrail.arcStartY, 20);
//	nvgCircle(vg, rearRightWheelTrail.arcStartX, rearRightWheelTrail.arcStartY, 20);
//	nvgStrokeWidth(vg, 2);
//	nvgStrokeColor(vg, nvgRGBAf(1, 1, 0, 1));
//	nvgStroke(vg);

#else
	drawArcWithBorder(vg, apexX, apexY, rearLeftWheelTrail.arcR,
	                  rearLeftWheelTrail.arcStartAngle, rearLeftWheelTrail.arcEndAngle,
	                  rearLeftWheelTrail.arcDir,
	                  arcWidth,
	                  borderWidth,
	                  rearLeftWheelTrail.paint,
	                  wheelTrailParam.fillColor);

	drawArcWithBorder(vg, apexX, apexY, rearRightWheelTrail.arcR,
	                  rearRightWheelTrail.arcStartAngle,  rearRightWheelTrail.arcEndAngle,
	                  rearRightWheelTrail.arcDir,
	                  arcWidth,
	                  borderWidth,
	                  rearRightWheelTrail.paint,
	                  wheelTrailParam.fillColor);
#endif
	nvgRestore(vg);
	nvgEndFrame(vg);
}

void drawLineFEV(NVGcontext* vg, float startX, float startY, float endX, float endY, int numSegments = 20);

void TrailView::drawArcWithBorder0(NVGcontext* vg, float startX, float startY)
{
	float endX = startX, endY = startY + wheelTrailParam.length / lut->header.mm_per_pixel_h;

	float innerStartX = startX - wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
		innerStartY = startY;
	float outerStartX = startX + wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
		outerStartY = startY;
	float innerEndX = innerStartX, innerEndY = endY;
	float outerEndX = outerStartX, outerEndY = endY;

	int numSegments = 10;

#if 0
	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	drawLineFEV(vg, innerStartX, innerStartY, innerEndX, innerEndY, numSegments);
	drawLineFEV(vg, innerEndX, innerEndY, outerEndX, outerEndY, numSegments);
	drawLineFEV(vg, outerEndX, outerEndY, outerStartX, outerStartY, numSegments);
	drawLineFEV(vg, outerStartX, outerStartY, innerStartX, innerStartY, numSegments);

	nvgClosePath(vg);
	nvgFillColor(vg, wheelTrailParam.fillColor);
	nvgFill(vg);
#endif

	nvgBeginPath(vg);
	nvgMoveTo(vg, innerStartX, innerStartY);
	drawLineFEV(vg, innerStartX, innerStartY, innerEndX, innerEndY, numSegments);
	drawLineFEV(vg, innerEndX, innerEndY, outerEndX, outerEndY, numSegments);
	drawLineFEV(vg, outerEndX, outerEndY, outerStartX, outerStartY, numSegments);
	drawLineFEV(vg, outerStartX, outerStartY, innerStartX, innerStartY, numSegments);

	NVGcolor startColor = wheelTrailParam.borderColor;
	startColor.a = wheelTrailParam.borderAlphaRange[0] / 255.0;
	NVGcolor endColor = wheelTrailParam.borderColor;
	endColor.a = wheelTrailParam.borderAlphaRange[1] / 255.0;

	NVGpaint paint = nvgLinearGradient(vg, startX, startY, endX, endY, startColor, endColor);
	nvgStrokeWidth(vg, wheelTrailParam.borderWidth / lut->header.mm_per_pixel_w);
	nvgStrokePaint(vg, paint);
	nvgStroke(vg);
}

void TrailView::drawWheelTrail0(int x, int y, int width, int height, const glm::mat4& transform)
{
	auto vg = vgCtx.get();

	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);
	nvgSave(vg);

	auto callback = [](void* ctx, NVGvertex* vert)
	{
		auto tv = reinterpret_cast<TrailView*>(ctx);
		std::tie(vert->x, vert->y) = tv->bev2fev(vert->x, vert->y);
	};
	nvgTransform4(vg, glm::value_ptr(transform));
	nvgVertexTransformCallback(vg, callback, static_cast<void*>(this));

	auto carRect = lut->header.carRealRect;
	carRect.x -= (vehicleWidthWithMirror - carRect.width) / 2;
	carRect.width = vehicleWidthWithMirror;

	float rearAxleCenterX = carRect.x + carRect.width / 2.0,
		rearAxleCenterY = carRect.br().y - rearOverhangLength;

	float offsetX = rearWheelTread / 2,
		offsetY = wheelTrailParam.distanceToBumper / lut->header.mm_per_pixel_h;
	drawArcWithBorder0(vg, rearAxleCenterX - offsetX, carRect.br().y + offsetY);
	drawArcWithBorder0(vg, rearAxleCenterX + offsetX, carRect.br().y + offsetY);

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void TrailView::drawWheelTrailBEV(float degree, int x, int y, int width, int height, const glm::mat4& transform)
{
	glm::mat4 transformMatrix = transform;
//	if (degree < 0)
//	{
//		// 转弯角为负时，即向右转时，轨迹是左转当前角度绝对值时的镜像
//		transformMatrix = glm::scale(transformMatrix, glm::vec3(-1, 1, 1));
//		degree = std::abs(degree);
//	}

	float radian = nvgDegToRad(degree);

	auto carRect = lut->header.carRealRect;
	// 前后轴中心
	float rearAxleCenterX = carRect.x + carRect.width / 2.0,
		rearAxleCenterY = carRect.br().y - rearOverhangLength;

	// 弯心坐标
	float apexX = rearAxleCenterX - wheelBase / std::tan(radian),
		apexY = rearAxleCenterY;


	auto vg = vgCtx.get();
	glViewport(x, y, width, height);
	//nvgBeginFrame(vg, resultWidth, resultHeight, 1.0);
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);

	nvgSave(vg);
	nvgReset(vg);

	//nvgTransform4(vg, glm::value_ptr(transformMatrix));

	float arcWidth = wheelTrailParam.width / lut->header.mm_per_pixel_w,
		borderWidth = wheelTrailParam.borderWidth / lut->header.mm_per_pixel_w;

#if 0
	drawArcWithBorder2(vg, apexX, apexY, rearLeftWheelTrail.arcR,
	                  rearLeftWheelTrail.arcStartAngle, rearLeftWheelTrail.arcEndAngle,
	                  rearLeftWheelTrail.arcDir,
	                  arcWidth,
	                  borderWidth,
	                  wheelTrailParam.borderColor,
	                  wheelTrailParam.fillColor);

	drawArcWithBorder2(vg, apexX, apexY, rearRightWheelTrail.arcR,
	                  rearRightWheelTrail.arcStartAngle,  rearRightWheelTrail.arcEndAngle,
	                  rearRightWheelTrail.arcDir,
	                  arcWidth,
	                  borderWidth,
	                  wheelTrailParam.borderColor,
	                  wheelTrailParam.fillColor);
#else
	drawArcWithBorder2(degree, rearLeftWheelTrail);
	drawArcWithBorder2(degree, rearRightWheelTrail);
#endif
	nvgRestore(vg);
	nvgEndFrame(vg);
}


void drawDashLine(NVGcontext *vg,
                  const std::vector<float> dashes,
                  float startX,
                  float startY,
                  float length,
                  float lineWidth,
                  const NVGcolor& color);

void drawDashArc(NVGcontext *vg, float cx, float cy, float r, float a0, float a1, int dir,
                 const std::vector<float>& dashArray, float arcWidth, const NVGcolor& color);

void drawDashArc(NVGcontext *vg, float cx, float cy, float r, float a0, float a1, int dir,
                 const std::vector<float>& dashArray, float arcWidth, const NVGpaint& paint);

void dashArcTest(NVGcontext *vg);

void TrailView::drawFisheye(float degree, int x, int y, int width, int height, const glm::mat4& transform)
{

#if 1
	if (camId == const_value::REAR_CAM)
	{
		ptrailView.drawFilledTrail(degree, x, y, width, height, filledTrailColor, transform);
		ptrailView.drawWheelTrail(degree, x, y, width, height, wheelTrailParam.fillColor, transform);

		if (degree == 0)
		{
			//drawFilledTrail0(x, y, width, height, transform);
			drawCarTrail0(x, y, width, height, transform);
			drawWheelTrail0(x, y, width, height, transform);
			drawWarningLine(x, y, width, height, transform);
			drawDistanceLine0(x, y, width, height, transform);
		}
		else
		{
			//drawFilledTrail(degree, x, y, width, height, transform);
			drawWheelTrail(degree, x, y, width, height, transform);
			drawCarTrail(degree, x, y, width, height, transform);
			drawWarningLine(x, y, width, height, transform);
		}
	}
	drawDistanceLine(degree, x, y, width, height, transform);
#else
	auto vg = vgCtx.get();
	glViewport(0, 0, 1920, 1080);
	nvgBeginFrame(vg, 1920, 1080, 1);
	drawArcWithBorder(vg, 1920 / 2, 1080 / 2, 400, M_PI / 6 * 5, M_PI_2, NVG_CCW,
		50, 4,
		nvgRGBAf(1, 1, 0, 1),
		nvgRGBAf(1, 0, 0, 1));
	drawArcWithBorder(vg, 1920 / 2, 1080 / 2, 400, M_PI / 6, M_PI_2, NVG_CW,
		50, 4,
		nvgRGBAf(1, 1, 0, 1),
		nvgRGBAf(1, 0, 0, 1));
	nvgEndFrame(vg);
#endif
}

void drawDashLine(NVGcontext *vg,
                  const std::vector<float> dashes,
                  float startX,
                  float startY,
                  float length,
                  float lineWidth,
                  const NVGcolor& color)
{
	nvgBeginPath(vg);

	nvgMoveTo(vg, startX, startY);
	float dashStep = length / std::accumulate(dashes.begin(), dashes.end(), 0.0);
	float lastX = startX, lastY = startY;
	for (int i = 0; i < dashes.size(); ++i)
	{
		lastY += dashStep * dashes[i];
		if (i % 2 == 0)
		{
			nvgLineTo(vg, lastX, lastY);
		}
		else
		{
			nvgMoveTo(vg, lastX, lastY);
		}
	}

	nvgStrokeWidth(vg, lineWidth);
	nvgStrokeColor(vg, color);
	nvgStroke(vg);
}

void drawDashLine(NVGcontext *vg,
                  const std::vector<float> dashes,
                  float startX,
                  float startY,
                  float length,
                  float lineWidth,
                  const NVGpaint& paint)
{
	nvgBeginPath(vg);

	nvgMoveTo(vg, startX, startY);
	float dashStep = length / std::accumulate(dashes.begin(), dashes.end(), 0.0);
	float lastX = startX, lastY = startY;
	for (int i = 0; i < dashes.size(); ++i)
	{
		lastY += dashStep * dashes[i];
		if (i % 2 == 0)
		{
			nvgLineTo(vg, lastX, lastY);
		}
		else
		{
			nvgMoveTo(vg, lastX, lastY);
		}
	}

	nvgStrokeWidth(vg, lineWidth);
	nvgStrokePaint(vg, paint);
	nvgStroke(vg);
}

/**
 * 画虚线形式的弧线
 */
void drawDashArc(NVGcontext *vg, float cx, float cy, float r, float a0, float a1, int dir,
                 const std::vector<float>& dashArray, float arcWidth, const NVGcolor& color)
{
	nvgBeginPath(vg);

	float lastAngle = a0;

	// 计算圆弧的角度，nanovg 中角度定义是从 x 正轴出发，顺时针旋转为 [0, 2 * PI]
	// 所以当顺时针画弧时，角度为 终止角 - 起始角
	// 逆时针画弧时，角度为 (终止角 - 起始角) - 2 * PI
	float deltaA = (dir == NVG_CW ? (a1 - a0) : ((a1 - a0) - 2 * M_PI));

	// 计算 dash 中单位 1 所表示的弧度值
	float dashStep = deltaA / std::accumulate(dashArray.begin(), dashArray.end(), 0.0);

	for (int i = 0; i < dashArray.size(); ++i)
	{
		float arcAngle = dashStep * dashArray[i];
		float arcStartAngle = lastAngle;
		float arcEndAngle = arcStartAngle + arcAngle;

		// 计算当前这段弧的起始和终止点
		float arcStartX = cx + std::cos(arcStartAngle) * r,
			arcStartY = cy + std::sin(arcStartAngle) * r,
			arcEndX = cx + std::cos(arcEndAngle) * r,
			arcEndY = cy + std::sin(arcEndAngle) * r;

		nvgMoveTo(vg, arcStartX, arcStartY);

		//  跳过空白段
		if (i % 2 == 0)
		{
			nvgArc(vg, cx, cy, r, arcStartAngle, arcEndAngle, dir);
		}

		nvgMoveTo(vg, arcEndX, arcEndY);

		lastAngle = arcEndAngle;
	}

	nvgStrokeWidth(vg, arcWidth);
	nvgStrokeColor(vg, color);
	nvgStroke(vg);
}

void drawDashArc(NVGcontext *vg, float cx, float cy, float r, float a0, float a1, int dir,
                 const std::vector<float>& dashArray, float arcWidth, const NVGpaint& paint)
{
	assert(dir == NVG_CW || dir == NVG_CCW);
	nvgBeginPath(vg);

	float lastAngle = a0;

	// 计算圆弧的角度，nanovg 中角度定义是从 x 正轴出发，顺时针旋转为 [0, 2 * PI]
	float deltaA = 0;
	if (dir == NVG_CW)
	{
		deltaA = (a1 > a0 ? (a1 - a0) : (2 * M_PI - (a0 - a1)));
	}
	else // dir == NVG_CCW
	{
		deltaA = (a1 > a0 ? ((a1 - a0) - 2 * M_PI) : (a1 - a0));
	}

	// 计算 dash 中单位 1 所表示的弧度值
	float dashStep = deltaA / std::accumulate(dashArray.begin(), dashArray.end(), 0.0);

	for (int i = 0; i < dashArray.size(); ++i)
	{
		float arcAngle = dashStep * dashArray[i];
		float arcStartAngle = lastAngle;
		float arcEndAngle = arcStartAngle + arcAngle;

		// 计算当前这段弧的起始和终止点
		float arcStartX = cx + std::cos(arcStartAngle) * r,
			arcStartY = cy + std::sin(arcStartAngle) * r,
			arcEndX = cx + std::cos(arcEndAngle) * r,
			arcEndY = cy + std::sin(arcEndAngle) * r;

		nvgMoveTo(vg, arcStartX, arcStartY);

		//  跳过空白段
		if (i % 2 == 0)
		{
			nvgArc(vg, cx, cy, r, arcStartAngle, arcEndAngle, dir);
		}

		nvgMoveTo(vg, arcEndX, arcEndY);

		lastAngle = arcEndAngle;
	}
	//nvgLineCap(vg, NVG_ROUND);
	nvgStrokeWidth(vg, arcWidth);
	nvgStrokePaint(vg, paint);
	nvgStroke(vg);
}

void dashArcTest(NVGcontext *vg)
{
	float r = 400;
	float lineWidth = 50;
	float cx = 1920 / 2, cy = 1080 / 2;
	float a0 = 0, a1 = M_PI_2;
	float arcStartX = cx + std::cos(a0) * r, arcStartY = cy + std::sin(a0) * r;
	float arcEndX = cx + std::cos(a1) * r, arcEndY = cy + std::sin(a1) * r;
	auto paint = nvgLinearGradient(vg, arcStartX, arcStartY, arcEndX, arcEndY,
	                               nvgRGBAf(1, 0, 0, 1),
	                               nvgRGBAf(1, 0, 0, 0.4));
	auto radPaint = nvgRadialGradient(vg, cx, cy, r, r + lineWidth,
	                                  nvgRGBAf(1, 0, 0, 1),
	                                  nvgRGBAf(1, 0, 0, 0.4));

	float arcAngle = a1 - a0;
	float arcLength = r / arcAngle;
	std::vector<float> dashArray = {1.0, 0.2, 0.8, 0.2, 0.6, 0.2, 0.4};

	nvgBeginFrame(vg, 1920, 1080, 1);
	nvgBeginPath(vg);
	//nvgArc(vg, cx, cy, r, a0, a1, NVG_CCW);
	nvgMoveTo(vg, arcStartX, arcStartY);
	nvgLineTo(vg, arcEndX, arcEndY);
	nvgStrokeWidth(vg, 10);
	nvgStrokePaint(vg, paint);
	nvgStroke(vg);

	drawDashArc(vg, cx, cy, r, a0, a1, NVG_CW, dashArray, lineWidth, paint);

	nvgEndFrame(vg);
}

std::tuple<float, float> TrailView::bev2fev(float x, float y)
{
	auto w = bev2world({x, y});
	double world[3] = {w.x, w.y, 0};
	double image_point[3] = {0};
	World_Ray_To_Image_Point(image_point, world, Camera_Model[camId]);
	return std::make_tuple<float, float>(image_point[1], image_point[0]);
}

void TrailView::drawCarTrail(float degree, int x, int y, int width, int height, const glm::mat4& transform)
{
	glm::mat4 transformMatrix = transform;

	float radian = nvgDegToRad(degree);

	auto carRect = lut->header.carRealRect;
	// 前后轴中心
	float rearAxleCenterX = carRect.x + carRect.width / 2.0,
		rearAxleCenterY = carRect.br().y - rearOverhangLength;

	// 四个轮子的坐标
	float rearLeftWheelX = rearAxleCenterX - rearWheelTread / 2.0,
		rearLeftWheelY = rearAxleCenterY;
	float rearRightWheelX = rearAxleCenterX + rearWheelTread / 2.0,
		rearRightWheelY = rearAxleCenterY;

	// 弯心坐标
	float apexX = rearAxleCenterX - wheelBase / std::tan(radian),
		apexY = rearAxleCenterY;

	auto vg = vgCtx.get();
	glViewport(x, y, width, height);
	//nvgBeginFrame(vg, resultWidth, resultHeight, 1.0);;
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);

	nvgSave(vg);
	nvgReset(vg);

	auto callback = [](void *ctx, NVGvertex *vert)
	{
		auto tv = reinterpret_cast<TrailView *>(ctx);

		cv::Vec3f worldCoord = tv->bev2world({vert->x, vert->y});
		Float64_t worldRay[3] = {worldCoord[0], worldCoord[1], 0};

		Float64_t camRay[3] = {0};
		World_Ray_To_Cam_Ray(camRay, worldRay, &(Camera_Model[tv->camId]->camera_model_ext));

		Float64_t imagePoint[3] = {0};
		Cam_Ray_To_Image_Point(imagePoint, camRay, &(Camera_Model[tv->camId]->camera_model_int));

		vert->x = imagePoint[1];
		vert->y = imagePoint[0];
	};

	nvgVertexTransformCallback(vg, static_cast<NVGVertexTransformCallback>(callback), static_cast<void *>(this));
	nvgTransform4(vg, glm::value_ptr(transformMatrix));

	float lineLength = wheelTrailParam.length / lut->header.mm_per_pixel_h;
	float lineWidth = wheelTrailParam.width / lut->header.mm_per_pixel_w;

	drawDashArc(vg, apexX, apexY, rearLeftCarTrail.arcR,
	            rearLeftCarTrail.arcStartAngle, rearLeftCarTrail.arcEndAngle, rearLeftCarTrail.arcDir,
	            carTrailParam.dashes,
	            carTrailParam.width / lut->header.mm_per_pixel_w,
	            rearLeftCarTrail.paint);

	drawDashArc(vg, apexX, apexY, rearRightCarTrail.arcR,
	            rearRightCarTrail.arcStartAngle, rearRightCarTrail.arcEndAngle, rearRightCarTrail.arcDir,
	            carTrailParam.dashes,
	            carTrailParam.width / lut->header.mm_per_pixel_w,
	            rearRightCarTrail.paint);

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void TrailView::drawCarTrail0(int x, int y, int width, int height, const glm::mat4& transform)
{
	auto vg = vgCtx.get();
	auto carRect = lut->header.carRealRect;
	carRect.x -= (vehicleWidthWithMirror - carRect.width) / 2;
	carRect.width = vehicleWidthWithMirror;

	glViewport(x, y, width, height);
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);
	nvgSave(vg);

	auto callback = [](void* ctx, NVGvertex* vert)
	{
		auto tv = reinterpret_cast<TrailView*>(ctx);
		std::tie(vert->x, vert->y) = tv->bev2fev(vert->x, vert->y);
	};
	nvgTransform4(vg, glm::value_ptr(transform));
	nvgVertexTransformCallback(vg, callback, static_cast<void*>(this));
	float offsetX = carTrailParam.width / lut->header.mm_per_pixel_w / 2;
	float offsetY = carTrailParam.distanceToBumper / lut->header.mm_per_pixel_h;
	float lineLength = carTrailParam.length / lut->header.mm_per_pixel_h;
	float lineWidth = carTrailParam.width / lut->header.mm_per_pixel_w;
	NVGcolor startColor = carTrailParam.color;
	startColor.a = carTrailParam.alphaRange[0] / 255.0;
	NVGcolor endColor = carTrailParam.color;
	endColor.a = carTrailParam.alphaRange[1] / 255.0;
	{
		// left
		float startX = carRect.x + offsetX,
			startY = carRect.br().y + offsetY;
		float endX = startX,
			endY = startY + lineLength;
		float fevStartX, fevStartY, fevEndX, fevEndY;
		std::tie(fevStartX, fevStartY) = bev2fev(startX, startY);
		std::tie(fevEndX, fevEndY) = bev2fev(endX, endY);

		NVGpaint paint = nvgLinearGradient(vg, fevStartX, fevStartY, fevEndX, fevEndY, startColor, endColor);
		drawDashLine(vg,
		             carTrailParam.dashes,
		             startX,
		             startY,
		             lineLength,
		             lineWidth,
		             paint);
	}
	{
		// right
		float startX = carRect.br().x - offsetX,
			startY = carRect.br().y + offsetY;
		float endX = startX,
			endY = startY + lineLength;
		float fevStartX, fevStartY, fevEndX, fevEndY;
		std::tie(fevStartX, fevStartY) = bev2fev(startX, startY);
		std::tie(fevEndX, fevEndY) = bev2fev(endX, endY);

		NVGpaint paint = nvgLinearGradient(vg, fevStartX, fevStartY, fevEndX, fevEndY, startColor, endColor);
		drawDashLine(vg,
		             carTrailParam.dashes,
		             startX,
		             startY,
		             lineLength,
		             lineWidth,
		             paint);
	}

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void TrailView::dumpTrailStatus(const TrailStatus& trail)
{
	std::ofstream out{ "trail.csv", std::ios::app };
	out << trail.arcCX << ','
	<< trail.arcCY << ','
	<< trail.arcR << ','
	<< trail.arcStartX << ','
	<< trail.arcStartY << ','
	<< trail.arcEndX << ','
	<< trail.arcEndY << ','
	<< trail.arcStartAngle << ','
	<< trail.arcEndAngle << std::endl;
}

void TrailView::update(float degree)
{
	float radian = nvgDegToRad(degree);

	cv::Rect2f carRect = lut->header.carRealRect;
	carRect.x -= (vehicleWidthWithMirror - carRect.width) / 2;
	carRect.width = vehicleWidthWithMirror;
	// 前后轴中心
	// 如果用 BEV 坐标来算，得看标定生成的表到底准不准
	float frontAxleCenterX = carRect.x + carRect.width / 2.0,
		frontAxleCenterY = carRect.y - frontOverhangLength;
	float rearAxleCenterX = carRect.x + carRect.width / 2.0,
		rearAxleCenterY = carRect.br().y - rearOverhangLength;

	// 弯心坐标
	float l = wheelBase / std::tan(radian);
	float apexX = rearAxleCenterX - wheelBase / std::tan(radian),
		apexY = rearAxleCenterY;

	auto updateSingleTrail = [&](TrailStatus& trail, float x, float distanceToBumper, float length)
	{
		trail.arcCX = apexX;
		trail.arcCY = apexY;

		trail.arcStartX = x;
		trail.arcStartY = carRect.br().y + distanceToBumper / lut->header.mm_per_pixel_h;
		float dxArcApex = trail.arcStartX - apexX;
		float dyArcApex = trail.arcStartY - apexY;

		trail.arcR = std::sqrt(dxArcApex * dxArcApex + dyArcApex * dyArcApex);
		trail.arcStartAngle = std::atan(std::abs(trail.arcStartY - apexY) / std::abs(trail.arcStartX - apexX));
		trail.arcEndAngle = trail.arcStartAngle + length / lut->header.mm_per_pixel_h / trail.arcR;

		if (radian < 0)
		{
			trail.arcStartAngle = M_PI - trail.arcStartAngle;
			trail.arcEndAngle = M_PI - trail.arcEndAngle;
		}

		// 按照 nanovg 的角度范围的定义，
		// 左转时，后轮轨迹线为顺时针；
		// 右转时，后轮轨迹线为逆时针。
		trail.arcDir = (radian > 0 ? NVG_CW : NVG_CCW);

		trail.arcEndX = apexX + std::cos(trail.arcEndAngle) * trail.arcR;
		trail.arcEndY = apexY + std::sin(trail.arcEndAngle) * trail.arcR;

		// 圆弧两端顶点在鱼眼图上的坐标

	};

	auto updateTrailPaint = [&](TrailStatus& trail, float startX, float startY, float endX, float endY, const NVGcolor& startColor, const NVGcolor& endColor)
	{
		float fevArcStartX, fevArcStartY, fevArcEndX, fevArcEndY;
		std::tie(fevArcStartX, fevArcStartY) = bev2fev(startX, startY);
		std::tie(fevArcEndX, fevArcEndY) = bev2fev(endX, endY);

		trail.paint = nvgLinearGradient(vgCtx.get(), fevArcStartX, fevArcStartY, fevArcEndX, fevArcEndY, startColor, endColor);
	};

	auto updateTrailPaint1 = [&](TrailStatus& trail, const NVGcolor& startColor, const NVGcolor& endColor)
	{
		updateTrailPaint(trail, trail.arcStartX, trail.arcStartY, trail.arcEndX, trail.arcEndY, startColor, endColor);
	};

	float offsetX = carTrailParam.width / lut->header.mm_per_pixel_w / 2;
	updateSingleTrail(rearLeftCarTrail,
		carRect.x + carTrailParam.width / lut->header.mm_per_pixel_w / 2,
		carTrailParam.distanceToBumper,
		//rearDisLineParam.distanceToBumper + rearDisLineParam.length,
		carTrailParam.length);
	updateSingleTrail(rearRightCarTrail,
		carRect.br().x - carTrailParam.width / lut->header.mm_per_pixel_w / 2,
		carTrailParam.distanceToBumper,
		//rearDisLineParam.distanceToBumper + rearDisLineParam.length,
		carTrailParam.length);

	updateSingleTrail(rearLeftWheelInnerTrail,
		rearAxleCenterX - rearWheelTread  / 2 - wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
		wheelTrailParam.distanceToBumper,
		wheelTrailParam.length);
	updateSingleTrail(rearLeftWheelOuterTrail,
	                  rearAxleCenterX - rearWheelTread  / 2 + wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
	                  wheelTrailParam.distanceToBumper,
	                  wheelTrailParam.length);

	updateSingleTrail(rearRightWheelInnerTrail,
		rearAxleCenterX + rearWheelTread / 2 - wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
		              wheelTrailParam.distanceToBumper,
		              wheelTrailParam.length);
	updateSingleTrail(rearRightWheelOuterTrail,
	                  rearAxleCenterX + rearWheelTread / 2 + wheelTrailParam.width / lut->header.mm_per_pixel_w / 2,
	                  wheelTrailParam.distanceToBumper,
	                  wheelTrailParam.length);


	updateSingleTrail(rearLeftWheelTrail,
	                  rearAxleCenterX - rearWheelTread  / 2,
	                  wheelTrailParam.distanceToBumper,
	                  wheelTrailParam.length);
	updateSingleTrail(rearRightWheelTrail,
		rearAxleCenterX + rearWheelTread / 2,
		wheelTrailParam.distanceToBumper,
		wheelTrailParam.length);

	dumpTrailStatus(rearLeftWheelTrail);


	updateSingleTrail(rearLeftDistanceTrail,
	                  carRect.x + rearDisLineParam.width / lut->header.mm_per_pixel_w / 2,
	                  rearDisLineParam.distanceToBumper,
	                  rearDisLineParam.length);
	updateSingleTrail(rearRightDistanceTrail,
	                  carRect.br().x - rearDisLineParam.width / lut->header.mm_per_pixel_w / 2,
	                  rearDisLineParam.distanceToBumper,
	                  rearDisLineParam.length);

	{
		NVGcolor startColor = carTrailParam.color;
		startColor.a = carTrailParam.alphaRange[0] / 255.0;
		NVGcolor endColor = carTrailParam.color;
		endColor.a = carTrailParam.alphaRange[1] / 255.0;
		updateTrailPaint(rearLeftCarTrail,
		                 rearLeftDistanceTrail.arcEndX,
		                 rearLeftDistanceTrail.arcEndY,
		                 rearLeftCarTrail.arcEndX,
		                 rearLeftCarTrail.arcEndY,
		                 startColor,
		                 endColor);
		updateTrailPaint(rearRightCarTrail,
			rearRightDistanceTrail.arcEndX,
			rearRightDistanceTrail.arcEndY,
			rearRightCarTrail.arcEndX,
			rearRightCarTrail.arcEndY,
			startColor,
			endColor);
	}
	{
		NVGcolor startColor = wheelTrailParam.borderColor;
		startColor.a = wheelTrailParam.borderAlphaRange[0] / 255.0;
		NVGcolor endColor = wheelTrailParam.borderColor;
		endColor.a = wheelTrailParam.borderAlphaRange[1] / 255.0;

		updateTrailPaint1(rearLeftWheelTrail, startColor, endColor);
		updateTrailPaint1(rearRightWheelTrail, startColor, endColor);
	}

}

void TrailView::drawFilledTrail(float degree, int x, int y, int width, int height, const glm::mat4& transform)
{
	glm::mat4 transformMatrix = transform;

	float radian = nvgDegToRad(degree);

	auto carRect = lut->header.carRealRect;

	// 前后轴中心
	float rearAxleCenterX = carRect.x + carRect.width / 2.0,
		rearAxleCenterY = carRect.br().y - rearOverhangLength;

	// 弯心坐标
	float apexX = rearAxleCenterX - wheelBase / std::tan(radian),
		apexY = rearAxleCenterY;

	auto vg = vgCtx.get();
	glViewport(x, y, width, height);
	//nvgBeginFrame(vg, resultWidth, resultHeight, 1.0);;
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);

	nvgSave(vg);
	nvgReset(vg);

	auto callback = [](void *ctx, NVGvertex *vert)
	{
		auto tv = reinterpret_cast<TrailView *>(ctx);

		cv::Vec3f worldCoord = tv->bev2world({vert->x, vert->y});
		Float64_t worldRay[3] = { worldCoord[0], worldCoord[1], 0 };

		Float64_t camRay[3] = {0};
		World_Ray_To_Cam_Ray(camRay, worldRay, &(Camera_Model[tv->camId]->camera_model_ext));

		Float64_t imagePoint[3] = {0};
		Cam_Ray_To_Image_Point(imagePoint, camRay, &(Camera_Model[tv->camId]->camera_model_int));

		vert->x = imagePoint[1];
		vert->y = imagePoint[0];
	};

	nvgVertexTransformCallback(vg, static_cast<NVGVertexTransformCallback>(callback), static_cast<void *>(this));
	nvgTransform4(vg, glm::value_ptr(transformMatrix));

	nvgBeginPath(vg);

#if 0
	float lineLength = wheelTrailParam.length / lut->header.mm_per_pixel_h;
	float lineWidth = wheelTrailParam.width / lut->header.mm_per_pixel_w;

	// 轨迹线起点
	float rearLeftWheelArcStartX = carRect.x;
	float rearLeftWheelArcStartY = carRect.br().y + wheelTrailParam.distanceToBumper / lut->header.mm_per_pixel_h;

	float dxRearLeftWheelApex = rearLeftWheelArcStartX - apexX;
	float dyRearLeftWheelApex = rearLeftWheelArcStartY - apexY;
	float rearLeftWheelR = std::sqrt(
		dxRearLeftWheelApex * dxRearLeftWheelApex + dyRearLeftWheelApex * dyRearLeftWheelApex);

	// 轨迹线圆弧的起始角度和终止角度
	float rearLeftWheelArcStartAngle = std::atan(
		std::abs(rearLeftWheelArcStartY - apexY) / std::abs(rearLeftWheelArcStartX - apexX));
	float rearLeftWheelArcEndAngle = rearLeftWheelArcStartAngle + lineLength / rearLeftWheelR;


	float rearRightWheelArcStartX = carRect.br().x;
	float rearRightWheelArcStartY = rearLeftWheelArcStartY;

	float dxRearRightWheelApex = rearRightWheelArcStartX - apexX;
	float dyRearRightWheelApex = rearRightWheelArcStartY - apexY;
	float rearRightWheelR = std::sqrt(
		dxRearRightWheelApex * dxRearRightWheelApex + dyRearRightWheelApex * dyRearRightWheelApex);

	float rearRightWheelArcStartAngle = std::atan(
		std::abs(rearRightWheelArcStartY - apexY) / std::abs(rearRightWheelArcStartX - apexX));
	float rearRightWheelArcEndAngle = rearRightWheelArcStartAngle + lineLength / rearRightWheelR;

	float rearLeftWheelArcEndX, rearLeftWheelArcEndY;
	float rearRightWheelArcEndX, rearRightWheelArcEndY;
	if (degree > 0)
	{
		rearLeftWheelArcEndX = apexX + std::cos(rearLeftWheelArcEndAngle) * rearLeftWheelR;
		rearLeftWheelArcEndY = apexY + std::sin(rearLeftWheelArcEndAngle) * rearLeftWheelR;
		rearRightWheelArcEndX = apexX + std::cos(rearRightWheelArcEndAngle) * rearRightWheelR;
		rearRightWheelArcEndY = apexY + std::sin(rearRightWheelArcEndAngle) * rearRightWheelR;

		nvgMoveTo(vg, rearLeftWheelArcStartX, rearLeftWheelArcStartY);
		nvgArc(vg, apexX, apexY, rearLeftWheelR, rearLeftWheelArcStartAngle, rearLeftWheelArcEndAngle, NVG_CW);
		nvgLineTo(vg, rearRightWheelArcEndX, rearRightWheelArcEndY);
		nvgArc(vg, apexX, apexY, rearRightWheelR, rearRightWheelArcEndAngle, rearRightWheelArcStartAngle, NVG_CCW);
	}
	else
	{
		rearLeftWheelArcEndX = apexX + std::cos(M_PI - rearLeftWheelArcEndAngle) * rearLeftWheelR;
		rearLeftWheelArcEndY = apexY + std::sin(M_PI - rearLeftWheelArcEndAngle) * rearLeftWheelR;
		rearRightWheelArcEndX = apexX + std::cos(M_PI - rearRightWheelArcEndAngle) * rearRightWheelR;
		rearRightWheelArcEndY = apexY + std::sin(M_PI - rearRightWheelArcEndAngle) * rearRightWheelR;

		nvgMoveTo(vg, rearLeftWheelArcStartX, rearLeftWheelArcStartY);
		nvgArc(vg, apexX, apexY, rearLeftWheelR, M_PI - rearLeftWheelArcStartAngle, M_PI - rearLeftWheelArcEndAngle, NVG_CCW);
		nvgLineTo(vg, rearRightWheelArcEndX, rearRightWheelArcEndY);
		nvgArc(vg, apexX, apexY, rearRightWheelR, M_PI - rearRightWheelArcEndAngle, M_PI - rearRightWheelArcStartAngle, NVG_CW);
	}
	nvgClosePath(vg);
	nvgFillColor(vg, nvgRGBAf(1, 0, 0, 1));
	nvgFill(vg);
#endif

	nvgBeginPath(vg);
	nvgArc(vg, apexX, apexY,
		rearLeftCarTrail.arcR,
		rearLeftCarTrail.arcStartAngle,
		rearLeftCarTrail.arcEndAngle,
		rearLeftCarTrail.arcDir);
	nvgLineTo(vg, rearRightCarTrail.arcEndX, rearRightCarTrail.arcEndY);
	nvgArc(vg, apexX, apexY,
		rearRightCarTrail.arcR,
		rearRightCarTrail.arcEndAngle,
		rearRightCarTrail.arcStartAngle,
		invertArcDir(rearLeftCarTrail.arcDir));

	int numSegments = 20;
	float stepX = (rearLeftCarTrail.arcStartX - rearRightCarTrail.arcStartX) / numSegments;
	float stepY = (rearLeftCarTrail.arcStartY - rearRightCarTrail.arcStartY) / numSegments;
	for (int i = 1; i <= numSegments; ++i)
	{
		nvgLineTo(vg, rearRightCarTrail.arcStartX + stepX * i, rearRightCarTrail.arcStartY + stepY * i);
	}

	nvgClosePath(vg);
	nvgFillColor(vg, filledTrailColor);
	nvgFill(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void drawLineFEV(NVGcontext* vg, float startX, float startY, float endX, float endY, int numSegments)
{
	float stepX = (endX - startX) / numSegments;
	float stepY = (endY - startY) / numSegments;
	for (int i = 1; i <= numSegments; ++i)
	{
		nvgLineTo(vg, startX + stepX * i, startY + stepY * i);
	}
}

void TrailView::drawFilledTrail0(int x, int y, int width, int height, const glm::mat4& transform)
{
	auto carRect = lut->header.carRealRect;
	carRect.x -= (vehicleWidthWithMirror - carRect.width) / 2;
	carRect.width = vehicleWidthWithMirror;

	auto vg = vgCtx.get();

	glViewport(x, y, width, height);
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);
	nvgSave(vg);

	auto callback = [](void* ctx, NVGvertex* vert)
	{
		auto tv = reinterpret_cast<TrailView*>(ctx);
		std::tie(vert->x, vert->y) = tv->bev2fev(vert->x, vert->y);
	};
	nvgTransform4(vg, glm::value_ptr(transform));
	nvgVertexTransformCallback(vg, callback, static_cast<void*>(this));

	float offsetX = carTrailParam.width / lut->header.mm_per_pixel_w / 2;
	float offsetY = carTrailParam.distanceToBumper / lut->header.mm_per_pixel_h;
	float lineLength = carTrailParam.length / lut->header.mm_per_pixel_h;
	float lineWidth = carTrailParam.width / lut->header.mm_per_pixel_w;

	float leftStartX = carRect.x + offsetX,
		leftStartY = carRect.br().y + offsetY;
	float leftEndX = leftStartX,
		leftEndY =  leftStartY + lineLength;

	float rightStartX = carRect.br().x - offsetX,
		rightStartY = leftStartY;
	float rightEndX = rightStartX,
		rightEndY = leftEndY;

	int numSegments = 10;
	nvgBeginPath(vg);

	nvgMoveTo(vg, leftStartX, leftStartY);
	drawLineFEV(vg, leftStartX, leftStartY, leftEndX, leftEndY, numSegments);
	drawLineFEV(vg, leftEndX, leftEndY, rightEndX, rightEndY, numSegments);
	drawLineFEV(vg, rightEndX, rightEndY, rightStartX, rightStartY, numSegments);
	drawLineFEV(vg, rightStartX, rightStartY, leftStartX, leftStartY, numSegments);

	nvgFillColor(vg, filledTrailColor);
	nvgFill(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void TrailView::drawBEV(float degree, int x, int y, int width, int height, const glm::mat4& transform)
{
	glm::mat4 transformMatrix = transform;
	if (std::round(degree) == 0)
	{
		drawBEV0(x, y, width, height, transformMatrix);
	}
	if (degree < 0)
	{
		// 转弯角为负时，即向右转时，轨迹是左转当前角度绝对值时的镜像
		// 这样子能用是基于车轴肯定位于车的中线上，如果不是，那你该问问车企为什么做个不对称的轴
		transformMatrix = glm::scale(transformMatrix, glm::vec3(-1, 1, 1));
		degree = std::abs(degree);
	}
	float radian = nvgDegToRad(degree);

	auto carRect = lut->header.car_Icon_Rect;

	// 车框的盲区暂时较大，先屏蔽直接按照车框来算，不然显示的比较难看
	// carRect.x -= (vehicleWidth - carRect.width) / 2;
	// carRect.width = vehicleWidth;

	// 前后轴中心
	float frontAxleCenterX = carRect.x + carRect.width / 2.0,
		frontAxleCenterY = carRect.y + frontOverhangLength;
	float rearAxleCenterX = frontAxleCenterX,
		rearAxleCenterY = carRect.br().y - rearOverhangLength;

	// 四个轮子的坐标（当轨迹线为车框四个角时，就是车框四个角的坐标）
	float frontLeftWheelX = carRect.x,
		frontLeftWheelY = carRect.y;
	float frontRightWheelX = carRect.br().x,
		frontRightWheelY = carRect.y;
	float rearLeftWheelX = carRect.x,
		rearLeftWheelY = carRect.br().y;
	float rearRightWheelX = carRect.br().x,
		rearRightWheelY = carRect.br().y;

	// 弯心坐标
	float apexX = rearAxleCenterX - wheelBase / std::tan(radian),
		apexY = rearAxleCenterY;

	// 两后轮转弯半径

//	float rearLeftWheelR = wheelBase / std::tan(radian) - rearWheelTread / 2.0;
//	float rearRightWheelR = rearLeftWheelR + rearWheelTread;

	float dxRearLeftWheelApex = rearLeftWheelX - apexX;
	float dyRearLeftWheelApex = rearLeftWheelY - apexY;
	float rearLeftWheelR = std::sqrt(
		dxRearLeftWheelApex * dxRearLeftWheelApex + dyRearLeftWheelApex * dyRearLeftWheelApex);

	float dxRearRightWheelApex = rearRightWheelX - apexX;
	float dyRearRightWheelApex = rearRightWheelY - apexY;
	float rearRightWheelR = std::sqrt(
		dxRearRightWheelApex * dxRearRightWheelApex + dyRearRightWheelApex * dyRearRightWheelApex);

	// 两前轮转弯半径
	float dxFrontLeftWheelApex = frontLeftWheelX - apexX;
	float dyFrontLeftWheelApex = frontLeftWheelY - apexY;
	float frontLeftWheelR = std::sqrt(
		dxFrontLeftWheelApex * dxFrontLeftWheelApex + dyFrontLeftWheelApex * dyFrontLeftWheelApex);

	float dxFrontRightWheelApex = frontRightWheelX - apexX;
	float dyFrontRightWheelApex = frontRightWheelY - apexY;
	float frontRightWheelR = std::sqrt(
		dxFrontRightWheelApex * dxFrontRightWheelApex + dyFrontRightWheelApex * dyFrontRightWheelApex);

	auto vg = vgCtx.get();
	glViewport(x, y, width, height);
	nvgBeginFrame(vg, lut->header.bev_img_width, lut->header.bev_img_height, 1.0);
	nvgSave(vg);

	auto carIcon = lut->header.car_Icon_Rect;

	nvgTransform4(vg, glm::value_ptr(transformMatrix));

#if 0
	nvgBeginPath(vg);

	nvgCircle(vg, apexX, apexY, 10);

	// 各轮到弯心
	nvgMoveTo(vg, apexX, apexY);
	nvgLineTo(vg, frontLeftWheelX, frontLeftWheelY);
	nvgMoveTo(vg, apexX, apexY);
	nvgLineTo(vg, frontRightWheelX, frontRightWheelY);
	nvgMoveTo(vg, apexX, apexY);
	nvgLineTo(vg, rearLeftWheelX, rearLeftWheelY);
	nvgMoveTo(vg, apexX, apexY);
	nvgLineTo(vg, rearRightWheelX, rearRightWheelY);

	//画轴
	nvgMoveTo(vg, frontLeftWheelX, frontLeftWheelY);
	nvgLineTo(vg, frontRightWheelX, frontRightWheelY);
	nvgMoveTo(vg, rearLeftWheelX, rearLeftWheelY);
	nvgLineTo(vg, rearRightWheelX, rearRightWheelY);
	nvgMoveTo(vg, frontAxleCenterX, frontAxleCenterY);
	nvgLineTo(vg, rearAxleCenterX, rearAxleCenterY);

	nvgCircle(vg, frontAxleCenterX, frontAxleCenterY, 10);
	nvgCircle(vg, rearAxleCenterX, rearAxleCenterY, 10);
	nvgCircle(vg, frontLeftWheelX, frontLeftWheelY, 10);
	nvgCircle(vg, frontRightWheelX, frontRightWheelY, 10);
	nvgCircle(vg, rearLeftWheelX, rearLeftWheelY, 10);
	nvgCircle(vg, rearRightWheelX, rearRightWheelY, 10);


	nvgCircle(vg, apexX, apexY, rearLeftWheelR);
	nvgCircle(vg, apexX, apexY, rearRightWheelR);
	nvgCircle(vg, apexX, apexY, frontLeftWheelR);
	nvgCircle(vg, apexX, apexY, frontRightWheelR);

	nvgStrokeColor(vg, nvgRGBAf(1, 1, 0, 1));
	nvgStrokeWidth(vg, 2);
	nvgStroke(vg);
#endif

	int isFrontCam = (camId == const_value::FRONT_CAM ? 1 : -1);
	int arcDir = (camId == const_value::FRONT_CAM ? NVG_CCW : NVG_CW);

	float lineLength = bevTrailParam.length / lut->header.mm_per_pixel_h;
	NVGcolor startColor = bevTrailParam.color, endColor = bevTrailParam.color;
	startColor.a = bevTrailParam.alphaRange[0] / 255.0;
	endColor.a = bevTrailParam.alphaRange[1] / 255.0;
	float lineWidth = bevTrailParam.width / lut->header.mm_per_pixel_w;

#if 1
	if (isFrontCam == -1 || degree > 0)
	{
		nvgBeginPath(vg);

		float rearLeftWheelArcStartAngle = std::asin(-dyRearLeftWheelApex / rearLeftWheelR);
		float rearLeftWheelArcEndAngle = isFrontCam * rearLeftWheelArcStartAngle + lineLength / rearLeftWheelR;
		nvgMoveTo(vg, rearLeftWheelX, rearLeftWheelY);
		nvgArc(vg, apexX, apexY, rearLeftWheelR, -rearLeftWheelArcStartAngle, isFrontCam * -rearLeftWheelArcEndAngle,
		       arcDir);

		float startX = rearLeftWheelX, startY = rearLeftWheelY;
		float endX = apexX + std::cos(rearLeftWheelArcEndAngle) * rearLeftWheelR,
			endY = apexY - isFrontCam * std::sin(rearLeftWheelArcEndAngle) * rearLeftWheelR;
		NVGpaint paint = nvgLinearGradient(vg, startX, startY, endX, endY, startColor, endColor);
		nvgStrokeWidth(vg, lineWidth);
		nvgStrokePaint(vg, paint);
		nvgStroke(vg);
	}

	if (isFrontCam == -1 || degree < 0)
	{
		nvgBeginPath(vg);

		float rearRightWheelArcStartAngle = std::asin(-dyRearRightWheelApex / rearRightWheelR);
		float rearRightWheelArcEndAngle = isFrontCam * rearRightWheelArcStartAngle + lineLength / rearRightWheelR;
		nvgMoveTo(vg, rearRightWheelX, rearRightWheelY);
		nvgArc(vg, apexX, apexY, rearRightWheelR, -rearRightWheelArcStartAngle, isFrontCam * -rearRightWheelArcEndAngle,
		       arcDir);

		float startX = rearRightWheelX, startY = rearRightWheelY;
		float endX = apexX + std::cos(rearRightWheelArcEndAngle) * rearRightWheelR,
			endY = apexY - isFrontCam * std::sin(rearRightWheelArcEndAngle) * rearRightWheelR;
		NVGpaint paint = nvgLinearGradient(vg, startX, startY, endX, endY, startColor, endColor);
		nvgStrokeWidth(vg, lineWidth);
		nvgStrokePaint(vg, paint);
		nvgStroke(vg);
	}

	if (isFrontCam == 1 || degree < 0)
	{
		nvgBeginPath(vg);

		float frontLeftWheelArcStartAngle = std::asin(std::abs(dyFrontLeftWheelApex) / frontLeftWheelR);
		float frontLeftWheelArcEndAngle = frontLeftWheelArcStartAngle + isFrontCam * lineLength / frontLeftWheelR;
		nvgMoveTo(vg, frontLeftWheelX, frontLeftWheelY);
		nvgArc(vg, apexX, apexY, frontLeftWheelR, -frontLeftWheelArcStartAngle, isFrontCam * -frontLeftWheelArcEndAngle,
		       arcDir);

		float startX = frontLeftWheelX, startY = frontLeftWheelY;
		float endX = apexX + std::cos(frontLeftWheelArcEndAngle) * frontLeftWheelR,
			endY = apexY - isFrontCam * std::sin(frontLeftWheelArcEndAngle) * frontLeftWheelR;
		NVGpaint paint = nvgLinearGradient(vg, startX, startY, endX, endY, startColor, endColor);
		nvgStrokeWidth(vg, lineWidth);
		nvgStrokePaint(vg, paint);
		nvgStroke(vg);
	}

	if (isFrontCam == 1 || degree > 0)
	{
		nvgBeginPath(vg);

		float frontRightWheelArcStartAngle = std::asin(std::abs(dyFrontRightWheelApex) / frontRightWheelR);
		float frontRightWheelArcEndAngle = frontRightWheelArcStartAngle + isFrontCam * lineLength / frontRightWheelR;
		nvgMoveTo(vg, frontRightWheelX, frontRightWheelY);
		nvgArc(vg, apexX, apexY, frontRightWheelR, -frontRightWheelArcStartAngle,
		       isFrontCam * -frontRightWheelArcEndAngle, arcDir);

		float startX = frontRightWheelX, startY = frontRightWheelY;
		float endX = apexX + std::cos(frontRightWheelArcEndAngle) * frontRightWheelR,
			endY = apexY - isFrontCam * std::sin(frontRightWheelArcEndAngle) * frontRightWheelR;
		NVGpaint paint = nvgLinearGradient(vg, startX, startY, endX, endY, startColor, endColor);
		nvgStrokeWidth(vg, lineWidth);
		nvgStrokePaint(vg, paint);
		nvgStroke(vg);
	}
#endif

//	nvgBeginPath(vg);
//	nvgRect(vg, carIcon.x, carIcon.y, carIcon.width, carIcon.height);
//	nvgFillColor(vg, nvgRGBAf(0, 0, 0, 1));
//	nvgFill(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void TrailView::drawBEV0(int x, int y, int width, int height, const glm::mat4& transform)
{
	auto vg = vgCtx.get();
	glViewport(x, y, width, height);
	nvgBeginFrame(vg, lut->header.bev_img_width, lut->header.bev_img_height, 1.0);
	nvgSave(vg);

	nvgTransform4(vg, glm::value_ptr(transform));

	nvgBeginPath(vg);

	cv::Rect2f carRect = lut->header.car_Icon_Rect;
	// carRect.x -= (vehicleWidth - carRect.width) / 2;
	// carRect.width = vehicleWidth;

	float lineLength = bevTrailParam.length / lut->header.mm_per_pixel_h;

	NVGcolor startColor = bevTrailParam.color,
		endColor = bevTrailParam.color;
	startColor.a = bevTrailParam.alphaRange[0] / 255.0;
	endColor.a = bevTrailParam.alphaRange[1] / 255.0;

	NVGpaint paint;
	if (camId == const_value::FRONT_CAM)
	{
		float startY = carRect.y,
			endY = carRect.y - lineLength;
		nvgMoveTo(vg, carRect.x, startY);
		nvgLineTo(vg, carRect.x, endY);

		nvgMoveTo(vg, carRect.br().x, startY);
		nvgLineTo(vg, carRect.br().x, endY);

		paint = nvgLinearGradient(vg, carRect.x, startY, carRect.x, endY, startColor, endColor);
	}
	else
	{
		float startY = carRect.br().y,
			endY = carRect.br().y + lineLength;
		nvgMoveTo(vg, carRect.x, startY);
		nvgLineTo(vg, carRect.x, endY);

		nvgMoveTo(vg, carRect.br().x, startY);
		nvgLineTo(vg, carRect.br().x, endY);

		paint = nvgLinearGradient(vg, carRect.x, startY, carRect.x, endY, startColor, endColor);
	}

	nvgStrokeWidth(vg, bevTrailParam.width / lut->header.mm_per_pixel_w);
	nvgStrokePaint(vg, paint);
	nvgStroke(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void TrailView::drawDistanceLine(float degree, int x, int y, int width, int height, const glm::mat4& transform)
{
	auto vg = vgCtx.get();

	glViewport(x, y, width, height);
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);

	nvgSave(vg);
	nvgTransform4(vg, glm::value_ptr(transform));

	auto callback = [](void *ctx, NVGvertex *vert)
	{
		auto tv = reinterpret_cast<TrailView *>(ctx);

		cv::Vec3f worldCoord = tv->bev2world({vert->x, vert->y});
		Float64_t worldRay[3] = {worldCoord[0], worldCoord[1], 0};

		Float64_t camRay[3] = {0};
		World_Ray_To_Cam_Ray(camRay, worldRay, &(Camera_Model[tv->camId]->camera_model_ext));

		Float64_t imagePoint[3] = {0};
		Cam_Ray_To_Image_Point(imagePoint, camRay, &(Camera_Model[tv->camId]->camera_model_int));

		vert->x = imagePoint[1];
		vert->y = imagePoint[0];
	};
	nvgVertexTransformCallback(vg, static_cast<NVGVertexTransformCallback>(callback), static_cast<void*>(this));

	const DistanceLineParam& disLineParam = (camId == const_value::FRONT_CAM ? frontDisLineParam : rearDisLineParam);
	auto carRect = lut->header.carRealRect;

	nvgBeginPath(vg);

	// 前视图
	if (camId == const_value::FRONT_CAM)
	{
		float startY = carRect.y - disLineParam.distanceToBumper / lut->header.mm_per_pixel_h;
		float endY = startY - disLineParam.length / lut->header.mm_per_pixel_h;
		float horLen = disLineParam.horizontalLength / lut->header.mm_per_pixel_w;

		int numSegments = 5;
		float dy = endY - startY;
		float stepY = dy / numSegments;
		nvgMoveTo(vg, carRect.x, startY);
		for (int i = 1; i <= numSegments; ++i)
		{
			nvgLineTo(vg, carRect.x, startY + i * stepY);
		}
		float stepX = horLen / numSegments;
		for (int i = 1; i <= numSegments; ++i)
		{
			nvgLineTo(vg, carRect.x + i * stepX, endY);
		}

		nvgMoveTo(vg, carRect.br().x, startY);
		for (int i = 1; i <= numSegments; ++i)
		{
			nvgLineTo(vg, carRect.br().x, startY + i * stepY);
		}
		for (int i = 1; i <= numSegments; ++i)
		{
			nvgLineTo(vg, carRect.br().x - i * stepX, endY);
		}

	}
	else if (camId == const_value::REAR_CAM)
	{
		float horLen = disLineParam.horizontalLength / lut->header.mm_per_pixel_w;
		nvgMoveTo(vg, rearLeftDistanceTrail.arcStartX, rearLeftDistanceTrail.arcStartY);
		nvgArc(vg, rearLeftDistanceTrail.arcCX, rearLeftDistanceTrail.arcCY, rearLeftDistanceTrail.arcR,
			rearLeftDistanceTrail.arcStartAngle, rearLeftDistanceTrail.arcEndAngle,
		       rearLeftDistanceTrail.arcDir);
		nvgLineTo(vg, rearLeftDistanceTrail.arcEndX + horLen, rearLeftDistanceTrail.arcEndY);

		nvgMoveTo(vg, rearRightDistanceTrail.arcStartX, rearRightDistanceTrail.arcStartY);
		nvgArc(vg, rearRightDistanceTrail.arcCX, rearRightDistanceTrail.arcCY, rearRightDistanceTrail.arcR,
		       rearRightDistanceTrail.arcStartAngle, rearRightDistanceTrail.arcEndAngle,
		       rearRightDistanceTrail.arcDir);
		nvgLineTo(vg, rearRightDistanceTrail.arcEndX - horLen, rearRightDistanceTrail.arcEndY);
	}

	nvgStrokeWidth(vg, disLineParam.width / lut->header.mm_per_pixel_w);
	nvgStrokeColor(vg, disLineParam.color);
	nvgStroke(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);
}


void TrailView::drawDistanceLine0(int x, int y, int width, int height, const glm::mat4& transform)
{
	auto vg = vgCtx.get();
	auto carRect = lut->header.carRealRect;
	carRect.x -= (vehicleWidthWithMirror - carRect.width) / 2;
	carRect.width = vehicleWidthWithMirror;

	glViewport(x, y, width, height);
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);
	nvgSave(vg);

	auto callback = [](void* ctx, NVGvertex* vert)
	{
		auto tv = reinterpret_cast<TrailView*>(ctx);
		std::tie(vert->x, vert->y) = tv->bev2fev(vert->x, vert->y);
	};
	nvgTransform4(vg, glm::value_ptr(transform));
	nvgVertexTransformCallback(vg, callback, static_cast<void*>(this));

	float startY = carRect.br().y + rearDisLineParam.distanceToBumper / lut->header.mm_per_pixel_h;
	float endY = startY + rearDisLineParam.length / lut->header.mm_per_pixel_h;
	float horLen = rearDisLineParam.horizontalLength / lut->header.mm_per_pixel_w;

	float offsetX = carTrailParam.width / lut->header.mm_per_pixel_w / 2;

	nvgBeginPath(vg);

	int numSegments = 10;
	nvgMoveTo(vg, carRect.x + offsetX, startY);
	drawLineFEV(vg, carRect.x + offsetX, startY, carRect.x, endY, numSegments);
	nvgLineTo(vg, carRect.x + offsetX + horLen, endY);

	nvgMoveTo(vg, carRect.br().x - offsetX, startY);
	drawLineFEV(vg, carRect.br().x - offsetX, startY, carRect.br().x, endY, numSegments);
	nvgLineTo(vg, carRect.br().x - offsetX - horLen, endY);

	nvgStrokeWidth(vg, rearDisLineParam.width / lut->header.mm_per_pixel_w);
	nvgStrokeColor(vg, rearDisLineParam.color);
	nvgStroke(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);
}
