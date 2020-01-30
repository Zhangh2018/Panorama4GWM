#ifndef _FINAL_SCENE_H__
#define _FINAL_SCENE_H__

#include <memory>
#include "nanovg.h"

#include "const_value.h"
#include "GLES2/gl2.h"
#include "opencv2/core.hpp"
#include "DrawScene.h"
#include "OGLWin.h"
//代表最终展示的界面类，初始展示时调用onStart，结束展示时调用onStop，并可以根据用户的输入执行不同的控制代码
//使用renderToSreen渲染界面

#include "LUT.h"
#include "utils.hpp"

#include "TransBot.h"
#include "BirdEyeView.hpp"
#include "PlaneExpandView.hpp"
#include "BoxExpandView.hpp"
#include "imgproc/smooth.hpp"
#include "TrailView.hpp"
#include "RadarView.hpp"
#include "DistanceLine.hpp"

#include <thread>
#include <mutex>

enum class SceneMode
{
	Scene2D,
	Scene3D,
	SceneSingle,
	Scene2DWith3D,
	Scene2DWithSingle,
	Scene2DWithZoomFront,
	Scene2DWithZoomRear,
	Scene2DWithZoomLeft,
	Scene2DWithZoomRight,
	Scene2DWithZoomLeftRight
};

enum GearStatus
{
	GEAR_N = 0,
	//GEAR_P = 0x02,
	GEAR_D = 1,
	GEAR_R = 2
};

extern int SwitchChannelNum;

class CFinalScene
{
public:
	CFinalScene();
	~CFinalScene(void);
public:
    void changeView();
    void onStart();
    void onStop();
    void renderToSreen();              //渲染界面
	void onMessage(int message);       //响应消息
	void onButtonUp(int times);
	void onButtonDown(int times);
	void onButtonLeft(int times);
	void onButtonRight(int times);
	void onButtonOK();
	void endPreviousMode(int mode);
	void beginCurrentMode(int mode);
	void switchMode();

	void onButtonZoomIn(int times);
	void onButtonZoomOut(int times);

	void onButtonMoveLeft(int times);
	void onButtonMoveRight(int times);
	void onButtonMoveUp(int times);
	void onButtonMoveDown(int times);

	void onButtonNextMode()
	{
		if (SwitchChannelNum >= 20)
		{
			SwitchChannelNum = 0;
		}
		else
		{
			++SwitchChannelNum;
		}
	}

	void onButtonPrevMode()
	{
		if (SwitchChannelNum <= 0)
		{
			SwitchChannelNum = 20;
		}
		else
		{
			--SwitchChannelNum;
		}
	}

	void onButtonTurnLeft(int times)
	{
		setSteeringAngle(clamp(steeringAngle - steeringStep, minAngle, maxAngle));
	}

	void onButtonTurnRight(int times)
	{
		setSteeringAngle(clamp(steeringAngle + steeringStep, minAngle, maxAngle));
	}

	void onButtonSwitchMode(SceneMode mode);

	inline void setBirdEyeViewport(int x, int y, int width, int height)
	{
		birdEyeViewX = x;
		birdEyeViewY = y;
		birdEyeViewWidth = width;
		birdEyeViewHeight = height;
		setBirdEyeViewParams(*lut);
	}

	inline void setSingleViewport(int x, int y, int width, int height)
	{
		singleViewX = x;
		singleViewY = y;
		singleViewWidth = width;
		singleViewHeight = height;
		//setSingleViewParams(singleViewWidth, singleViewHeight);
		setSingleViewParams(1110, 810);
	}

	inline void set3DViewport(int x, int y, int width, int height)
	{
		threeDViewX = x;
		threeDViewY = y;
		threeDViewWidth = width;
		threeDViewHeight = height;
	}

	inline void setSteeringAngle(float steeringAngle)
	{
		this->steeringAngle = steeringAngle;
		for (int i = 0; i < 2; ++i)
		{
			trailViews[i].update(this->steeringAngle);
		}
		printf("steering angle = %.2f\n", this->steeringAngle);
	}

	inline void setSteeringAngleBySteeringWheel(float steeringWheelAngle)
	{
		setSteeringAngle(steeringDir * steeringWheelAngle / steeringRatio);
	}

	inline void setGearStatus(int gear)
	{
		this->gearStatus = (GearStatus)gear;
	}

	/* 只渲染2D拼接图，不带车 icon，不带雷达图和轨迹线 */
	void render2DViewRaw(int x, int y, int width, int height, void* data, const glm::mat4& transform = glm::mat4(1.0));

	void render2DView(int x, int y, int width, int height, void* data, bool hasTrail = false, const glm::mat4& transform = glm::mat4(1.0));

	/**
	 * 渲染剪裁后的 2D 拼接图
	 * viewport 为 (x, y, width, height)，剪裁大小为从图像中心出发剪裁 (width, height) 大小的图
	 * 注意：剪裁只用 ortho projection 来实现的，并且变换发生在 transform 之前
	 */
	void render2DViewCrop(int x, int y, int width, int height, void* data, bool hasTrail = false, const glm::mat4& transform = glm::mat4(1.0));

	void render3DView(int x, int y, int width, int height, void* data);
	void renderSingleView(int camID, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void renderImage(const char* path, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void renderImage(GLuint tex, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void renderTexView(GLint textureID, int x, int y, int width, int height, glm::mat4 transMatrix = glm::mat4(1.0));

	void renderFisheyeView(int camId, GLint texture, const cv::Rect2f& cropRect,
						   int viewX, int viewY, int viewWidth, int viewHeight,
						   bool hasTrail = false,
						   bool showObstacle = false,
						   const glm::mat4& transform = glm::mat4(1.0));

	void loadTrailImages(const char* dir, int minAngle, int maxAngle);

	/**
	 * 渲染文本框
	 *
	 * @param text 要渲染的文字，编码必须为 UTF-8
	 * @param x 文本框左上角位置 (坐标原点在屏幕左上角)
	 * @param y
	 * @param rowWidth 行宽，文本超过行宽会换行。(默认为当前屏幕宽度)
	 * @param fontSize 字体大小
	 * @param textColor 字体颜色 (RGBA) (默认不透明白色)
	 * @param boxColor 文本框的填充色 (RGBA) (默认全透明色)
	 */
	void renderTextBox(const char* text, int x, int y, int fontSize,
					  int rowWidth = getScreenWidth(),
					  const NVGcolor& textColor = nvgRGBAf(1, 1, 1, 1),
					  const NVGcolor& boxColor = nvgRGBAf(1, 1, 1, 0));
	/**
	 * 添加障碍物框
	 *
	 * @param quad 障碍物框的四个顶点，逆时针或顺时针排列
	 * @param borderColor 边框颜色 (RGBA, 默认黄色不透明)
	 * @param borderWidth 边框宽度 (像素，默认 4 个像素)
	 */
	inline void addObstacleFrame(int camId, const std::array<cv::Point, 4>& quad,
				 const NVGcolor& borderColor = nvgRGBAf(1, 1, 0, 1),
				 int borderWidth = 4)
	{
		std::unique_lock<std::mutex> lock{ obstacleMutex };
		obstacleFrames.emplace_back(camId, quad, borderColor, borderWidth);
	}

	/**
	 * 清空障碍物框
	 */
	inline void clearObstacleFrames()
	{
		std::unique_lock<std::mutex> lock{ obstacleMutex };
		obstacleFrames.clear();
	}

	void copyFBOTexture(GLsizei srcWidth, GLsizei srcHeight, GLsizei srcDepth);

	void setRegionHighlight(int regionId);
	void unsetRegionHighlight(int regionId);
	// 透明车底
	// 2. 里程计数据
	Encoder encoderData;
	// 3. 透明车底类
    bool reset;

	// 轨迹线转向
	float steeringRatio = 13.44;
	float steeringDir = -1;
	float steeringAngle = minAngle;
	GearStatus gearStatus;

	float radarSignals[16];

private:
    int m_endView;
    int m_curView;
    float m_phei[4];
    float m_theta[4];
    GLuint m_imageTextureID[const_value::CAMERANUM];
    GLvoid *m_dataBuffer[const_value::CAMERANUM];
    static const int ADJUST_TIMES = 8;

    GLuint fbo, rbo, dstTex;

	static const int lumEqualRate = 15;

	std::shared_ptr<CalibLUT> lut;

	std::shared_ptr<NVGcontext> vgCtx;

	//SceneMode mode = SceneMode::Scene2DWith3D;
	SceneMode mode = SceneMode::Scene2DWithSingle;
	int switchModeNum = 0;

	const float minAngle = -31.0;
	const float maxAngle = 31.0;
	float steeringStep = 1.0;

	bool isRegionHighlighted[const_value::REGION_NUM];
	static float highlightColor[4];	// RGBA
	static float blankColor[4];

	int dynamicLineFlag = 31;

    GLuint Calib_BirdView_TextureID[1];

	int birdEyeViewX, birdEyeViewY, birdEyeViewWidth, birdEyeViewHeight;
	int singleViewX, singleViewY, singleViewWidth, singleViewHeight;
	int threeDViewX, threeDViewY, threeDViewWidth, threeDViewHeight;

	BirdEyeView bev;
	PlaneExpandView pevs[const_value::CAMERANUM];
	BoxExpandView boxViews[2];
	TrailView trailViews[2];

	// 0 = front camera
	// 1 = rear camera
	// 2 = left camera look at front wheel
	// 3 = right camera look at front wheel
	// 4 = left camera look at rear wheel
	// 5 = right camera look at rear wheel
	cv::Rect2f fisheyeCropRects[6];

	float fisheyeRollAngles[6];
	cv::Rect2f wideAngleCropRects[2];

	ExposureCompensation exposureComp;

	DistanceLineView disLine;

	RadarView radar;
//	float radarSignals[16];

	std::map<int, GLuint> trailImages;

	int vgFontId;

	std::map<std::string, int> images;

	// 障碍物框
	struct ObstacleFrame
	{
		using Quad = std::array<cv::Point, 4>;
		Quad quad;
		NVGcolor borderColor;
		int borderWidth;
		int camId;

		ObstacleFrame(int camId, const Quad& quad, const NVGcolor& color, int width)
			: camId(camId), quad(quad), borderColor(color), borderWidth(width)
		{
		}
	};
	std::mutex obstacleMutex;
	std::vector<ObstacleFrame> obstacleFrames;
	void renderObstacleFrames(int camId, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));

	// 透明车底
	// 1. 前一帧的帧缓存
	GLuint fboPre, dstTexPre;
	// 2. 里程计数据
	//Encoder encoderData;
	// 3. 透明车底类
	TransBot *transBotC;
    //bool reset;

	// 2019-08-12 透明车底改进: 用多帧前的数据做补偿
    bool transBotInitFlag;
    bool transBotEncoderPreChangeFlag;
    Encoder encoderDataPre;
};

#endif
