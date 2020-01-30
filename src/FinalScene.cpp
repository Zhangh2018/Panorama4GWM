#include "FinalScene.h"
#include "CameraDevice.h"
#include "const_value.h"
#include "DrawScene.h"
#include "TextureIL.h"
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include "OGLWin.h"
#include "PanoramaParam.h"
#include "Panorama3DScene.h"
#include <time.h>
#include <pthread.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

//#include "json.hpp"


#include "dump.h"
#include "CVOGLBridge.h"
#include "utils.hpp"

#include "imgproc/LuminanceEqualize.hpp"

#include "utils/Timer.hpp"

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include "trail.hpp"
#include "DistanceLine.hpp"

#include "nanovg.h"
#define NANOVG_GLES2_IMPLEMENTATION
#include "nanovg_gl.h"
#include "nanovg_gl_utils.h"

#include "sumat.hpp"
#include "umat.hpp"

float hOffset = 0.0;
float vOffset = 0.0;
float view2DScale = 1.0;


using namespace const_value;



static const float ANGLETORADIUS = 3.141592654 / 180.0;
//定义旋转动画的多个视角，旋转时，依次执行到下一个视角，并停顿一段时间，然后继续旋转
static const float constPhei = 55 * ANGLETORADIUS;
//static const float constPhei = 35 * ANGLETORADIUS;
static float gTheta[] = {0 * ANGLETORADIUS,45 * ANGLETORADIUS,90 * ANGLETORADIUS,135 * ANGLETORADIUS,180 * ANGLETORADIUS,
225 * ANGLETORADIUS,270 * ANGLETORADIUS,315 * ANGLETORADIUS};
static float gPhei[] = {constPhei,constPhei,constPhei,constPhei,constPhei,constPhei,constPhei,constPhei};
static int gViewNum = sizeof(gTheta) / sizeof(gTheta[0]);

float CFinalScene::blankColor[4] = {0.0, 0.0, 0.0, 0.0};
float CFinalScene::highlightColor[4] = {1.0, 1.0, 0.0, 0.3};

////////////////////////////////////////////////////////wyd added///////////////////////////////////////////
int SwitchChannelNum = 1;
float yangjiao=30*3.141592654 / 180.0;
float xuanzhuan=0;
int angle;
int angle_yangjiao=30;
///////////////////////////////////////////////////////////////////////////////////////////////////////////

void CFinalScene::loadTrailImages(const char* dir, int minAngle, int maxAngle)
{
	for (int i = minAngle; i <= maxAngle; ++i)
	{
		std::string filename = "/data/opengl_new/trails/trail_" + std::to_string(i) + ".png";
		std::cout << filename << std::endl;
		cv::Mat bgra = cv::imread(filename, cv::IMREAD_UNCHANGED);
		std::cout << "(" << bgra.size().width << "," << bgra.size().height << "," << bgra.channels() << ")" << std::endl;
		cv::Mat rgba;
		cv::cvtColor(bgra, rgba, cv::COLOR_BGRA2RGBA);
		GLuint texId = 0;
		glGenTextures(1, &texId);
		glBindTexture(GL_TEXTURE_2D, texId);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgba.size().width, rgba.size().height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		trailImages[i] = texId;
	}
}

CFinalScene::CFinalScene()
{
	for(int i = 0;i < CAMERANUM;i++){
		m_dataBuffer[i] = NULL;
	}
	m_endView = 0;
	m_curView = 4;

	lut = std::make_shared<CalibLUT>();
	lut->load("/data/opengl_new/LUT.bin");

	int bevWidth = 1920;
	int bevHeight = 1080;

	glGenFramebuffers(1, &fbo);
	glGenRenderbuffers(1, &rbo);

	glGenTextures(1, &dstTex);
	glBindTexture(GL_TEXTURE_2D, dstTex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, bevWidth, bevHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	// glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1080, 720, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glBindRenderbuffer(GL_RENDERBUFFER,  rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, bevWidth, bevHeight);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dstTex, 0);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rbo);

	auto status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		printf("framebuffer failed: ");
		switch (status)
		{
		case GL_FRAMEBUFFER_UNSUPPORTED:
			printf("unsupported\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
			printf("incomplete attachment\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
			printf("incomplete dimensions\n");
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
			printf("incomplete missing attchment\n");
			break;
		default:
			printf("unknown error: %d (0x%x)\n", status, status);
			break;
		}
	}

	// render to screen
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glGenFramebuffers(1, &fboPre);
	// 用于保存前一帧的数据
	glGenTextures(1, &dstTexPre);
	glBindTexture(GL_TEXTURE_2D, dstTexPre);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, bevWidth, bevHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, 0);
	std::cout << " 初始化dstTexPre: " << dstTexPre << std::endl;
	glBindFramebuffer(GL_FRAMEBUFFER, fboPre);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dstTexPre, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	setBirdEyeViewParams(*lut);
	setSingleViewParams(singleViewWidth, singleViewHeight);

	// NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG
	vgCtx = std::shared_ptr<NVGcontext>(
		nvgCreateGLES2(0),
		[this](NVGcontext* ctx)
		{
			nvgDeleteGLES2(this->vgCtx.get());
		});
	vgFontId = nvgCreateFont(vgCtx.get(), "wqy", "/data/opengl_new/WenQuanYiZenHei.ttf");

	bev = BirdEyeView(lut);
	exposureComp = ExposureCompensation("/data/opengl_new/param/exposure_compensation.toml");

	constexpr const char* singleViewConfigPaths[] = {
		"/data/opengl_new/param/single_view_front.toml",
		"/data/opengl_new/param/single_view_rear.toml",
		"/data/opengl_new/param/single_view_left.toml",
		"/data/opengl_new/param/single_view_right.toml",
		"/data/opengl_new/param/single_view_left.toml",
		"/data/opengl_new/param/single_view_right.toml"
	};

	for (int i = 0; i < const_value::CAMERANUM; ++i)
	{
		pevs[i] = PlaneExpandView(i, singleViewConfigPaths[i], "/data/opengl_new/param/vehicle.toml");
	}

	for (int i = 0; i < 2; ++i)
	{
		boxViews[i] = BoxExpandView(i, singleViewConfigPaths[i]);
		trailViews[i] = TrailView(i,
			"/data/opengl_new/param/trail_aiways.toml",
			"/data/opengl_new/param/vehicle.toml",
			vgCtx,
			lut);
		trailViews[i].update(steeringAngle);
	}
	{
		const auto cfg = toml::parse("/data/opengl_new/param/trail_aiways.toml");
		const toml::value steeringParam = toml::find<toml::table>(cfg, "steering");
		steeringRatio = toml::find<float>(steeringParam, "ratio");
		steeringDir = toml::find<float>(steeringParam, "dir");
	}
	//loadTrailImages("/data/opengl_new/trails", -45, 45);

	// initialize fisheye view
	for (int i = 0; i < 6; ++i)
	{
		const auto config = toml::parse(singleViewConfigPaths[i]);

		const toml::value cropRect = toml::find<toml::table>(config, std::string(i < 4 ? "crop_rect" : "rear_crop_rect"));
		fisheyeCropRects[i] = {
			toml::find<int>(cropRect, "x"),
			toml::find<int>(cropRect, "y"),
			toml::find<int>(cropRect, "width"),
			toml::find<int>(cropRect, "height")
		};
		fisheyeRollAngles[i] = toml::find<float>(cropRect, "roll_angle");


		if (i < 2)
		{
			const toml::value wideAngleCropRect = toml::find<toml::table>(config, "wide_angle_crop_rect");
			wideAngleCropRects[i] = {
				toml::find<int>(cropRect, "x"),
				toml::find<int>(cropRect, "y"),
				toml::find<int>(cropRect, "width"),
				toml::find<int>(cropRect, "height")
			};
		}
	}

	radar = RadarView("/data/opengl_new/param/radar.toml", vgCtx, lut);

	disLine = DistanceLineView("/data/opengl_new/param/distance_line.toml",
		"/data/opengl_new/param/vehicle.toml",
		vgCtx, lut);

	for (auto& h : isRegionHighlighted)
	{
		h = false;
	}

	// 透明车底功能
	transBotC = new TransBot(fbo, dstTexPre, dstTex, lut);
	encoderData.x = 0;
	encoderData.y = 0;
	encoderData.theta = 0;
    transBotInitFlag = true;
    transBotEncoderPreChangeFlag = false;
    encoderDataPre.x = 0;
    encoderDataPre.y = 0;
    encoderDataPre.theta = 0;
}

CFinalScene::~CFinalScene(void)
{
	delete transBotC;
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &rbo);
    glDeleteTextures(1, &dstTex);
}

void CFinalScene::onStart()
{
    //getCameraDevice().startCapture(SRC_IMAGE_WIDTH,SRC_IMAGE_HEIGHT);
    for(int i = 0;i < CAMERANUM;i++){
        m_imageTextureID[i] = createDynamicYUYVTexture2(SRC_IMAGE_WIDTH,SRC_IMAGE_HEIGHT,&m_dataBuffer[i]);
        //m_imageTextureID[i] = createDynamicYUYVTexture(SRC_IMAGE_WIDTH,SRC_IMAGE_HEIGHT,&m_dataBuffer[i]);
    }
    int colorRGB[][3] = {{139, 0, 255},{125,125,125},{0,255,0},{255,255,255},{0,0,0}};   //紫色,灰色,绿色,白色,黑色
    int curColor = g_globalParam.m_themeColor;
    setBackGround(0,0,0,255);
    beginCurrentMode(g_globalParam.m_displayMode);
    getPanorama3DScene().setPanorama3dParam(gTheta[m_curView],gPhei[m_curView]);
}

void CFinalScene::onStop()
{
    getCameraDevice().stopCapture();    //停止4路摄像头采集
    for(int i = 0; i < CAMERANUM; i++)  //释放4个g2d_buf缓冲区
    {
         delete[] m_dataBuffer[i];
    }
}
void CFinalScene::onMessage(int message)
{
    //响应键盘上的数字按键，数字表示旋转的结束视角
    if(message >= '0' && message < '0' + gViewNum){
        m_endView = (message - '0');
        m_curView = (m_curView + 1) % gViewNum;
        //printf("m_curView = %d,m_endView = %d,gViewNum = %d\n",m_curView,m_endView,gViewNum);
        getPanorama3DScene().setNextCameraViewByThetaPhei(gTheta[m_curView],gPhei[m_curView],30);
    }
}
void CFinalScene::changeView(){
    static int times = 0;
	//如果还没旋转到最终的视角，且当前的动画已经结束
    if(m_curView != m_endView && getPanorama3DScene().isAnimationCompleted()){
        times++;
        if(times >= 20){     //如果已经停止了20帧，则开始下一次旋转
            m_curView = (m_curView + 1) % gViewNum;
            //printf("m_curView = %d,m_endView = %d,gViewNum = %d\n",m_curView,m_endView,gViewNum);
            getPanorama3DScene().setNextCameraViewByThetaPhei(gTheta[m_curView],gPhei[m_curView],30);
            times = 0;
        }
    }
}

extern glm::mat4 bevTransformMatrix;
static bool rotated = false;

void CFinalScene::render2DViewRaw(int x, int y, int width, int height, void* data, const glm::mat4& transform)
{
	float* delta = static_cast<float*>(data);
	glViewport(x, y, width, height);
	bev.draw(m_imageTextureID, delta, transform);
	glViewport(0, 0, getScreenWidth(), getScreenHeight());
}

void CFinalScene::render2DView(int x, int y, int width, int height, void* data, bool hasTrail, const glm::mat4& transform)
{
	float* delta = static_cast<float*>(data);

	glViewport(x, y, width, height);

	bev.draw(m_imageTextureID, delta, transform);

	if (hasTrail)
	{
		int trailCamId = (gearStatus == GEAR_R ? const_value::REAR_CAM : const_value::FRONT_CAM);
		trailViews[trailCamId].drawBEV(steeringAngle, x, y, width, height, transform);
	}

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	Generate_Car_Icon(transform);
	glDisable(GL_BLEND);

	//std::unique_lock<std::mutex> lock{ radarSignalMutex };
	//radar.test(x, y, width, height, transform);
	radar.draw(radarSignals, x, y, width, height, transform);

	//lock.unlock();

	glViewport(0, 0, getScreenWidth(), getScreenHeight());
}

void CFinalScene::render2DViewCrop(int x, int y, int width, int height, void* data, bool hasTrail, const glm::mat4& transform)
{
	float right = 1.0 * width / lut->header.bev_img_width,
		top = 1.0 * height / lut->header.bev_img_height;

	glm::mat4 crop = glm::ortho(-right, right, -top, top);
	render2DView(x, y, width, height, data, hasTrail, transform * crop);
}

void CFinalScene::render3DView(int x, int y, int width, int height, void* data)
{
	float* delta = static_cast<float*>(data);
	getPanorama3DScene().getTrackBall()->setCurViewByAngle(yangjiao, xuanzhuan);
	getPanorama3DScene().setViewport(x, y, width, height);
	getPanorama3DScene().draw(delta);
	std::string s = "yanjiao = " + std::to_string(yangjiao) + ", xuanzhuan = " + std::to_string(xuanzhuan);
	renderTextBox(s.c_str(), 1920 / 2, 1080 / 2, 32);
}

void CFinalScene::renderSingleView(int camId,int x, int y, int width, int height, const glm::mat4& transform)
{
	glViewport(x, y, width, height);
	pevs[camId].draw(m_imageTextureID[camId], x, y, width, height, transform);
	if (camId >= 2)
	{
		disLine.draw(camId, x, y, width, height, transform);
	}
	glViewport(0, 0, getScreenWidth(), getScreenHeight());
}

void CFinalScene::renderTexView(GLint textureID, int x, int y, int width, int height,
								glm::mat4 transMatrix)
{
	glViewport(x, y, width, height);

	glm::mat4 flipXY = glm::scale(glm::mat4(1.0), glm::vec3(-1.0, 1.0, 1.0));

#if 1
	Generate_Image_View(textureID, x, y, width, height, flipXY * transMatrix);
#else
	auto vg = vgCtx.get();
	glViewport(0, 0, getScreenWidth(), getScreenHeight());
	nvgBeginFrame(vg, getScreenWidth(), getScreenHeight(), 1.0);

	nvgSave(vg);

	auto imgHandle = nvglCreateImageFromHandleGLES2(vg, textureID, width, height, NVG_IMAGE_NEAREST);
	auto imgPattern = nvgImagePattern(vg, x, y, width, height, 0, imgHandle, 1.0);

	nvgTransform4(vg, glm::value_ptr(transMatrix));
	nvgBeginPath(vg);
	nvgRect(vg, x, y, width, height);
	nvgFillPaint(vg, imgPattern);
	nvgFill(vg);

	nvgRestore(vg);
	nvgEndFrame(vg);

#endif

	// glViewport(x, y, width, height);
	// glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	// glEnable(GL_BLEND);
	// Generate_Car_Icon();
	// glDisable(GL_BLEND);

	glViewport(0, 0, getScreenWidth(), getScreenHeight());
}

void CFinalScene::copyFBOTexture(GLsizei srcWidth, GLsizei srcHeight, GLsizei srcDepth)
{
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glBindTexture(GL_TEXTURE_2D, dstTexPre);
	glCopyTexSubImage2D(GL_TEXTURE_2D, 0,0,0,0,0,srcWidth,srcHeight);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void CFinalScene::renderImage(const char* path, int x, int y, int width, int height, const glm::mat4& transform)
{
	auto vg = vgCtx.get();
	glViewport(0, 0, getScreenWidth(), getScreenHeight());
	nvgBeginFrame(vg, getScreenWidth(), getScreenHeight(), 1.0);

	nvgSave(vg);

	int imgHandle = 0;
	if (images.find(path) != images.end())
	{
	    imgHandle = images[path];
	}
	else
	{
	    imgHandle = nvgCreateImage(vg, path, NVG_IMAGE_NEAREST);
	    images[path] = imgHandle;
	}

	int imgWidth = 0, imgHeight = 0;
	nvgImageSize(vg, imgHandle, &imgWidth, &imgHeight);
	auto imgPattern = nvgImagePattern(vg, x, y, width, height, 0, imgHandle, 1.0);
	nvgBeginPath(vg);
	nvgRect(vg, x, y, width, height);
	nvgFillPaint(vg, imgPattern);
	//nvgFillColor(vg, nvgRGBAf(1, 1, 0, 0.6));
	nvgFill(vg);

	nvgRestore(vg);

	nvgEndFrame(vg);
}

void CFinalScene::renderTextBox(const char* text, int x, int y, int fontSize, int rowWidth, const NVGcolor& textColor, const NVGcolor& boxColor)
{
	int winWidth = getScreenWidth(), winHeight = getScreenHeight();
	auto vg = vgCtx.get();
	glViewport(0, 0, winWidth, winHeight);
	nvgBeginFrame(vg, winWidth, winHeight, 1);

	nvgFontFaceId(vg, vgFontId);
	nvgFontSize(vg, fontSize);
	nvgTextLineHeight(vg, 1.0);
	nvgTextLetterSpacing(vg, 1.0);

	float bounds[4] = {0};

	// TextBox 的 x y 坐标为第一行第一个字的右下角坐标，而输入的参数 x y 为 TextBox 的左上角坐标
	nvgTextBoxBounds(vg, x, y + fontSize - 2, rowWidth, text, NULL, bounds);
	nvgBeginPath(vg);
	nvgRoundedRect(vg, bounds[0] - 2, bounds[1] - 5, bounds[2] - bounds[0] + 4, bounds[3] - bounds[1] + 4, 3);
	nvgFillColor(vg, boxColor);
	nvgFill(vg);

	nvgFillColor(vg, textColor);
	nvgTextBox(vg, x, y + fontSize - 5, rowWidth, text, NULL);

	nvgEndFrame(vg);
}

void CFinalScene::renderObstacleFrames(int camId, int x, int y, int width, int height, const glm::mat4& transform)
{
	glViewport(x, y, width, height);

	auto vg = vgCtx.get();
	nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1.0);
	nvgSave(vg);

	nvgTransform4(vg, glm::value_ptr(transform));

	std::unique_lock<std::mutex> lock{ obstacleMutex };
	for (const auto& frame : obstacleFrames)
	{
		if (frame.camId == camId)
		{
			nvgBeginPath(vg);
			nvgMoveTo(vg, frame.quad[0].x, frame.quad[0].y);
			for (int i = 1; i < 4; ++i)
			{
				nvgLineTo(vg, frame.quad[i].x, frame.quad[i].y);
			}
			nvgLineTo(vg, frame.quad[0].x, frame.quad[0].y);

			nvgStrokeWidth(vg, frame.borderWidth);
			nvgStrokeColor(vg, frame.borderColor);
			nvgStroke(vg);
		}
	}
	lock.unlock();

	nvgRestore(vg);
	nvgEndFrame(vg);
}

void CFinalScene::renderFisheyeView(int camId, GLint texture, const cv::Rect2f& cropRect,
									int viewX, int viewY, int viewWidth, int viewHeight,
									bool hasTrail,
									bool showObstacle,
									const glm::mat4& transform)
{
	//float width2 = viewWidth / 2.0, height2 = viewHeight / 2.0;
	float width2 = const_value::SRC_IMAGE_WIDTH / 2.0, height2 = const_value::SRC_IMAGE_HEIGHT / 2.0;
	float left = cropRect.x / width2 - 1.0;
	float right = cropRect.br().x / width2 - 1.0;
	float top = 1.0 - cropRect.y / height2;
	float bottom = 1.0 - cropRect.br().y / height2;
	glm::mat4 crop = glm::ortho(left, right, bottom, top);
	glm::mat4 rotate = glm::rotate(glm::mat4(1.0), glm::radians(fisheyeRollAngles[camId]), glm::vec3(0, 0, 1));

	glm::mat4 newTransform = transform * rotate * crop;
	// printf("fisheye crop rect: (%f, %f, %f, %f)\n", cropRect.x, cropRect.y, cropRect.width, cropRect.height);
	//printf("fisheye ortho: (%f, %f, %f, %f)\n", left, right, bottom, top);

	//glViewport(0, 0, getScreenWidth(), getScreenHeight());
	glViewport(viewX, viewY, viewWidth, viewHeight);

	Generate_Image_View(texture, viewX, viewY, viewWidth, viewHeight, newTransform);
	//nvgDeleteImage(vg, imgHandle);
	if (hasTrail)
	{
		if (camId < 2)
		{
#if 1

			trailViews[camId].drawFisheye(steeringAngle, viewX, viewY, viewWidth, viewHeight, newTransform);

#else

			switch (camId)
			{
			case const_value::FRONT_CAM:
				trailViews[camId].drawFisheye(steeringAngle, viewX, viewY, viewWidth, viewHeight, newTransform);
				break;
			case const_value::REAR_CAM:
				glViewport(viewX, viewY, viewWidth, viewHeight);

				glEnable(GL_BLEND);
				glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glBlendEquation(GL_FUNC_ADD);
				Generate_Image_View(trailImages[steeringAngle], viewX, viewY, viewWidth, viewHeight, newTransform);
				glDisable(GL_BLEND);
				break;
			default:
				break;
			}

#endif
		}
		else
		{
			disLine.draw(camId, viewX, viewY, viewWidth, viewHeight, newTransform);
		}
	}

	if (showObstacle)
	{
		renderObstacleFrames(camId,  viewX, viewY, viewWidth, viewHeight, newTransform);
	}
}

inline void CFinalScene::setRegionHighlight(int regionId)
{
	isRegionHighlighted[regionId] = true;
}

inline void CFinalScene::unsetRegionHighlight(int regionId)
{
	isRegionHighlighted[regionId] = false;
}

extern vsdk::SMat frame_map[4];
//extern unsigned char* frameDataPtr[4];
//extern unsigned char  mem_tmp_T0[1280*720*2];
//extern unsigned char  mem_tmp_T1[1280*720*2];
//extern unsigned char  mem_tmp_T2[1280*720*2];
//extern unsigned char  mem_tmp_T3[1280*720*2];

extern int Show_Change_View_Flag;
extern char temp_buf[10];

int readflag;
void CFinalScene::renderToSreen()
{
    static int Flag_Next =0;
    static int index = 0;
    static time_t preTime = time(NULL);
    static int States_Flag = 0;
    const int nframes = 50;
	uint8_t* ptr = NULL;
    //changeView();
	vsdk::SMat databufs[4];
	cv::Mat mats[4];
	//cv::Mat databufs[4];
	//usleep(30000);
	Timer<> timer;
	timer.start();

	readflag = 1;
    for(int i = 0; i < CAMERANUM; i++)
    {
//#define IMAGE_DEBUG_MODE
		//从摄像头设备中取出一个图像缓冲区Calib_Bird_View
#ifdef IMAGE_DEBUG_MODE

 		//MatType& databuf = getCameraDevice().getBuffer(i);		//Get camera video from CCameraVedioDevice
 		MatType& databuf= getCameraDevice().getImageBuffer(i);   	//Get yuv image
 		databufs[i] = databuf.getMat(vsdk::ACCESS_READ | OAL_USAGE_CACHED);
		mats[i] = cv::Mat(cv::Size(SRC_IMAGE_WIDTH, SRC_IMAGE_HEIGHT), CV_8UC2, databufs[i].data);

 		ptr = databufs[i].data;

#else

		ptr = frame_map[i].data;   //Get camera video from VideoCaptureTask
		mats[i] = cv::Mat(cv::Size(SRC_IMAGE_WIDTH, SRC_IMAGE_HEIGHT), CV_8UC2, frame_map[i].data);

#endif

	   /**********************************************************************************/
		updateYUYVTexture2(i,m_imageTextureID[i],ptr,SRC_IMAGE_WIDTH,SRC_IMAGE_HEIGHT);


		//释放摄像头设备中取出的图像缓冲区
		#if 0
		#if defined(PLATFORM_S32V)
				getCameraDevice().giveBackBuffer(i);	//free camera video from CCameraVedioDevice
		#else
				getCameraDevice().giveBackImageBuffer(i);
		#endif
		#endif
    }
	readflag = 0;
	timer.stop();
	//std::cout << "read images: " << timer.get() << std::endl;

 	static float delta[4] = {1.0, 1.0, 1.0, 1.0};
#if defined(PLATFORM_S32V)
	timer.start();
	//exposureComp.calcExposureGainBGR(databufs, *lut, delta);
	exposureComp.calcExposureGainUYVY(mats, *lut, delta);
	timer.stop();
	std::cout << "calcExposureGain: " << timer.get() << std::endl;
#else
	exposureComp.calcExposureGainBGR(databufs, *lut, delta);
#endif

    if(g_globalParam.m_displayMode == 1)
	{
		void* data = static_cast<void*>(delta);
		#if 1
		{
			// 透明车底功能

			// 2019-06-13:
			//  实现方法:后台运行一个全屏2d拼接

			int bevWidth = 1920; //lut->header.bev_img_width;
			int bevHeight = 1080; //lut->header.bev_img_height;

			glBindFramebuffer(GL_FRAMEBUFFER, fbo);

			glm::mat4 rotate = glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3(0.0, 0.0, 1.0));
		    render2DViewRaw(0, 0, bevWidth, bevHeight, data, rotate);

            //renderTexView(dstTexPre,birdEyeViewX, birdEyeViewY, birdEyeViewWidth, birdEyeViewHeight);
            transBotC->judgeChangeFlag(encoderData, encoderDataPre, transBotEncoderPreChangeFlag, 1);

            transBotC->setTransMatrix(rotate);
		    transBotC->generateTransBot(encoderData);

            // 将当前fbo中的数据赋值到dstTexPre纹理中
            if(transBotEncoderPreChangeFlag || transBotInitFlag){

                encoderDataPre.x = encoderData.x;
                encoderDataPre.y = encoderData.y;
                encoderDataPre.theta = encoderData.theta;

                transBotC->setEncoderDataPre(encoderDataPre);
                copyFBOTexture(bevWidth, bevHeight, 0);

                if(transBotInitFlag)
                    transBotInitFlag = false;
            }

			// 将当前fbo中的数据赋值到dstTexPre纹理中
			// copyFBOTexture(getScreenWidth(), getScreenHeight(), 0);
			//renderFBOView(birdEyeViewX, birdEyeViewY, getScreenWidth(), getScreenHeight());

			glBindFramebuffer(GL_FRAMEBUFFER, 0);
		}
		#endif
		static int cnt;
		//switch (mode)

		if(SwitchChannelNum != 8)
		{
			cnt = 181;
		}

		glm::mat4 transMatrix2w3 = glm::rotate(glm::mat4(1.0), glm::radians(-90.0f), glm::vec3(0.0, 0.0, 1.0));
		auto vg = vgCtx.get();
		//nvgBeginFrame(vg, getScreenWidth(), getScreenHeight(), 1.0);

		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glBlendColor(0, 0, 0, 0);
		glClearColor(0, 0, 0, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
		xuanzhuan = 2*angle*3.141592654/180.0;
		yangjiao = angle_yangjiao*3.141592654/180.0;
		switch(SwitchChannelNum)
		{
			//case SceneMode::Scene2D:
		case 0:	//大2D
			{
				glm::mat4 rotate = glm::rotate(glm::mat4(1.0), glm::radians(90.0f), glm::vec3(0.0, 0.0, 1.0));

				// 在OpenGL坐标系下，即图像中心为 (0, 0)，裁剪 (-304, 0) 到 (304, 1080) 区域
				glm::mat4 crop = glm::ortho(-1.0 * 608 / lut->header.bev_img_width,
											 1.0 * 608 / lut->header.bev_img_height,
											 -1.0,
											 1.0);
				render2DView(0, 0, getScreenWidth(), getScreenHeight(), data, false, rotate * crop);
				break;
			}

			// 1056x1080 单视图 + 864x1080 拼接图
			case 1:    //FRONT
			{
				int camId = SwitchChannelNum - 1;
				std::cout << "fisheye crop rect = " << fisheyeCropRects[camId] << std::endl;
				renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  fisheyeCropRects[camId],
								  0, 0, 1056, 1080,
								  true,
								  true);
				render2DViewCrop(1056, 0, 864, 1080, data, true);
				break;
			}
			case 2:		//BACK
			{
				int camId = SwitchChannelNum - 1;
				std::cout << "fisheye crop rect = " << fisheyeCropRects[camId] << std::endl;
				renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  fisheyeCropRects[camId],
								  0, 0, 1056, 1080,
								  true,
								  true,
								  glm::scale(glm::mat4(1.0), glm::vec3(-1.0, 1.0, 1.0)));

				render2DViewCrop(1056, 0, 864, 1080, data, true);
				break;
			}

			case 3:		//LEFT

			{
				int camId = SwitchChannelNum - 1;
				//renderSingleView(const_value::LEFT_CAM, 0, 0, 1056, 1080);

				renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  fisheyeCropRects[camId],
								  0, 0, 1056, 1080,
								  true,
								  true);
				render2DViewCrop(1056, 0, 864, 1080, data, true);
				break;
			}
			case 4:		//RIGHT 1056x1080 单视图 + 864x1080 拼接图

			{
				int camId = SwitchChannelNum - 1;
				//renderSingleView(const_value::RIGHT_CAM, 0, 0, 1056, 1080);
				renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  fisheyeCropRects[camId],
								  0, 0, 1056, 1080,
								  true,
								  true);

				render2DViewCrop(1057, 0, 864, 1080, data, true);
				break;
			}

			// 1200x1080 拼接图
			case 5:		// 1200x1080 拼接图    扫描车位用
			{
				//renderSingleView(const_value::FRONT_CAM, 0, 0, 720, 1080, data);
				render2DView(720, 0, 1200, 1080, data);
				break;
			}


			case 6:	// 800x900 前单视图 + 720x900 拼接图
			{
				int camId = SwitchChannelNum - 6;
				//renderSingleView(const_value::LEFT_CAM, 400, 180, 800, 900);
				renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  fisheyeCropRects[camId],
								  400, 180, 800, 900,
								  true,
								  true);
					render2DViewCrop(1200, 180, 720, 900, data, true);
					break;
				}
			case 7:	// 800x900 后单视图 + 720x900 拼接图 泊车RUN状态用
			{
				int camId = SwitchChannelNum - 6;
				//renderSingleView(const_value::RIGHT_CAM, 400, 180, 800, 900);
				renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  fisheyeCropRects[camId],
								  400, 180, 800, 900,
								  true,
								  true,
								  glm::scale(glm::mat4(1.0), glm::vec3(-1.0, 1.0, 1.0)));
				render2DViewCrop(1200, 180, 720, 900, data, true);
				break;
			}
			case 8:// 1056x1080 前近距 + 864x1080 拼接图
			{
				float translateY = (-2.4);
				float scale = 1.0 * lut->header.bev_img_width / (lut->header.car_Icon_Rect.width + 100);
				glm::mat4 zoom = glm::mat4(1.0);
				zoom = glm::translate(zoom, glm::vec3(0.0, translateY, 0.0));
				zoom = glm::scale(zoom, glm::vec3(scale, scale, 0.0));

				render2DView(0, 0, 1056, 1080, data, false, zoom);
				disLine.draw(const_value::FRONT_CAM, 0, 0, 1056, 1080, zoom);

				render2DViewCrop(1056, 0, 864, 1080, data, true);
				break;
		   }
			case 9:  //1056x1080 后近距 + 864x1080 拼接图
			{
				float translateY = (2.4);
				float scale = 1.0 * lut->header.bev_img_width / (lut->header.car_Icon_Rect.width + 100);
				glm::mat4 zoom = glm::mat4(1.0);
				zoom = glm::translate(zoom, glm::vec3(0.0, translateY, 0.0));
				zoom = glm::scale(zoom, glm::vec3(scale, scale, 0.0));

				render2DView(0, 0, 1056, 1080, data, false, zoom);
				disLine.draw(const_value::REAR_CAM, 0, 0, 1056, 1080, zoom);

				bevTransformMatrix = glm::mat4(1.0);

				render2DViewCrop(1056, 0, 864, 1080, data, true);
				break;
			}
			case 10:	//LEFT AND RIGHT 前轮 左+右近距
			{
				/*
				glm::mat4 leftTransform = glm::ortho(0, 1, -1, 1);
				renderSingleView(const_value::LEFT_CAM, 0, 0, 528, 1080, leftTransform);

				glm::mat4 rightTransform = glm::ortho(-1, 0, -1, 1);
				renderSingleView(const_value::RIGHT_CAM, 528, 0, 528, 1080, rightTransform);
				*/
				renderFisheyeView(const_value::LEFT_CAM,
								  m_imageTextureID[const_value::LEFT_CAM],
								  fisheyeCropRects[const_value::LEFT_CAM],
								  0, 0, 528, 1080,
								  true);

				renderFisheyeView(const_value::RIGHT_CAM,
								  m_imageTextureID[const_value::RIGHT_CAM],
								  fisheyeCropRects[const_value::RIGHT_CAM],
								  528, 0, 528, 1080,
								  true);
				render2DViewCrop(1056, 0, 864, 1080, data, true);
				break;
			}

			case 11:  //大3D
			{
				Timer<> timer;
				timer.start();
				if(cnt <= 180)
				{
					cnt++;
					xuanzhuan = 2*cnt*3.141592654 / 180.0;
				}
				getPanorama3DScene().setViewMode(Panorama3DScene::VIEW_MODE_TRACKBALL);
				render3DView(0, 0, getScreenWidth(), getScreenHeight(), data);
				timer.stop();
				//std::cout << "render3DView: " << timer.get() << std::endl;
			}
				break;

			case 12: //2D+3D
				getPanorama3DScene().setViewMode(Panorama3DScene::VIEW_MODE_TRACKBALL);
				bevTransformMatrix = glm::mat4(1.0);
				render3DView(0, 0, 1056, 1080, data);
				render2DViewCrop(1056, 0, 864, 1080, data);
				break;


			case 13:	// 前左 3D A柱盲区
			case 14:	// 前右 3D A柱盲区
			case 15:	// 后左 3D B柱盲区
			case 16:	// 后右 3D B柱盲区
				getPanorama3DScene().setViewMode(Panorama3DScene::VIEW_MODE_PRESET);
				getPanorama3DScene().setViewPreset(SwitchChannelNum - 13);
				render3DView(0, 0, 1056, 1080, data);
				render2DViewCrop(1056, 0, 864, 1080, data);
				break;

			case 17:	// 前广角
				{
					int camId = SwitchChannelNum - 17;
					renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  wideAngleCropRects[camId],
								  0, 0, getScreenWidth(), getScreenHeight()
					);
					break;
				}
			case 18:	// 后广角
			{
				int camId = SwitchChannelNum - 17;
				renderFisheyeView(camId,
								  m_imageTextureID[camId],
								  wideAngleCropRects[camId],
								  0, 0, getScreenWidth(), getScreenHeight(),
								  false,
								  false,
								  glm::scale(glm::mat4(1.0), glm::vec3(-1.0, 1.0, 1.0))
				);
				break;
			}
			case 19:	// 透明车底
			{
				renderTexView(dstTex, 720, 0, 1200, 1080, transMatrix2w3);
				break;
			}
			case 20:	// 后轮的左右近距
			{
				renderFisheyeView(const_value::LEFT_CAM,
								  m_imageTextureID[const_value::LEFT_CAM],
								  fisheyeCropRects[const_value::LEFT_CAM + 2],
								  0, 0, 528, 1080);

				renderFisheyeView(const_value::RIGHT_CAM,
								  m_imageTextureID[const_value::RIGHT_CAM],
								  fisheyeCropRects[const_value::RIGHT_CAM + 2],
								  528, 0, 528, 1080);
				break;
			}
			default:
				break;
		}
		glViewport(0, 0, getScreenWidth(), getScreenHeight());
    }
}

void CFinalScene::endPreviousMode(int mode)
{
}
void CFinalScene::beginCurrentMode(int mode)
{
    if(g_globalParam.m_displayMode == 1)
    {
        clearForeGround();
        getPanorama3DScene().initDrawScene(m_imageTextureID);
    }
}
void CFinalScene::onButtonUp(int times)
{
    if(g_globalParam.m_displayMode == 1)
    {
    	getPanorama3DScene().getTrackBall()->incrementPhei(-ADJUST_TIMES * times);
    }
}
void CFinalScene::onButtonDown(int times)
{
    onButtonUp(-times);
}

void CFinalScene::onButtonLeft(int times)
{
    if(g_globalParam.m_displayMode == 1)
    {
		getPanorama3DScene().getTrackBall()->incrementTheta(-ADJUST_TIMES * times);
    }
}
void CFinalScene::onButtonRight(int times)
{
    onButtonLeft(-times);
}
void CFinalScene::onButtonOK()
{
    endPreviousMode(g_globalParam.m_displayMode);
    g_globalParam.m_displayMode = (g_globalParam.m_displayMode + 1) % 3;
    beginCurrentMode(g_globalParam.m_displayMode);
}

void CFinalScene::onButtonZoomIn(int times)
{
	view2DScale += times * 0.1;
}


void CFinalScene::onButtonZoomOut(int times)
{
	if (view2DScale - times * 0.1 >= 1.0f)
	{
		view2DScale -= times * 0.1;
	}
}

void CFinalScene::onButtonMoveLeft(int times)
{
	hOffset -= 0.1;
}

void CFinalScene::onButtonMoveRight(int times)
{
	hOffset += 0.1;
}

void CFinalScene::onButtonMoveUp(int times)
{
	vOffset += 0.1;
}

void CFinalScene::onButtonMoveDown(int times)
{
	vOffset -= 0.1;
}

void CFinalScene::onButtonSwitchMode(SceneMode mode)
{
	this->mode = mode;
}
