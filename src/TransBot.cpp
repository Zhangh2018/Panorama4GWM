/***********************************************************************
 *
 * Mario Created on   : 2019-06-11 18:01
 * Filename      : TransBot.cpp
 * Function      : 透明车底功能函数
 *
 *************************************************************************/

# pragma once
#include <fstream>
#include "TransBot.h"

//#define __DEBUG__

void TransBot::setTransMatrix(const glm::mat4 bevTransformMatrixt){
    bevTransformMatrix = bevTransformMatrixt;
}

void TransBot::transCur2Pre(const float& p0x, const float& p0y, const cv::Mat_<float> TAB, float& pos0X, float& pos0Y){

    cv::Mat_<float> CurPose = cv::Mat_<float>::zeros(3,1);
    CurPose << p0x, p0y, 1;
    cv::Mat_<float> PrePose = cv::Mat_<float>::zeros(3,1);

    // TEO: opengl 到 encoder 坐标转换
    cv::Mat_<float> TEO = cv::Mat_<float>::eye(3,3);
    TEO(0,2) = - 0.5;           // 图像中心位置
    //TEO(1,2) = - 391/1080.0;    // 291+1m 后轴中心位置
    TEO(1,2) = - (rowOffset/factor + 100)/wholeRows;    // 291+1m 后轴中心位置
    cv::Mat_<float> TOE = cv::Mat_<float>::eye(3,3);
    cv::invert(TEO, TOE);

    // ** IMPORTANT **: 从当前坐标转换到前一帧坐标
    PrePose = TOE * TAB * TEO * CurPose;

    pos0X = PrePose(0,0);
    pos0Y = PrePose(1,0);

#ifdef __DEBUG__
    std::cout << "## DEBUG: " << p0x << ", " << p0y << ", " << pos0X << ", " << pos0Y << std::endl;
#endif
}

TransBot::TransBot(GLuint fbot,
				   GLuint preTextureIDt,
				   GLuint currentTextureIDt,
				   std::shared_ptr<CalibLUT> lut) :
	lut(lut),
	fbo(fbot),
	preTextureID(preTextureIDt),
	currentTextureID(currentTextureIDt)
{
	wholeCols = lut->header.bev_img_width;
	wholeRows = lut->header.bev_img_height;
	colOffset = lut->header.car_Icon_Rect.x;
	rowOffset = lut->header.car_Icon_Rect.y;

	// OPENGL 初始化
	bevTransformMatrix = glm::mat4(1.0);

	if(loadShaders("/data/opengl_new/shader/transparent.vert", "/data/opengl_new/shader/transparent.frag", program)!= 0)
		printf("return error : Generate_Tranparent_Bottom()!!!!!!!!!!!!!!!!!!!\n");
	else
		printf("return success : Generate_Tranparent_Bottom()!!!!!!!!!!!!!!!!!!!\n");

	TextureID1 = glGetUniformLocation(program, "uTexture1");
	TextureID2 = glGetUniformLocation(program, "uTexture2");

	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_DYNAMIC_DRAW);

	glGenBuffers(1, &uvbufferOrigin);
	glBindBuffer(GL_ARRAY_BUFFER, uvbufferOrigin);
	glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data_origin), g_uv_buffer_data_origin, GL_DYNAMIC_DRAW);

	m_transformMatrixHandle = glGetUniformLocation(program, "uTransformMatrix");

	m_minXHandle = glGetUniformLocation(program, "uminX");
	m_maxXHandle = glGetUniformLocation(program, "umaxX");
	m_minYHandle = glGetUniformLocation(program, "uminY");
	m_maxYHandle = glGetUniformLocation(program, "umaxY");

	m_minXFHandle = glGetUniformLocation(program, "uminXF");
	m_maxXFHandle = glGetUniformLocation(program, "umaxXF");
	m_minYFHandle = glGetUniformLocation(program, "uminYF");
	m_maxYFHandle = glGetUniformLocation(program, "umaxYF");

	// 前一帧数据置零
	encoderXP = 0;
	encoderYP = 0;
	encoderThetaP = 0;

	// 边缘融合
	factor = 0.95;
	colOffset *= factor;
	rowOffset *= factor;

	// 滞后滤波[在转弯时不适用]
	encoderThetaShakeOff = 0;
	NShakeOffThreshold = 3;
	NShakeOff = 0;

	resetP = false;

	// 用于保存数据[TODO:在实际应用中需要删除]
	ofstream write;
}

void TransBot::calTransformMatrix(const Encoder& encoderData, cv::Mat& TAB, const bool& reset){

    // 取数据
	int encoderX = encoderData.x;
	int encoderY = encoderData.y;
	int encoderTheta = encoderData.theta;

    // ** IMPORTANT **: OPENGL坐标与车辆坐标翻转;历史原因,在这里交换X,Y值并加对Y置加-号
    int tmp;
    tmp = encoderX;
    encoderX = -encoderY;
    encoderY = tmp;

    if(reset != resetP){
        encoderXP = 0;
        encoderYP = 0;
        encoderThetaP = 0;
    }

    float encoderThetaM = (float) encoderTheta / 10000;

    cv::Mat_<float> TOB = cv::Mat_<float>::eye(3,3);
    TOB(0,0) = cos(encoderThetaM);
    TOB(0,1) = -sin(encoderThetaM);
    TOB(1,0) = sin(encoderThetaM);
    TOB(1,1) = cos(encoderThetaM);
    TOB(0,2) = encoderX/(wholeCols * lut->header.mm_per_pixel_w);
    TOB(1,2) = encoderY/(wholeRows * lut->header.mm_per_pixel_h);

    cv::Mat_<float> TOA = cv::Mat_<float>::eye(3,3);
    float encoderThetaPM = (float)encoderThetaP / 10000;
    TOA(0,0) = cos(encoderThetaPM);
    TOA(0,1) = -sin(encoderThetaPM);
    TOA(1,0) = sin(encoderThetaPM);
    TOA(1,1) = cos(encoderThetaPM);
    TOA(0,2) = encoderXP/(wholeCols * lut->header.mm_per_pixel_w);
    TOA(1,2) = encoderYP/(wholeRows * lut->header.mm_per_pixel_h);
    cv::Mat_<float> TAO = cv::Mat_<float>::eye(3,3);
    cv::invert(TOA, TAO);

    TAB = TAO * TOB;

#ifdef __DEBUG__
    std::cout << "### DEBUG: " << TAB << std::endl;
#endif

    /**
	   encoderXP = encoderX;
	   encoderYP = encoderY;
	   encoderThetaP = encoderTheta;
    */

    resetP = reset;

#if 0

    cv::Mat_<float> TOBd = cv::Mat_<float>::eye(3,3);
    TOBd(0,0) = cos(encoderThetaM);
    TOBd(0,1) = -sin(encoderThetaM);
    TOBd(1,0) = sin(encoderThetaM);
    TOBd(1,1) = cos(encoderThetaM);
    TOBd(0,2) = encoderData.x;
    TOBd(1,2) = encoderData.y;
    cv::Mat_<float> TOAd = cv::Mat_<float>::eye(3,3);
    TOAd(0,0) = cos(encoderThetaPM);
    TOAd(0,1) = -sin(encoderThetaPM);
    TOAd(1,0) = sin(encoderThetaPM);
    TOAd(1,1) = cos(encoderThetaPM);
    TOAd(0,2) = encoderYP;
    TOAd(1,2) = -encoderXP;
    cv::Mat_<float> TAOd = cv::Mat_<float>::eye(3,3);
    cv::invert(TOAd, TAOd);
    cv::Mat_<float> TABd = cv::Mat_<float>::eye(3,3);
    TABd= TAOd * TOBd;
    cv::invert(TABd, TABd);

    write.open("/home/intesight/gpu_project/data/data0619/runxytheta.txt",ios::app);
    write << TABd(0,0) << " "  << TABd(0,1) << " "  << TABd(0,2) << " "  << TABd(1,0) << " "  << TABd(1,1) << " "  << TABd(1,2) << " "  << TABd(2,0) << " "  << TABd(2,1) << " "  << TABd(2,2)<< std::endl;
    write.close();

#endif

}

cv::Mat_<float> tranAll = cv::Mat_<float>::eye(3,3);
int __debugEncoderDataX = 0;
int __debugEncoderDataY = 0;
int __debugEncoderDataTheta = 0;




/**
 * @brief 透明车底主函数
 *
 * @param encoderData
 * @param reset
 */
void TransBot::generateTransBot(Encoder &encoderData, const bool& reset){

#ifdef __DEBUG__
	__debugEncoderDataTheta += 785;
	encoderData.x = 0 ;
	encoderData.y = 0 ;
	encoderData.theta = __debugEncoderDataTheta ;
#endif

    //shakeOff(encoderData);
    //firstOrderLag(encoderData);

    cv::Mat_<float> TAB;
    calTransformMatrix(encoderData, TAB, reset);

#define ENCODER_DATA_LOGGING
#ifdef ENCODER_DATA_LOGGING
	write.open("runxytheta.txt", ios::app);
	write << encoderData.x << " " << encoderData.y << " " << encoderData.theta << std::endl;
	write.close();
#endif

    float x0 = 1 - 2 * colOffset / wholeCols;
    float y0 = 1 - 2 * rowOffset/ wholeRows;

    GLfloat g_vertex_buffer_data[] = {
		-x0, -y0, 0.0f,
		-x0,  y0, 0.0f,
		x0, -y0, 0.0f,

		-x0,  y0, 0.0f,
		x0, -y0, 0.0f,
		x0,  y0, 0.0f,
    };

    float p0x = colOffset/wholeCols;
    float p0y = rowOffset/wholeRows;
    float p1x = colOffset/wholeCols;
    float p1y = (wholeRows - rowOffset) / wholeRows;
    float p2x = (wholeCols - colOffset) / wholeCols;
    float p2y = rowOffset/wholeRows;
    float p3x = (wholeCols - colOffset) / wholeCols;
    float p3y = (wholeRows - rowOffset) / wholeRows;


    float pos0X = 0;
    float pos0Y = 0;
    transCur2Pre(p0x, p0y, TAB, pos0X, pos0Y);

    cv::Mat_<float> tranDebug = cv::Mat_<float>::eye(3,3);
    tranDebug =  TAB.clone() ;
    tranAll = tranAll * tranDebug;

    // 保存里程计数据
#if __DEBUG__
    write.open("/home/intesight/gpu_project/data/data0619/runxytheta.txt",ios::app);
    write << tranAll(0,0) << " "  << tranAll(0,1) << " "  << tranAll(0,2) * wholeCols << " "  << tranAll(1,0) << " "  << tranAll(1,1) << " "  << tranAll(1,2) * wholeRows << " "  << tranAll(2,0) << " "  << tranAll(2,1) << " "  << tranAll(2,2)<< std::endl;
    write.close();
#endif

    float pos1X = 0;
    float pos1Y = 0;
    transCur2Pre(p1x, p1y, TAB, pos1X, pos1Y);
    float pos2X = 0;
    float pos2Y = 0;
    transCur2Pre(p2x, p2y, TAB, pos2X, pos2Y);
    float pos3X = 0;
    float pos3Y = 0;
    transCur2Pre(p3x, p3y, TAB, pos3X, pos3Y);

    // 上一帧图像的透明车底４个角的坐标
    GLfloat g_uv_buffer_data[] = {
		pos0X, pos0Y,
		pos1X, pos1Y,
		pos2X, pos2Y,

		pos1X, pos1Y,
		pos2X, pos2Y,
		pos3X, pos3Y,
    };

    // 当前图像的透明车底４个角的坐标
    GLfloat g_uv_buffer_data_origin[] = {
		p0x, p0y,
		p1x, p1y,
		p2x, p2y,

		p1x, p1y,
		p2x, p2y,
		p3x, p3y,
    };

    // 融合区域
    glm::vec4 minPosition = bevTransformMatrix * glm::vec4(colOffset/(factor * wholeCols), rowOffset/(factor * wholeRows), 0, 1);
    glm::vec4 maxPosition = bevTransformMatrix * glm::vec4(1 - colOffset/(factor * wholeCols), 1 - rowOffset/(factor * wholeRows), 0, 1);

    glm::vec4 minPositionF = bevTransformMatrix * glm::vec4(colOffset/wholeCols, rowOffset/wholeRows, 0, 1);
    glm::vec4 maxPositionF = bevTransformMatrix * glm::vec4(1 - colOffset/wholeCols, 1 - rowOffset/wholeRows, 0, 1);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glUseProgram(program);

    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(g_vertex_buffer_data), g_vertex_buffer_data);

    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(g_uv_buffer_data), g_uv_buffer_data);

    glBindBuffer(GL_ARRAY_BUFFER, uvbufferOrigin);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(g_uv_buffer_data_origin), g_uv_buffer_data_origin);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, preTextureID);
    glUniform1i(TextureID1, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, currentTextureID);
    glUniform1i(TextureID2, 1);

    glUniformMatrix4fv(m_transformMatrixHandle, 1, GL_FALSE, (GLfloat*)glm::value_ptr(bevTransformMatrix));

    glUniform1f(m_minXHandle, minPosition.x);
    glUniform1f(m_minYHandle, minPosition.y);
    glUniform1f(m_maxXHandle, maxPosition.x);
    glUniform1f(m_maxYHandle, maxPosition.y);

    glUniform1f(m_minXFHandle, minPositionF.x);
    glUniform1f(m_minYFHandle, minPositionF.y);
    glUniform1f(m_maxXFHandle, maxPositionF.x);
    glUniform1f(m_maxYFHandle, maxPositionF.y);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(
		0,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
    );

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glVertexAttribPointer(
		1,
		2,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
    );

    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, uvbufferOrigin);
    glVertexAttribPointer(
		2,
		2,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
    );

    glDrawArrays(GL_TRIANGLES, 0, 2 * 3);

    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

};

void TransBot::shakeOff(Encoder &encoderData){

    if(encoderData.theta != encoderThetaShakeOff){
        NShakeOff ++;
        if(NShakeOff >= NShakeOffThreshold){
            NShakeOff = 0;
            encoderThetaShakeOff =  encoderData.theta;
        }
        else{
            encoderData.theta = encoderThetaShakeOff;
        }
    }else{
        NShakeOff = 0;
    }
}

void TransBot::firstOrderLag(Encoder &encoderData, float factor){
    encoderData.theta = encoderThetaP * factor +
		encoderData.theta * ( 1 - factor);
}


void TransBot::judgeChangeFlag(Encoder& encoderData, Encoder& encoderDataPre, bool& transBotEncoderPreChangeFlag, float threshold){

    std::cout << "Cur : " << encoderData.x << ", " << encoderData.y << ", " << encoderData.theta << ", " << std::endl;
    std::cout << "Pre : " << encoderDataPre.x << ", " << encoderDataPre.y << ", " << encoderDataPre.theta << ", " << std::endl;

    if(calculateDistance(encoderData, encoderDataPre) > threshold){
        transBotEncoderPreChangeFlag = true;
        std::cout << "2019-08-12 changed due to threshold" << std::endl;
    }
    else{
        transBotEncoderPreChangeFlag = false;
        std::cout << "2019-08-12 haven't changed due to threshold" << std::endl;
    }
}

float TransBot::calculateDistance(Encoder& encoderData1, Encoder& encoderData2){
    /**
	   float debugx = pow((encoderData1.x/1000.0 - encoderData2.x/1000.0),2);
	   float debugy = pow((encoderData1.y/1000.0 - encoderData2.y/1000.0),2);
	   float debugdistance = debugx + debugy;
	   float distance = sqrtf(debugdistance);
	   return distance;
    */
    return sqrtf(pow((encoderData1.x/1000.0 - encoderData2.x/1000.0),2) +
                 pow((encoderData1.y/1000.0 - encoderData2.y/1000.0),2));
}

void TransBot::setEncoderDataPre(Encoder& encoderDataToSet){

    encoderXP = encoderDataToSet.x;
    encoderYP = encoderDataToSet.y;
    encoderThetaP = encoderDataToSet.theta;

    int tmp;
    tmp = encoderXP;
    encoderXP = -encoderYP;
    encoderYP = tmp;
}
