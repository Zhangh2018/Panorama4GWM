#pragma once

#include "trail_lut.h"
#include <GLES2/gl2.h>
#include <nanovg.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

class PseudoTrailView
{
public:
    PseudoTrailView() = default;
    PseudoTrailView(float minAngle, float maxAngle, float step,
        const TrailIndex* filledIndices, const float* filledVertices,
        const TrailIndex* leftWheelIndices, const float* leftWheelVertices,
        const TrailIndex* rightWheelIndices, const float* rightWheelVertices);

    void drawWheelTrail(float degree, int x, int y, int width, int height, const NVGcolor& color, const glm::mat4& transform = glm::mat4(1.0))
    {
        int idx = angle2index(degree);
        glViewport(x, y, width, height);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        drawTriangles(leftWheelVBO, leftWheelIndices[idx].offset, leftWheelIndices[idx].num_triangles, color, transform);
        drawTriangles(rightWheelVBO, rightWheelIndices[idx].offset, rightWheelIndices[idx].num_triangles, color, transform);
        glDisable(GL_BLEND);
    }
    void drawFilledTrail(float degree, int x, int y, int width, int height, const NVGcolor& color, const glm::mat4& transform = glm::mat4(1.0))
    {
        int idx = angle2index(degree);
        glViewport(x, y, width, height);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        drawTriangles(filledVBO, filledIndices[idx].offset, filledIndices[idx].num_triangles, color, transform);
        glDisable(GL_BLEND);
    }

private:
    void drawTriangles(GLuint vbo, int offset, int n, const NVGcolor& color, const glm::mat4& transform = glm::mat4(1.0));
    int angle2index(float degree) { return (degree - minAngle) / step; }
    GLuint createVBOFromVertices(const float* vertices, const TrailIndex* filledIndices, int n);

    GLuint program;
    GLuint filledVBO, leftWheelVBO, rightWheelVBO;
    GLint positionAttr, transformMatrixUnif, colorUnif;
    float minAngle, maxAngle, step;
    const TrailIndex *filledIndices, *leftWheelIndices, *rightWheelIndices;
};