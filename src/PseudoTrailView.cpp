#include <stdexcept>
#include "PseudoTrailView.hpp"
#include "ogl/ShaderUtil.h"

PseudoTrailView::PseudoTrailView(float minAngle, float maxAngle, float step,
        const TrailIndex* filledIndices, const float* filledVertices,
        const TrailIndex* leftWheelIndices, const float* leftWheelVertices,
        const TrailIndex* rightWheelIndices, const float* rightWheelVertices)
    : minAngle(minAngle), maxAngle(maxAngle), step(step),
     filledIndices(filledIndices), leftWheelIndices(leftWheelIndices), rightWheelIndices(rightWheelIndices)
{
    int n = (maxAngle - minAngle) / step + 1;

    filledVBO = createVBOFromVertices(filledVertices, filledIndices, n);
    leftWheelVBO = createVBOFromVertices(leftWheelVertices, leftWheelIndices, n);
    rightWheelVBO = createVBOFromVertices(rightWheelVertices, rightWheelIndices, n);
}

GLuint PseudoTrailView::createVBOFromVertices(const float* vertices, const TrailIndex* indices, int n)
{
    GLuint vbo = 0;
    size_t vertexCount = 0;
    for (int i = 0; i < n; ++i)
    {
        vertexCount += indices[i].num_triangles * 3;
    }

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertexCount * 2, vertices, GL_STATIC_DRAW);

    return vbo;
}

void PseudoTrailView::drawTriangles(GLuint vbo, int offset, int n, const NVGcolor& color, const glm::mat4& transform)
{
    static bool initialized = false;
    if (!initialized)
    {
        if (loadShaders("/data/opengl_new/shader/shape.vert", "/data/opengl_new/shader/shape.frag", program) != 0)
        {
            throw std::runtime_error("could not load shape.vert/shape.frag");
        }
        positionAttr = glGetAttribLocation(program, "aPosition");
        transformMatrixUnif = glGetUniformLocation(program, "uTransformMatrix");
        colorUnif = glGetUniformLocation(program, "uColor");
        initialized = true;
    }

    glUseProgram(program);

    glUniformMatrix4fv(transformMatrixUnif, 1, GL_FALSE, glm::value_ptr(transform));
    NVGcolor permultipledColor = nvgRGBAf(color.r * color.a, color.g * color.a, color.b * color.a, color.a);
    glUniform4fv(colorUnif, 1, permultipledColor.rgba);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(positionAttr, 2, GL_FLOAT, GL_FALSE, 0, NULL);

    glEnableVertexAttribArray(positionAttr);

    glDrawArrays(GL_TRIANGLES, offset * 3, n * 3);

    glDisableVertexAttribArray(positionAttr);
}
