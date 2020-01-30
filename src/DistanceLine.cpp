#include "DistanceLine.hpp"
#include "camera_model.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>

DistanceLineView::DistanceLineView(const char* configPath, const char* vehicleConfigPath, std::shared_ptr<NVGcontext> vgCtx, std::shared_ptr<CalibLUT> lut)
    : vgCtx(vgCtx), lut(lut)
{
    loadConfig(configPath);
    loadVehicleConfig(vehicleConfigPath);
}

void DistanceLineView::loadVehicleConfig(const char* configPath)
{
    const auto config = toml::parse(configPath);
    vehicleWidth = toml::find<float>(config, "vehicle_width");
    vehicleLength = toml::find<float>(config, "vehicle_length");
}

void DistanceLineView::DistanceLineParam::from_toml(const toml::value& v)
{
    distanceToCar = toml::find<float>(v, "distance_to_car");
    distanceToCarFront = toml::find<float>(v, "distance_to_car_front");
    length = toml::find<float>(v, "length");
    width = toml::find<float>(v, "width");
    auto rgba = toml::find<std::array<int, 4>>(v, "color");
    color = nvgRGBA(rgba[0], rgba[1], rgba[2], rgba[3]);
}

void DistanceLineView::loadConfig(const char* configPath)
{
    const auto config = toml::parse(configPath);
    static constexpr char* table[] = { "front", "rear", "left", "right" };
    for (int i = 0; i < 4; ++i)
    {
        params[i] = toml::find<DistanceLineParam>(config, table[i]);
    }
}

void DistanceLineView::draw(int camId, int x, int y, int width, int height, const glm::mat4& transform)
{
    glViewport(x, y, width, height);
    auto vg = vgCtx.get();
    if (camId < 2)
    {
        float mmPerPixelH = lut->header.mm_per_pixel_h, mmPerPixelW = lut->header.mm_per_pixel_w;
        nvgBeginFrame(vg, lut->header.bev_img_width, lut->header.bev_img_height, 1);
        nvgSave(vg);
        nvgTransform4(vg, glm::value_ptr(transform));

    
        const auto& carRect = lut->header.car_Icon_Rect;
        float length = params[camId].length / mmPerPixelW,
            width = params[camId].width / mmPerPixelH,
            distance = params[camId].distanceToCar / mmPerPixelH;

        float startX, startY, endX, endY;
        startX = carRect.x + carRect.width / 2 - length / 2;
        endX = carRect.x + carRect.width / 2 + length / 2;
        startY = (camId == const_value::FRONT_CAM ? carRect.y - distance : carRect.br().y + distance);
        endY = startY;

        nvgBeginPath(vg);
        nvgMoveTo(vg, startX, startY);
        nvgLineTo(vg, endX, endY);
        nvgStrokeWidth(vg, width);
        nvgStrokeColor(vg, params[camId].color);
        nvgStroke(vg);

        nvgRestore(vg);
        nvgEndFrame(vg);
    }
    else
    {
        nvgBeginFrame(vg, lut->header.src_img_width, lut->header.src_img_height, 1);
        nvgSave(vg);
        nvgTransform4(vg, glm::value_ptr(transform));
        
        auto callback = [](void* ctx, NVGvertex* vert)
        {
            int camId = *(int*)ctx;
            double worldRay[3] = { vert->x, vert->y, 0 };
            double imagePoint[2] = { 0 };
            World_Ray_To_Image_Point(imagePoint, worldRay, Camera_Model[camId]);
            //printf("camId = %d, (%f, %f) -> (%f, %f)\n", camId, vert->x, vert->y, imagePoint[1], imagePoint[0]);
            vert->x = imagePoint[1];
            vert->y = imagePoint[0];
        };
        nvgVertexTransformCallback(vg, static_cast<NVGVertexTransformCallback>(callback), static_cast<void*>(&camId));

        float startX = (camId == const_value::LEFT_CAM ? -1 : 1) * (vehicleWidth / 2 + params[camId].distanceToCar);
        float startY = vehicleLength / 2 - params[camId].distanceToCarFront;
        float endX = startX;
        float endY = startY + params[camId].length;
        //printf("distance line: (%f, %f) -> (%f, %f)\n", startX, startY, endX, endY);
        nvgBeginPath(vg);
        nvgMoveTo(vg, startX, startY);

        int numSegments = 10;
        float step = params[camId].length / numSegments;
        for (int i = 1; i <= numSegments; ++i)
        {
            nvgLineTo(vg, endX, startY + i * step);
        }
        
        nvgLineTo(vg, endX + (camId == const_value::LEFT_CAM ? 200.0 : -200.0), endY);

        nvgStrokeWidth(vg, params[camId].width);
        nvgStrokeColor(vg, params[camId].color);
        nvgStroke(vg);

        nvgRestore(vg);
        nvgEndFrame(vg);
    }
}