#pragma once

#include <memory>

#include <GLES2/gl2.h>
#include <glm/glm.hpp>
#include "nanovg.h"
#include "toml11/toml.hpp"
#include "LUT.h"

#include "const_value.h"

class DistanceLineView
{
public:
	DistanceLineView() = default;
	DistanceLineView(const char* configPath, const char* vehicleConfigPath, std::shared_ptr<NVGcontext> vgCtx, std::shared_ptr<CalibLUT> lut);
	
	void draw(int camId, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));

private:
	void loadConfig(const char* configPath);
	void loadVehicleConfig(const char* configPath);
	
	std::shared_ptr<NVGcontext> vgCtx;
	std::shared_ptr<CalibLUT> lut;

	struct DistanceLineParam
	{
		float distanceToCar;
		float distanceToCarFront;
		float width;
		float length;
		NVGcolor color;

		void from_toml(const toml::value& v);
	};

	DistanceLineParam params[const_value::CAMERANUM];

	float vehicleLength, vehicleWidth;
};
