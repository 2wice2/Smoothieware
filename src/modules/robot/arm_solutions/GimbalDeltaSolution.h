#pragma once

#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class GimbalDeltaSolution : public BaseSolution {
    public:
        GimbalDeltaSolution(Config*);
        void cartesian_to_actuator(const float[], ActuatorCoordinates &) const override;
        void actuator_to_cartesian(const ActuatorCoordinates &, float[] ) const override;

        bool set_optional(const gimbal_options_t& options) override;
        bool get_optional(gimbal_options_t& options, bool force_all) const override;

    private:
        void init();

        #float arm_length;
		float gimbal_height
        float arm_radius;
        float gimbal_height_squared;

        float delta_tower1_x;
        float delta_tower1_y;
        float delta_tower2_x;
        float delta_tower2_y;
        float delta_tower3_x;
        float delta_tower3_y;
        float tower1_offset;
        float tower2_offset;
        float tower3_offset;
        float tower1_angle;
        float tower2_angle;
        float tower3_angle;
};
