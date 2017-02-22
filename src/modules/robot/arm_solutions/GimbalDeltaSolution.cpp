#include "GimbalDeltaSolution.h"
#include "ActuatorCoordinates.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"

#include <fastmath.h>
#include "Vector3.h"


#define gimbal_delta_radius_checksum  CHECKSUM("gimbal_delta_radius")
#define gimbal_height_checksum        CHECKSUM("gimbal_height")
#define arm_radius_checksum           CHECKSUM("arm_radius")

#define tower1_offset_checksum        CHECKSUM("delta_tower1_offset")
#define tower2_offset_checksum        CHECKSUM("delta_tower2_offset")
#define tower3_offset_checksum        CHECKSUM("delta_tower3_offset")
#define tower1_angle_checksum         CHECKSUM("delta_tower1_angle")
#define tower2_angle_checksum         CHECKSUM("delta_tower2_angle")
#define tower3_angle_checksum         CHECKSUM("delta_tower3_angle")

#define SQ(x) powf(x, 2)
#define ROUND(x, y)       (roundf(x * (float)(1e ## y)) / (float)(1e ## y))
#define PIOVER180         0.01745329251994329576923690768489F
#define COS60             0.5
#define SIN60             (sqrtf(3)/2)
     
    

GimbalDeltaSolution::GimbalDeltaSolution(Config* config)
{
    //This is the height of the gimbal center from the bottom rod pivot
	gimbal_height = config->value(gimbal_height_checksum)->by_default(354.85f)->as_number();
	//This is the horizontal distance between top gimbal pivot and bottom pivot when centered
	arm_radius = config->value(arm_radius_checksum)->by_default(209.74f)->as_number();
	
    tower1_angle = config->value(tower1_angle_checksum)->by_default(0.0f)->as_number();
    tower2_angle = config->value(tower2_angle_checksum)->by_default(0.0f)->as_number();
    tower3_angle = config->value(tower3_angle_checksum)->by_default(0.0f)->as_number();
    tower1_offset = config->value(tower1_offset_checksum)->by_default(0.0f)->as_number();
    tower2_offset = config->value(tower2_offset_checksum)->by_default(0.0f)->as_number();
    tower3_offset = config->value(tower3_offset_checksum)->by_default(0.0f)->as_number();

    init();
}

void GimbalDeltaSolution::init()
{
    //gimbal_height_squared = SQ(gimbal_height);
	arm_radius_squared = SQ(arm_radius);
	
    // Effective X/Y positions of the three vertical towers.
    float delta_radius = arm_radius;

    delta_tower1_x = (delta_radius + tower1_offset) * cosf((210.0F + tower1_angle) * PIOVER180); // front left tower
    delta_tower1_y = (delta_radius + tower1_offset) * sinf((210.0F + tower1_angle) * PIOVER180);
    delta_tower2_x = (delta_radius + tower2_offset) * cosf((330.0F + tower2_angle) * PIOVER180); // front right tower
    delta_tower2_y = (delta_radius + tower2_offset) * sinf((330.0F + tower2_angle) * PIOVER180);
    delta_tower3_x = (delta_radius + tower3_offset) * cosf((90.0F  + tower3_angle) * PIOVER180); // back middle tower
    delta_tower3_y = (delta_radius + tower3_offset) * sinf((90.0F  + tower3_angle) * PIOVER180);
}

void GimbalDeltaSolution::cartesian_to_actuator(const float cartesian_mm[], ActuatorCoordinates &actuator_mm ) const
{
    actuator_mm[ALPHA_STEPPER] = sqrtf(this->SQ(delta_tower1_x - cartesian_mm[X_AXIS]) 
                                            - SQ(delta_tower1_y - cartesian_mm[Y_AXIS]) 
                                            + SQ(gimbal_height - cartesian_mm[Z_AXIS]));
									  
	actuator_mm[BETA_STEPPER ] = sqrtf(this->SQ(delta_tower2_x - cartesian_mm[X_AXIS]) 
                                            - SQ(delta_tower2_y - cartesian_mm[Y_AXIS]) 
                                            + SQ(gimbal_height - cartesian_mm[Z_AXIS]));
											
    actuator_mm[GAMMA_STEPPER] = sqrtf(this->SQ(delta_tower3_x - cartesian_mm[X_AXIS]) 
                                            - SQ(delta_tower3_y - cartesian_mm[Y_AXIS]) 
                                            + SQ(gimbal_height - cartesian_mm[Z_AXIS]));
}

void GimbalDeltaSolution::actuator_to_cartesian(const ActuatorCoordinates &actuator_mm, float cartesian_mm[] ) const
{
    //tranlate all towers down so that Tower3(Gamma) lies on (0,0)
	float ttower1_x = delta_tower1_x;
	float ttower1_y = delta_tower1_y - arm_radius;
	float ttower2_x = delta_tower2_x;
	float ttower2_y = delta_tower2_y - arm_radius;
	float ttower3_x = delta_tower3_x;
	float ttower3_y = delta_tower3_y - arm_radius;

	//Rotate all towers around (0,0) so that towers 3 & 2 lie on the x-axis
	float rtower1_x = (ttower1_x * COS60) - (ttower1_y * SIN60);
	float rtower1_y = (ttower1_x * SIN60) + (ttower1_y * COS60);
	float rtower2_x = (ttower2_x * COS60) - (ttower2_y * SIN60);
	float rtower2_y = (ttower2_x * SIN60) + (ttower2_y * COS60);
	float rtower3_x = ttower3_x;      //already on (0,0)
	float rtower3_y = ttower3_y;      //already on (0,0)
	
	float tower1_radius = actuator_mm[0];
    float tower2_radius = actuator_mm[1];
    float tower3_radius = actuator_mm[2];
	
	//find the x coord of intersect between sphere 1 & 2
	float intersect_x = (SQ(tower3_radius) - SQ(tower2_radius) + SQ(rtower2_x)) / ( 2 * rtower2_x);
    //find the y coord of intersect between sphere 1 & 2
	float intersect_y = (SQ(tower3_radius) - SQ(tower1_radius) + SQ(rtower1_x) + SQ(rtower1_y)) / ( 2 * rtower1_y) 
	                                    - (rtower1_x / rtower1_y) * intersect_x;
	//find the z coord of intersect
	float intersect_z = sqrtf(SQ(tower3_radius) - SQ(intersect_x) - SQ(intersect_y) );
	
	//translate and rotate back to cartesian
	float nr_intersect_x = (intersect_x * COS60 + intersect_y * SIN60);
	float nr_intersect_y = (-intersect_x * SIN60 + ttower1_y * COS60) + arm_radius;
	
	float nr_intersect_z = intersect_z + gimbal_height;
	
	cartesian_mm[0] = ROUND(nr_intersect_x, 4);
    cartesian_mm[1] = ROUND(nr_intersect_y, 4);
    cartesian_mm[2] = ROUND(nr_intersect_z, 4);
}

bool GimbalDeltaSolution::set_optional(const gimbal_options_t& options)
{

    for(auto &i : options) {
        switch(i.first) {
            //case 'L': arm_length = i.second; break;
			case 'L': gimbal_height = i.second; break;
            case 'R': arm_radius = i.second; break;
            case 'A': tower1_offset = i.second; break;
            case 'B': tower2_offset = i.second; break;
            case 'C': tower3_offset = i.second; break;
            case 'D': tower1_angle = i.second; break;
            case 'E': tower2_angle = i.second; break;
            case 'F': tower3_angle = i.second; break; // WARNING this will be deprecated
            case 'H': tower3_angle = i.second; break;
        }
    }
    init();
    return true;
}

bool GimbalDeltaSolution::get_optional(gimbal_options_t& options, bool force_all) const
{
    options['L'] = this->gimbal_height;
    options['R'] = this->arm_radius;

    // don't report these if none of them are set
    if(force_all || (this->tower1_offset != 0.0F || this->tower2_offset != 0.0F || this->tower3_offset != 0.0F ||
                     this->tower1_angle != 0.0F  || this->tower2_angle != 0.0F  || this->tower3_angle != 0.0F) ) {

        options['A'] = this->tower1_offset;
        options['B'] = this->tower2_offset;
        options['C'] = this->tower3_offset;
        options['D'] = this->tower1_angle;
        options['E'] = this->tower2_angle;
        options['H'] = this->tower3_angle;
    }

    return true;
};
