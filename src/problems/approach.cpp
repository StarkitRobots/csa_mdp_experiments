#include "problems/approach.h"

// State limits
double Approach::max_pos        =  3   ;
double Approach::min_step_x     = -0.02;
double Approach::max_step_x     =  0.04;
double Approach::max_step_y     =  0.03;
double Approach::max_step_theta =  0.2 ;
// Action limits
double Approach::max_step_x_diff     = 0.02;
double Approach::max_step_y_diff     = 0.02;
double Approach::max_step_theta_diff = 0.05;
// Step noise
double Approach::step_x_noise     = 0.01;
double Approach::step_y_noise     = 0.01;
double Approach::step_theta_noise = 0.02;
// Kick
double Approach::kick_x_min     = 0.1    ;
double Approach::kick_x_max     = 0.2    ;
double Approach::kick_y_tol     = 0.05   ;
double Approach::kick_theta_tol = M_PI/30;
double Approach::kick_reward = 1;
// Viewing the ball
double Approach::viewing_angle  = 2*M_PI/3;
double Approach::no_view_reward = -0.01   ;
// Collision
double Approach::collision_x      =  0.1;
double Approach::collision_y      =  0.3;
double Approach::collision_reward = -1  ;
// Misc
double Approach::out_of_space_reward = -1;
double Approach::init_min_dist = 0.4;
double Approach::walk_gain = 3;
