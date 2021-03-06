#include "Copter.h"

/*
 * Init and run calls for AB_Waypoint flight mode
 */

// should be called at 100hz or more

bool ModeAB_Waypoint::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
    float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }
    ab_waypoint_state = WAIT_A_LOCATION;
    ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::NORMAL_RUN;

    return true;
}

void ModeAB_Waypoint::run_loiter_control()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch, G_Dt);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);

        // update altitude target and call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed_Ground_Idle:

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        // FALLTHROUGH

    case AltHold_Landed_Pre_Takeoff:

        loiter_nav->init_target();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

/*#if PRECISION_LANDING == ENABLED
        if (do_precision_loiter()) {
            precision_loiter_xy();
        }
#endif*/

        // run loiter controller
        loiter_nav->update();
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(loiter_nav->get_roll(), loiter_nav->get_pitch(), target_yaw_rate);
        // adjust climb rate using rangefinder
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}

Vector2f ModeAB_Waypoint::Convert_NavPos_to_XYPos(Vector3f NavPos)
{
    return Vector2f(NavPos.y, NavPos.x);
}

void ModeAB_Waypoint::Mem_AB_Point_Tricked()
{
    Vector2f Temp_vector;
    if(ab_waypoint_state == AB_WAYPOINT_STATE::WAIT_A_LOCATION){
        A_Point_Loc = inertial_nav.get_position();
        A_Point = Convert_NavPos_to_XYPos(A_Point_Loc); // Convert N-E to X-Y Coordinate

        ab_waypoint_state = AB_WAYPOINT_STATE::WAIT_B_LOCATION;
        gcs().send_text(MAV_SEVERITY_INFO, "Save A Point at = %f %f",A_Point.x,A_Point.y);
    }
    else if (ab_waypoint_state == AB_WAYPOINT_STATE::WAIT_B_LOCATION){
        B_Point_Loc = inertial_nav.get_position();
        B_Point = Convert_NavPos_to_XYPos(B_Point_Loc); // Convert N-E to X-Y Coordinate

        ab_waypoint_state = AB_WAYPOINT_STATE::EXPAND_TO_LR;
        gcs().send_text(MAV_SEVERITY_INFO, "Save B Point at = %f %f",B_Point.x,B_Point.y);
    }
    else if (ab_waypoint_state == AB_WAYPOINT_STATE::EXPAND_TO_LR){
        ab_waypoint_state = AB_WAYPOINT_STATE::CAL_TF;
    }
}

Vector2f ModeAB_Waypoint::Transform_Point_to_BA_Frame(Vector2f Point_2be_Transformed,Vector2f b_point, float Angle_from_North)
{
    Matrix3f Rotation_Matrix;
    Rotation_Matrix.from_euler(0,0,Angle_from_North);
    Vector3f Point_2be_Transformed_V3 = Vector3f(Point_2be_Transformed.x - b_point.x, Point_2be_Transformed.y - b_point.y, 0);

    return Rotation_Matrix.mulXY(Point_2be_Transformed_V3);
}

Vector2f ModeAB_Waypoint::Transform_BA_Fram_to_Point_NE(Vector2f Point_2be_Transformed,Vector2f b_point, float Angle_from_North)
{
    Matrix3f Rotation_Matrix;
    Rotation_Matrix.from_euler(0,0,-1*Angle_from_North);
    Vector3f Point_2be_Transformed_V3 = Vector3f(Point_2be_Transformed.x, Point_2be_Transformed.y, 0);
    Vector2f Result = Rotation_Matrix.mulXY(Point_2be_Transformed_V3);
    Result.x += b_point.x;
    Result.y += b_point.y;
    return Result;
}

void ModeAB_Waypoint::run()
{
    switch(ab_waypoint_state)
    {
        case WAIT_A_LOCATION:
        case WAIT_B_LOCATION:
        case EXPAND_TO_LR:
            run_loiter_control();
        break;
        // Function under above cases are implemented in RC_Channel.cpp ------- RC_Channel_Copter::do_aux_function
        
        // Calculate transformation matrix to Vector B to A frame
        case CAL_TF:
            wp_nav->wp_and_spline_init();

            // CAL_TF from A B point
            // Calulate heading of Vector from B to A
            V_BA = A_Point - B_Point;
            Angle_of_V_BA = -1*(V_BA.angle() - M_PI_2); // Calculate Angle from X axis
            Top_End_Pos_TF = V_BA.length();
            Bottom_End_Pos_TF = 0;

            A_Point_TF = Transform_Point_to_BA_Frame(A_Point, B_Point, Angle_of_V_BA); // Transform point A to Vector B to A frame
            B_Point_TF = Vector2f(0,0);                                                // Transform B point to Vector B to A frame 
            Curr_Vehicle_2DPosTF = Transform_Point_to_BA_Frame(Convert_NavPos_to_XYPos(inertial_nav.get_position()), B_Point, Angle_of_V_BA); // transform current vehicle loc to Vector B to A frame
            spacing_distance = g.ab_slide_space; // load spacing distance from parameter
            
            gcs().send_text(MAV_SEVERITY_INFO, "Angle to North = %f",Angle_of_V_BA);
            gcs().send_text(MAV_SEVERITY_INFO, "A_Point_TF = %f, %f",A_Point_TF.x, A_Point_TF.y);
            gcs().send_text(MAV_SEVERITY_INFO, "Curr_Vehicle_2DPos = %f, %f",Curr_Vehicle_2DPosTF.x, Curr_Vehicle_2DPosTF.y);

            // determine sliding direction (left or right)
            if(Curr_Vehicle_2DPosTF.x >= 0){
                lr_flag = LR_FLAG::RIGHT;
                gcs().send_text(MAV_SEVERITY_INFO, "SLIDE TO THE RIGHT");
            }else{
                lr_flag = LR_FLAG::LEFT;
                spacing_distance *= -1;
                gcs().send_text(MAV_SEVERITY_INFO, "SLIDE TO THE LEFT");
            }
            Prior_Dest_TF = B_Point_TF;
            prior_ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::TO_BOTTOM_END;


        // Calculate next destination in ZigZag pattern    
        case CAL_NEXT_DEST:
            // CAL Next Destination on Vector B to A frame
            switch(ab_waypoint_auto_state){
                case AB_WAYPOINT_AUTO_STATE::SLIDE:
                    if(prior_ab_waypoint_auto_state==AB_WAYPOINT_AUTO_STATE::TO_TOP_END)
                    {
                        Next_Dest_TF = Vector2f(Prior_Dest_TF.x + spacing_distance, Top_End_Pos_TF);
                        gcs().send_text(MAV_SEVERITY_INFO, "Cal Slide at Top End");
                    }else if(prior_ab_waypoint_auto_state==AB_WAYPOINT_AUTO_STATE::TO_BOTTOM_END){

                        Next_Dest_TF = Vector2f(Prior_Dest_TF.x + spacing_distance, Bottom_End_Pos_TF); 
                        gcs().send_text(MAV_SEVERITY_INFO, "Cal Slide at Bottom End");           
                    }
            
                    if(Next_Dest_TF.y > Bottom_End_Pos_TF-0.05f && Next_Dest_TF.y < Bottom_End_Pos_TF+0.05f){
                        ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::TO_TOP_END;
                        gcs().send_text(MAV_SEVERITY_INFO, "Update state to TO_TOP_END");
                    }else{
                        ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::TO_BOTTOM_END;
                        gcs().send_text(MAV_SEVERITY_INFO, "Update state to TO_BOTTEM_END");
                    }
                    
                    prior_ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::SLIDE;
                    break;
                case AB_WAYPOINT_AUTO_STATE::TO_TOP_END:
                    //Next_Dest_TF = Vector2f(Prior_Dest_TF.x, Prior_Dest_TF.y + Area_width);
                    Next_Dest_TF = Vector2f(Prior_Dest_TF.x, Top_End_Pos_TF);
                    ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::SLIDE;
                    prior_ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::TO_TOP_END;
                    gcs().send_text(MAV_SEVERITY_INFO, "Cal Top End Dest");
                    break;
                case AB_WAYPOINT_AUTO_STATE::TO_BOTTOM_END:
                    Next_Dest_TF = Vector2f(Prior_Dest_TF.x, Bottom_End_Pos_TF);
                    ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::SLIDE;
                    prior_ab_waypoint_auto_state = AB_WAYPOINT_AUTO_STATE::TO_BOTTOM_END;
                    gcs().send_text(MAV_SEVERITY_INFO, "Cal Bottom End Dest");
                    break;
            };
            
            Prior_Dest_TF = Next_Dest_TF;

            // Transform calculated next destination in Vector B to A frame to N-E frame before set for drone
            Next_Dest = Transform_BA_Fram_to_Point_NE(Next_Dest_TF, B_Point, Angle_of_V_BA);
            Current_Alt = inertial_nav.get_altitude();
            Next_Dest_Loc = Vector3f(Next_Dest.y, Next_Dest.x, Current_Alt);
            wp_nav->set_wp_destination(Next_Dest_Loc,false); // set next destination for drone

            // set motors to full range
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Move the drone to calculated next destination
        case MOVE_TO_DEST:
            float target_roll, target_pitch;

            // get climb rate input from pilot
            float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in()); 
            target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up); 

            // get roll-pitch rate input from pilot
            get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max());


            // Manage drone behavior when pilot intervension
            switch(ab_waypoint_move_to_dest_state)
            {
                case AB_WAYPOINT_MOVE_TO_DEST_STATE::CHANGE_ALT: // Active once pilot move altitude stick up or down...De-active once pilot move altitude stick to nuetral
                    run_loiter_control();
                    if(abs(target_climb_rate)<=g.ab_min_input_climb_rate) ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::UPDATE_DEST;
                break;
                case AB_WAYPOINT_MOVE_TO_DEST_STATE::CHANGE_EDGE: // Active once pilot move Pitch stick up or down...De-active once pilot move Pitch stick to nuetral
                    run_loiter_control();
                    if(abs(target_pitch)<=g.ab_min_input_roll_pitch_rate && inertial_nav.get_speed_xy()<=g.ab_min_xy_speed){
                        Vector2f Curr_XYPos = Convert_NavPos_to_XYPos(inertial_nav.get_position());
                        Vector2f Curr_XYPos_TF = Transform_Point_to_BA_Frame(Curr_XYPos,B_Point,Angle_of_V_BA); // Transform current pos to Vector B to A

                        // Update Top_End_Pos_TF or Bottom_End_Pos_TF
                        if(ab_waypoint_auto_state == AB_WAYPOINT_AUTO_STATE::SLIDE)
                        {
                            if(prior_ab_waypoint_auto_state==AB_WAYPOINT_AUTO_STATE::TO_TOP_END) 
                            {
                                Top_End_Pos_TF = Curr_XYPos_TF.y;                          
                                gcs().send_text(MAV_SEVERITY_INFO, "Cal Top_End_Pos_TF");
                            }else if(prior_ab_waypoint_auto_state==AB_WAYPOINT_AUTO_STATE::TO_BOTTOM_END){
                                Bottom_End_Pos_TF = Curr_XYPos_TF.y;
                                gcs().send_text(MAV_SEVERITY_INFO, "Cal Bottom_End_Pos_TF");
                            }
                        }

                        ab_waypoint_state = CAL_NEXT_DEST; // Re-calculate next destination after update 
                        wp_nav->wp_and_spline_init();
                        ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::NORMAL_RUN;
                    }
                break;

                case AB_WAYPOINT_MOVE_TO_DEST_STATE::CHANGE_EXTENSION: // Active once pilot move Roll stick up or down...De-active once pilot move Roll stick to nuetral
                    run_loiter_control();
                    if(abs(target_roll)<=g.ab_min_input_roll_pitch_rate && inertial_nav.get_speed_xy()<=g.ab_min_xy_speed){
                        Vector2f Curr_XYPos = Convert_NavPos_to_XYPos(inertial_nav.get_position());
                        Vector2f Curr_XYPos_TF = Transform_Point_to_BA_Frame(Curr_XYPos,B_Point,Angle_of_V_BA);
                        if(ab_waypoint_auto_state == AB_WAYPOINT_AUTO_STATE::SLIDE)
                        {
                            Prior_Dest_TF.x = Curr_XYPos_TF.x;
                            ab_waypoint_auto_state = prior_ab_waypoint_auto_state;
                        }
                        ab_waypoint_state = CAL_NEXT_DEST;
                        wp_nav->wp_and_spline_init();
                        ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::NORMAL_RUN;
                    }
                break;

                case AB_WAYPOINT_MOVE_TO_DEST_STATE::UPDATE_DEST: // Update destination point after altitude changed
                    Current_Alt = inertial_nav.get_altitude();
                    Next_Dest_Loc = Vector3f(Next_Dest.y, Next_Dest.x, Current_Alt);
                    wp_nav->wp_and_spline_init();
                    wp_nav->set_wp_destination(Next_Dest_Loc,false);
                    ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::NORMAL_RUN;
                break;

                case AB_WAYPOINT_MOVE_TO_DEST_STATE::NORMAL_RUN:
                    // Auto to destination
                    wp_nav->update_wpnav();
                    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(),ToDeg(Angle_of_V_BA)*100 ,true);
                    
                    // check destination reached
                    if(wp_nav->reached_wp_destination_xy())
                    {
                        gcs().send_text(MAV_SEVERITY_INFO, "Destination reached CAL_NEXT_DEST");
                        ab_waypoint_state = CAL_NEXT_DEST;
                    }else{
                        ab_waypoint_state = MOVE_TO_DEST;
                    }   

                    // State transition
                    if(abs(target_climb_rate)>g.ab_min_input_climb_rate && ab_waypoint_auto_state == AB_WAYPOINT_AUTO_STATE::SLIDE)
                        ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::CHANGE_ALT;
                    else if(abs(target_pitch)>g.ab_min_input_roll_pitch_rate && ab_waypoint_auto_state == AB_WAYPOINT_AUTO_STATE::SLIDE)
                        ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::CHANGE_EDGE;
                    else if(abs(target_roll)>g.ab_min_input_roll_pitch_rate && ab_waypoint_auto_state == AB_WAYPOINT_AUTO_STATE::SLIDE)
                        ab_waypoint_move_to_dest_state = AB_WAYPOINT_MOVE_TO_DEST_STATE::CHANGE_EXTENSION;

                break;
                default:
                break;
            }
            
            pos_control->update_z_controller();
            
        break;

        //default:

        //break;
    };
}