

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/

#include "start_igk.h"
#include "..\System\carManagement.h"
#include "..\..\UART_VCOM.h"
#include "..\..\SD_CARD.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
extern t_car s_car;                             // define of car data

char * p_string_car_status = "car_st =";
char * p_string_brake = "brake =";
char * p_string_gearSpeedStatus = "gearSt =";
char * p_string_speed = "speed =";
char * p_string_acc = "accele =";
char * p_string_retour = "\n";
char * p_string_pipe = " | ";

long int cpt_display_info = 0;                  // data are display  every 1000000 loops on USB UART
long int cpt_speed_tempo = 0;                   // speed forward is update every 100000 loops
long int cpt_speed_tempo_rev = 0;               // speed reverse is update every 100000 loops
long int cpt_speed_regu = 0;                    // speed regulation is updated every 100000 loops
long int cpt_speed_limitation = 0;              // speed limitation is updated every 100000 loops
long int cpt_direction_control = 0;             // direction control is updated every 100000 loops

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/



/* speed gear control */
void gearRatio(void){

    /* front moving speed gear management */
    if((s_car.ss_system.car_status == CAR_STARTED) && (s_car.ss_controlMotor.acceleration == 1)&&
       (s_car.ss_controlMotor.gearSpeedStatus == 1) && (s_car.ss_controlMotor.brake == 0))
    {
        s_car.ss_system.car_status = FRONT_MOVING_CAR;

    }

    /* return to CAR_STARTED if the car state is FRONT_MOVING_CAR */
    if((s_car.ss_system.car_status == FRONT_MOVING_CAR) && (s_car.ss_controlMotor.brake == 1) &&
       (s_car.ss_controlMotor.speed == 0) && (s_car.ss_controlMotor.gearSpeedStatus == 0) &&
       (s_car.ss_controlMotor.engineTorque < 5) && (s_car.ss_controlMotor.acceleration == 0))
    {
        s_car.ss_system.car_status = CAR_STARTED;

    }


    /* reverse speed gear management */
    if((s_car.ss_system.car_status == CAR_STARTED) && (s_car.ss_controlMotor.acceleration == 1)&&
       (s_car.ss_controlMotor.gearSpeedStatus == 9) && (s_car.ss_controlMotor.brake == 0))
    {
        s_car.ss_system.car_status = CAR_IN_REVERSE;

    }

    /* return to CAR_STARTED if the car state is CAR_IN_REVERSE */
    if((s_car.ss_system.car_status == CAR_IN_REVERSE) && (s_car.ss_controlMotor.brake == 1) &&
       (s_car.ss_controlMotor.speed == 0) && (s_car.ss_controlMotor.gearSpeedStatus == 0) &&
       (s_car.ss_controlMotor.engineTorque < 5) && (s_car.ss_controlMotor.acceleration == 0))
    {
       s_car.ss_system.car_status = CAR_STARTED;
    }
}


/* speed front moving control */
void forwardMotion(void){

    if (cpt_speed_tempo == 100000)
    {

        if((s_car.ss_system.car_status == FRONT_MOVING_CAR) && (s_car.ss_controlMotor.acceleration == 1) &&
                        (s_car.ss_controlMotor.speed < s_car.ss_controlMotor.speedLimitForward))
                {
                   s_car.ss_controlMotor.speed++;
                }

        if((s_car.ss_system.car_status == FRONT_MOVING_CAR) && (s_car.ss_controlMotor.brake == 1) &&
                    (s_car.ss_controlMotor.speed > 0))
                {
                s_car.ss_controlMotor.speed--;
                }

        cpt_speed_tempo = 0;
    }
    cpt_speed_tempo++;
}


/* speed reverse control */
void reverseGear(void){
    if (cpt_speed_tempo_rev == 100000)
    {
        if((s_car.ss_system.car_status == CAR_IN_REVERSE) && (s_car.ss_controlMotor.acceleration == 1) &&
               (s_car.ss_controlMotor.speed < s_car.ss_controlMotor.speedLimitBackward))
            {
               s_car.ss_controlMotor.speed++;
            }

        if((s_car.ss_system.car_status == CAR_IN_REVERSE) && (s_car.ss_controlMotor.brake == 1) &&
               (s_car.ss_controlMotor.speed > 0))
            {
               s_car.ss_controlMotor.speed--;
            }

        cpt_speed_tempo_rev = 0;
    }
    cpt_speed_tempo_rev++;

}

/* speed regulation */

void speedRegulation(void){
    if (cpt_speed_regu == 100000){
        if ( (s_car.ss_system.car_status == FRONT_MOVING_CAR ) && (s_car.ss_controlMotor.speed > 0)
                && (s_car.ss_cockpit.cruiseControl > 0 )){
                s_car.ss_controlMotor.speed = s_car.ss_cockpit.cruiseControl;
                s_car.ss_system.car_status = REGULATED_SPEED_VEHICLE;
            }
        cpt_speed_regu = 0;
    }
    cpt_speed_regu++;
}


/* speed limitation */
void speedLimit(void){
    if (cpt_speed_limitation == 100000){
        if ( (s_car.ss_system.car_status == FRONT_MOVING_CAR ) && (s_car.ss_controlMotor.speed > 0)
                && (s_car.ss_cockpit.speedLimiter > 0)){
                    s_car.ss_controlMotor.speedLimitForward = s_car.ss_cockpit.speedLimiter;
                    s_car.ss_system.car_status = LIMITED_SPEED_VEHICLE;
        }
        else{
            s_car.ss_controlMotor.speedLimitForward = 220;
        }
        cpt_speed_limitation = 0;
    }
    cpt_speed_limitation++;
}

/* direction control*/

void directionControl(void){
    if (cpt_direction_control == 100000){
        if ( s_car.ss_system.car_status == CAR_DIRECTION){
            if (s_car.ss_controlMotor.direction == 1 ){
                s_car.ss_system.car_status = TURNS_RIGHT;
            }
            else if (s_car.ss_controlMotor.direction == 2 ){
                s_car.ss_system.car_status = TURNS_LEFT;
            }
        }
        cpt_direction_control = 0;
    }
    cpt_direction_control++;
}
