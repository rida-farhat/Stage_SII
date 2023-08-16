#include "FreeRTOS.h"
#include "task.h"
#include "IfxPort.h"
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "MCMCAN.h"
#include "UART_VCOM.h"
#include "..\AppSw\System\carManagement.h"
#include "SD_CARD.h"
#include "..\..\MCMCAN.h"
#include "..\..\STM_System_Time.h"
#include "..\..\Libraries\Service\CpuGeneric\SysSe\Bsp\Bsp.h"
#include "..\..\UART_VCOM.h"
#include "..\AppSw\TestCar\testCar.h"
#include "..\AppSw\PowerLatch\powerlatch.h"
#include "SD_CARD.h"
#define powerlatch_timeout 30   /* The time in second to wait before the trig of powerlatch */

IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;
t_car s_car;
int alertInitCar = 1;   // an alert must be done if the data initialization is not OK

systemTime start_g_time;
extern systemTime g_time;
int initCarStatus = 0;

int powerlatch_waiting = 0;     /* set at 1 if the waiting is started otherwise to 0 */
systemTime powerlatch_start_time;
powerlatch_critical_emergency = 0;

char * p_string_cpuLoad = "CpuLoad =0x";  // constant string use to display with USB UART
char * p_string_alert = "! ALERT !";

cpu_load_state = 1;                     // set for running CPU Load at the start of application

extern int cpu_send_can;
int cpu_send_can = 1;
extern t_car s_car;
extern t_car s_car;                             // define of car data
int powerlatch_error = 0;
long int cpt_number_loops;

/* Les fonctions necessaires */
int carInit(void){
    /* system data */
    s_car.ss_system.my_cpt = 0;
    s_car.ss_system.car_status = INITIAL;
    s_car.ss_system.powerlatch = 0;
    s_car.ss_system.powerlatch_saved = 0;

    /* motion control data */
    s_car.ss_controlMotor.igk = 0;
    s_car.ss_controlMotor.tempMotor = 15;
    s_car.ss_controlMotor.brake = 0;
    s_car.ss_controlMotor.handbrake = 1;
    s_car.ss_controlMotor.acceleration = 0;
    s_car.ss_controlMotor.speed = 0;
    s_car.ss_controlMotor.speedDimmer = 0;
    s_car.ss_controlMotor.gearSpeedStatus = 0;
    s_car.ss_controlMotor.engineTorque = 0;
    s_car.ss_controlMotor.consumption = 0;
    s_car.ss_controlMotor.speedLimitForward = 220;
    s_car.ss_controlMotor.speedLimitBackward = 20;

    s_car.ss_controlMotor.refreshValve = 0;

    s_car.ss_controlMotor.direction = 0;
    /* signalling data */
    s_car.ss_signalling.roadLightStatus = 0;
    s_car.ss_signalling.rearLightStatus = 0;
    s_car.ss_signalling.blinkerStatus = 0;
    s_car.ss_signalling.airbagLight = 0;
    s_car.ss_signalling.batteryLight = 90;
    s_car.ss_signalling.seatbeltLight = 0;
    s_car.ss_signalling.defaultAlarm = 0;
    s_car.ss_signalling.stateTrunk = 0;
    s_car.ss_signalling.rightDoorStatus = 0;
    s_car.ss_signalling.leftDoorStatus = 0;
    s_car.ss_signalling.AirbagStatus = 0;
    s_car.ss_signalling.roadStripDetector = 0;
    s_car.ss_signalling.forwardDistanceSensor = 0;
    s_car.ss_signalling.rearDistanceSensor = 0;
    s_car.ss_signalling.seatbeltStatus = 0;

    s_car.ss_signalling.roadLight = 0;
    s_car.ss_signalling.blinker = 0;
    s_car.ss_signalling.rightDoorLight = 0;
    s_car.ss_signalling.leftDoorLight = 0;
    s_car.ss_signalling.trunkLight = 0;
    /* battery data */
    s_car.ss_battery.batteryTemp = 15;
    s_car.ss_battery.batteryLevel = 90;
    s_car.ss_battery.batteryVoltage = 18;
    s_car.ss_battery.batteryStatus = 1;
    s_car.ss_battery.chargingCord = 0;

    s_car.ss_battery.batteryVentilation = 0;
    /* cockpit data */
    s_car.ss_cockpit.indoorTemp = 15;
    s_car.ss_cockpit.airConditioning = 0;
    s_car.ss_cockpit.speedDisplay = 0;
    s_car.ss_cockpit.brightnessSensor = 0;
    s_car.ss_cockpit.humiditySensor = 0;
    s_car.ss_cockpit.cruiseControl = 0;
    s_car.ss_cockpit.speedLimiter = 0;

    s_car.ss_cockpit.indoorLight = 0;


    return (1);
}




void carInitImpossible(t_car * car){
    /* system data */
    car->ss_system.my_cpt = 200000;
    car->ss_system.car_status = 99;
    car->ss_system.powerlatch = 99;
    car->ss_system.powerlatch_saved = 99;

    /* motion control data */
    car->ss_controlMotor.igk = 99;
    car->ss_controlMotor.tempMotor = 999;
    car->ss_controlMotor.brake = 99;
    car->ss_controlMotor.handbrake = 99;
    car->ss_controlMotor.acceleration = 99;
    car->ss_controlMotor.speed = 999;
    car->ss_controlMotor.speedDimmer = 99;
    car->ss_controlMotor.gearSpeedStatus = 99;
    car->ss_controlMotor.engineTorque = 999;
    car->ss_controlMotor.consumption = 999;
    car->ss_controlMotor.speedLimitForward = 999;
    car->ss_controlMotor.speedLimitBackward = 99;

    car->ss_controlMotor.refreshValve = 99;

    car->ss_controlMotor.direction = 99;
    /* signalling data */
    car->ss_signalling.roadLightStatus = 99;
    car->ss_signalling.rearLightStatus = 99;
    car->ss_signalling.blinkerStatus = 99;
    car->ss_signalling.airbagLight = 99;
    car->ss_signalling.batteryLight = 999;
    car->ss_signalling.seatbeltLight = 99;
    car->ss_signalling.defaultAlarm = 99;
    car->ss_signalling.stateTrunk = 99;
    car->ss_signalling.rightDoorStatus = 99;
    car->ss_signalling.leftDoorStatus = 99;
    car->ss_signalling.AirbagStatus = 99;
    car->ss_signalling.roadStripDetector = 99;
    car->ss_signalling.forwardDistanceSensor = 99;
    car->ss_signalling.rearDistanceSensor = 99;
    car->ss_signalling.seatbeltStatus = 99;

    car->ss_signalling.roadLight = 99;
    car->ss_signalling.blinker = 99;
    car->ss_signalling.rightDoorLight = 99;
    car->ss_signalling.leftDoorLight = 99;
    car->ss_signalling.trunkLight = 99;
    /* battery data */
    car->ss_battery.batteryTemp = 999;
    car->ss_battery.batteryLevel = 999;
    car->ss_battery.batteryVoltage = 999;
    car->ss_battery.batteryStatus = 99;
    car->ss_battery.chargingCord = 99;

    car->ss_battery.batteryVentilation = 99;
    /* cockpit data */
    car->ss_cockpit.indoorTemp = 99;
    car->ss_cockpit.airConditioning = 99;
    car->ss_cockpit.speedDisplay = 999;
    car->ss_cockpit.brightnessSensor = 99;
    car->ss_cockpit.humiditySensor = 999;
    car->ss_cockpit.cruiseControl = 99;
    car->ss_cockpit.speedLimiter = 99;

    car->ss_cockpit.indoorLight = 99;
}







// Taches
void initTask(void *pvParameters) {
    carInit();
    initMcmcan();
    initSD_CARD();
    initLeds();
    initIO(IGK_INPUT_PIN);
    /* Initialize a time variable */
//    Ifx_TickTime ticksFor1s = IfxStm_getTicksFromMilliseconds(BSP_DEFAULT_TIMER, WAIT_TIME);

//    systemTime start_g_time;
//    int initCarStatus = 0;

//    int powerlatch_waiting = 0;     /* set at 1 if the waiting is started otherwise to 0 */
//    systemTime powerlatch_start_time;
//    powerlatch_critical_emergency = 0;

//    char * p_string_cpuLoad = "CpuLoad =0x";  // constant string use to display with USB UART
//    char * p_string_alert = "! ALERT !";

    cpu_load_state = 1;                     // set for running CPU Load at the start of application

    init_UART();

    vTaskDelete(NULL);
}


void initDataTask(void *pvParameters) {
    int initCarStatus;

    if(!(initCarStatus = carInit())) {
        s_car.ss_system.car_status = ALERT;
        //return (-1);
    }

    if(s_car.ss_system.car_status != ALERT) {
        s_car.ss_system.car_status = CAR_STOPPED;
    } else {
        send_UART_string(p_string_alert);  // display a string on USB UART Terminal
    }

    vTaskDelete(NULL);
}


void igkTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* igk pin management */
        if (!IfxPort_getPinState(IGK_INPUT_PIN)) {
            s_car.ss_controlMotor.igk = 1;
        } else {
            s_car.ss_controlMotor.igk = 0;
        }
    }
}

//int powerlatch_waiting = 0;     /* set at 1 if the waiting is started otherwise to 0 */
//systemTime powerlatch_start_time;
//powerlatch_critical_emergency = 0;
void StartCarTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if ((s_car.ss_controlMotor.brake == 1) && (s_car.ss_controlMotor.handbrake == 0) &&
            (s_car.ss_controlMotor.acceleration == 0) && (s_car.ss_controlMotor.speed == 0) &&
            (s_car.ss_controlMotor.gearSpeedStatus == 0) && (s_car.ss_controlMotor.engineTorque < 5) &&
            (s_car.ss_battery.batteryVoltage > 11) && (s_car.ss_controlMotor.igk == 1) &&
            ((s_car.ss_system.car_status == CAR_STOPPED) || (s_car.ss_system.car_status == POWERLATCH) || (s_car.ss_system.car_status == POWERLATCH_SAVE)))
        {
            if (s_car.ss_system.powerlatch_saved == 1) {
                carInitImpossible(&s_car);
                get_data_from_sd_card(&s_car);
                s_car.ss_system.car_status = CAR_STARTED;
                s_car.ss_system.powerlatch_saved = 0;
            }

            s_car.ss_system.car_status = CAR_STARTED;
        }
        if(s_car.ss_system.powerlatch == 1 && s_car.ss_system.car_status == POWERLATCH_SAVE){
                 save_data_to_sd_card(&s_car);
                 s_car.ss_system.powerlatch = 0;
                 if(powerlatch_error == 0){
                     s_car.ss_system.powerlatch_saved = 1;
                 }
                 else{
                     s_car.ss_system.powerlatch_saved = 0;
                 }

       }
}
}

void PowerlatchTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (s_car.ss_system.powerlatch == 1 && s_car.ss_system.car_status == POWERLATCH_SAVE) {
            save_data_to_sd_card(&s_car);
            s_car.ss_system.powerlatch = 0;

            if (powerlatch_error == 0) {
                s_car.ss_system.powerlatch_saved = 1;
            } else {
                s_car.ss_system.powerlatch_saved = 0;
            }
        }
        if((s_car.ss_system.car_status == CAR_STARTED) && (s_car.ss_controlMotor.igk == 0))
                    {
                        s_car.ss_system.car_status = POWERLATCH;
                    }

        if(s_car.ss_system.car_status == POWERLATCH || s_car.ss_system.car_status == POWERLATCH_SAVE )
                                    {
                                        if (powerlatch_waiting == 0)
                                        {
                                            /* we start to wait 20s */
//                                            getTime();
                                            powerlatch_start_time.totalSeconds = g_time.totalSeconds;
                                            powerlatch_waiting = 1;
                                            s_car.ss_system.powerlatch = 1;
                                            s_car.ss_system.car_status = POWERLATCH_SAVE;
                                        }
                                        else
                                        {

                                            /* ckeck timeout */
//                                            getTime();
                                            if (g_time.totalSeconds - powerlatch_start_time.totalSeconds >= powerlatch_timeout)
                                            {
                                                powerlatch_waiting = 0;
                                                if(s_car.ss_system.powerlatch_saved == 1){
                                                    s_car.ss_system.car_status = CAR_STOPPED;
                                                }else{
                                                    s_car.ss_system.car_status = ALERT;
                                                }

                                            }
                                        }
                                    }
                                    else
                                    {
                                        powerlatch_waiting = 0;
                                    }
                                    if (powerlatch_critical_emergency == TRUE)
                                    {
                                        /* launch powerlatch treatment */
                                        powerlatch_waiting = 0;
                                        /* save data */
                                        powerlatch_save(s_car.ss_battery.batteryLevel);
                                        save_data_to_sd_card(&s_car);
                                        s_car.ss_system.car_status = CAR_STOPPED;
                                        s_car.ss_system.powerlatch = 1;
                                    }

    }
}


void SystemCounterTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        s_car.ss_system.my_cpt++;      // increase the system count - that to verify system running

        if (s_car.ss_system.my_cpt == 100000) {
            s_car.ss_system.my_cpt = 0;
        }
    }
}


void CANTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        transmitCanMessage_car_status(s_car.ss_system.car_status);
        transmitCanMessage_igk_my_cpt(s_car.ss_controlMotor.igk, s_car.ss_system.my_cpt);
        transmitCanMessage_speed(s_car.ss_controlMotor.speed, s_car.ss_controlMotor.speedDimmer);
        //transmitCanMessage_tempMotor_cons_engTor(s_car.ss_controlMotor.tempMotor,
        //                                 s_car.ss_controlMotor.consumption, s_car.ss_controlMotor.engineTorque);
        transmitCanMessage_gearSpeedStatus(s_car.ss_controlMotor.gearSpeedStatus);
        transmitCanMessage_brake(s_car.ss_controlMotor.brake, s_car.ss_controlMotor.handbrake);
        transmitCanMessage_acc(s_car.ss_controlMotor.acceleration);
    }
}

extern McmcanType           g_mcmcan;/* define in MCMCAN.c  */

void HandleReceivedMessageTask(void *pvParameters) {

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        IfxCan_Can_readMessage(&g_mcmcan.canDstNode, &g_mcmcan.rxMsg, g_mcmcan.rxData);

        if (g_mcmcan.rxMsg.messageId == (uint32)0x300) {
            received_0x300message();
        }
    }
}


void core0_main(void) {
    IfxCpu_enableInterrupts();

    /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
     * Enable the watchdogs and service them periodically if it is required
     */
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    /* Wait for CPU sync event */
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);
    carInit();
    initMcmcan();
    initSD_CARD();
    initLeds();
    initIO(IGK_INPUT_PIN);
    init_UART();
//    xTaskCreate(initTask, "InitTask", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
//    xTaskCreate(initDataTask, "InitDataTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(igkTask, "IGKTask", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
    xTaskCreate(StartCarTask, "StartCarTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(PowerlatchTask, "PowerlatchTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(SystemCounterTask, "SystemCounterTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(CANTask, "CANTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(HandleReceivedMessageTask, "HandleReceivedMessageTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);

    // Démarrage du planificateur FreeRTOS
    vTaskStartScheduler();

    while (1) {
        // Boucle d'attente pour empêcher le programme de se terminer
    }
}
