#include "ROSbot.h"

namespace hFramework
{
ROSbot rosbot;

void ROSbot::initROSbot(SensorType s, ImuType i)
{
    Serial.printf("ROSbot initialization begin\n");
#ifdef MY_ROSBOT2
    tick_to_rad = 2 * 3.14159 / enc_res;
    resetWheelStates();
#endif
    initIMU(i);
    initBatteryMonitor();
    initOdometry();
    initDistanceSensors(s);
    initWheelController();
    return;
}

void ROSbot::initWheelController()
{
#ifdef MY_ROSBOT2
    wheelFL = new Wheel(hMotD, 0);      
    wheelRL = new Wheel(hMotC, 0);
    wheelFR = new Wheel(hMotA, 1);      // reverse
    wheelRR = new Wheel(hMotB, 1);      // reverse
#else    
    wheelFL = new Wheel(hMotD, 1);
    wheelRL = new Wheel(hMotC, 1);
    wheelFR = new Wheel(hMotA, 0);
    wheelRR = new Wheel(hMotB, 0);
#endif

    wheelFL->begin();
    wheelRL->begin();
    wheelFR->begin();
    wheelRR->begin();
    sys.taskCreate(std::bind(&ROSbot::wheelUpdater, this));
}

void ROSbot::wheelUpdater()
{
    uint32_t t = sys.getRefTime();
    long dt = 10;
    for (;;)
    {
        wheelFR->update(dt);
        wheelRR->update(dt);
        wheelRL->update(dt);
        wheelFL->update(dt);
        sys.delaySync(t, dt);
    }
}

void ROSbot::setSpeed(float linear, float angular)
{
    lin = linear;
    ang = angular;

    L_wheel_lin_speed = lin - (ang * robot_width / 2);
    R_wheel_lin_speed = lin + (ang * robot_width / 2);
    L_wheel_angular_velocity = L_wheel_lin_speed / wheel_radius;
    R_wheel_angular_velocity = R_wheel_lin_speed / wheel_radius;
    L_enc_speed = enc_res * L_wheel_angular_velocity / (2 * M_PI);
    R_enc_speed = enc_res * R_wheel_angular_velocity / (2 * M_PI);

    wheelFL->setSpeed(L_enc_speed);
    wheelRL->setSpeed(L_enc_speed);
    wheelFR->setSpeed(R_enc_speed);
    wheelRR->setSpeed(R_enc_speed);
}

void ROSbot::batteryMonitor()
{
    int i = 0;
    int publish_cnt = 0;
    for (;;)
    {
        if (sys.getSupplyVoltage() > voltage_limit)
        {
            i--;
        }
        else
        {
            i++;
        }
        if (i > 50)
        {
            batteryLow = true;
            i = 50;
        }
        if (i < -50)
        {
            batteryLow = false;
            i = -50;
        }
        if (batteryLow == true)
        {
            LED1.toggle();
        }
        else
        {
            LED1.on();
        }
        if (publish_cnt == 8)
        {
            current_voltage = sys.getSupplyVoltage();
            // battery_pub->publish(&battery);
            publish_cnt = 0;
            // Serial.printf("Battery monitor running\n");
        }
        publish_cnt++;
        sys.delay(250);
    }
}

void ROSbot::initBatteryMonitor(float voltage_threshold)
{
#ifndef MY_ROSBOT2      // for optimization    
    voltage_limit = voltage_threshold;
    sys.taskCreate(std::bind(&ROSbot::batteryMonitor, this));
#endif
}

float ROSbot::getBatteryLevel()
{
    return current_voltage;
}

void ROSbot::initOdometry()
{
    delay_s = (float)loop_delay / (float)1000;
    Serial.printf("Create odometryUpdater task\n");
    sys.taskCreate(std::bind(&ROSbot::odometryUpdater, this));
}

#ifdef  MY_ROSBOT2

void ROSbot::odometryUpdater()
{
    float curr_ang_pos;

    while (true)
    {
        enc_FR = wheelFR->getDistance();
        enc_RR = wheelRR->getDistance();
        enc_RL = wheelRL->getDistance();
        enc_FL = wheelFL->getDistance();

        wheel_FL_ang_pos = tick_to_rad * enc_FL;
        wheel_FR_ang_pos = tick_to_rad * enc_FR;
        wheel_RL_ang_pos = tick_to_rad * enc_RL;
        wheel_RR_ang_pos = tick_to_rad * enc_RR;

        wheelStates.angVel[FL] = (wheel_FL_ang_pos - wheelStates.angPos[FL]) / delay_s;
        wheelStates.angPos[FL] = wheel_FL_ang_pos;        
        wheelStates.angVel[FR] = (wheel_FR_ang_pos - wheelStates.angPos[FR]) / delay_s;
        wheelStates.angPos[FR] = wheel_FR_ang_pos;        
        wheelStates.angVel[RR] = (wheel_RR_ang_pos - wheelStates.angPos[RR]) / delay_s;
        wheelStates.angPos[RR] = wheel_RR_ang_pos;
        wheelStates.angVel[RL] = (wheel_RL_ang_pos - wheelStates.angPos[RL]) / delay_s;
        wheelStates.angPos[RL] = wheel_RL_ang_pos;        

        enc_L = (enc_FL + enc_RL) / (2 * tyre_deflection);
        enc_R = (enc_FR + enc_RR) / (2 * tyre_deflection);

        curr_ang_pos = tick_to_rad * enc_L;
        wheel_L_ang_vel = (curr_ang_pos - wheel_L_ang_pos) / delay_s;
        wheel_L_ang_pos = curr_ang_pos;

        curr_ang_pos = tick_to_rad * enc_R;
        wheel_R_ang_vel = (curr_ang_pos - wheel_R_ang_pos) / delay_s;
        wheel_R_ang_pos = curr_ang_pos;

        curr_ang_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / (robot_width * diameter_mod);
        robot_angular_vel = (curr_ang_pos - robot_angular_pos) / delay_s;
        robot_angular_pos = curr_ang_pos;

        curr_ang_pos = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2);
        robot_x_vel = curr_ang_pos * cos(robot_angular_pos);
        robot_y_vel = curr_ang_pos * sin(robot_angular_pos);
        robot_x_pos = robot_x_pos + robot_x_vel * delay_s;
        robot_y_pos = robot_y_pos + robot_y_vel * delay_s;
        
        sys.delay(loop_delay);
    }
}

std::vector<float> ROSbot::getVelocity()
{
    rosbotVelocity.clear();
    rosbotVelocity.push_back(robot_x_vel);
    rosbotVelocity.push_back(robot_y_vel);
    rosbotVelocity.push_back(robot_angular_vel);
    return rosbotVelocity;
}

void ROSbot::resetWheelStates()
{
    for (int i=0; i<4; i++) {
        wheelStates.angPos[i] = 0.0;
        wheelStates.angVel[i] = 0.0;
    }
}

WheelStates ROSbot::getWheelStates()
{
    return wheelStates;
}

#else
wheelsState ROSbot::getWheelsState()
{
    wheelsState ws;
    ws.FL = wheel_FL_ang_pos;
    ws.FR = wheel_FR_ang_pos;
    ws.RL = wheel_RL_ang_pos;
    ws.RR = wheel_RR_ang_pos;
    return ws;
}

void ROSbot::odometryUpdater()
{
    while (true)
    {
        enc_FR = wheelFR->getDistance();
        enc_RR = wheelRR->getDistance();
        enc_RL = wheelRL->getDistance();
        enc_FL = wheelFL->getDistance();

        wheel_FL_ang_pos = 2 * 3.14 * enc_FL / enc_res;
        wheel_FR_ang_pos = 2 * 3.14 * enc_FR / enc_res;
        wheel_RL_ang_pos = 2 * 3.14 * enc_RL / enc_res;
        wheel_RR_ang_pos = 2 * 3.14 * enc_RR / enc_res;

        enc_L = (enc_FL + enc_RL) / (2 * tyre_deflection);
        enc_R = (enc_FR + enc_RR) / (2 * tyre_deflection);

        wheel_L_ang_vel = ((2 * 3.14 * enc_L / enc_res) - wheel_L_ang_pos) / delay_s;
        wheel_R_ang_vel = ((2 * 3.14 * enc_R / enc_res) - wheel_R_ang_pos) / delay_s;

        wheel_L_ang_pos = 2 * 3.14 * enc_L / enc_res;
        wheel_R_ang_pos = 2 * 3.14 * enc_R / enc_res;

        robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / (robot_width * diameter_mod)) - robot_angular_pos) / delay_s;
        robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / (robot_width * diameter_mod);
        robot_x_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * cos(robot_angular_pos);
        robot_y_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * sin(robot_angular_pos);
        robot_x_pos = robot_x_pos + robot_x_vel * delay_s;
        robot_y_pos = robot_y_pos + robot_y_vel * delay_s;
        sys.delay(loop_delay);
    }
}
#endif  // MY_ROSBOT2

std::vector<float> ROSbot::getPose()
{
    rosbotPose.clear();
    rosbotPose.push_back(robot_x_pos);
    rosbotPose.push_back(robot_y_pos);
    rosbotPose.push_back(robot_angular_pos);
    return rosbotPose;
}

void ROSbot::reset_encoders()
{
    hMot1.resetEncoderCnt();
    hMot2.resetEncoderCnt();
    hMot3.resetEncoderCnt();
    hMot4.resetEncoderCnt();
}

void ROSbot::reset_odom_vars()
{
    enc_FL = 0;            // encoder tics
    enc_RL = 0;            // encoder tics
    enc_FR = 0;            // encoder tics
    enc_RR = 0;            // encoder tics
    enc_L = 0;             // encoder tics
    wheel_L_ang_pos = 0;   // radians
    wheel_L_ang_vel = 0;   // radians per second
    enc_R = 0;             // encoder tics
    wheel_R_ang_pos = 0;   // radians
    wheel_R_ang_vel = 0;   // radians per second
    robot_angular_pos = 0; // radians
    robot_angular_vel = 0; // radians per second
    robot_x_pos = 0;       // meters
    robot_y_pos = 0;       // meters
    robot_x_vel = 0;       // meters per second
    robot_y_vel = 0;       // meters per second
}

void ROSbot::reset_wheels()
{
    wheelFR->reset();
    wheelRR->reset();
    wheelRL->reset();
    wheelFL->reset();
}

void ROSbot::reset_odometry()
{
    reset_wheels();
    reset_odom_vars();
}

void ROSbot::initDistanceSensors(SensorType s)
{
    sensor_type = s;
    switch (sensor_type)
    {
    case SENSOR_LASER:
        hSens1.selectI2C();
        VL53L0XXshutPort.pin1.setOut();
        VL53L0XXshutPort.pin1.write(false);
        VL53L0XXshutPort.pin2.setOut();
        VL53L0XXshutPort.pin2.write(false);
        VL53L0XXshutPort.pin3.setOut();
        VL53L0XXshutPort.pin3.write(false);
        VL53L0XXshutPort.pin4.setOut();
        VL53L0XXshutPort.pin4.write(false);
        VL53L0XXshutPort.pin1.reset();
        sensor_dis[0].setAddress(44);
        sys.delay(100);
        VL53L0XXshutPort.pin2.reset();
        sensor_dis[1].setAddress(43);
        sys.delay(100);
        VL53L0XXshutPort.pin3.reset();
        sensor_dis[2].setAddress(42);
        sys.delay(100);
        VL53L0XXshutPort.pin4.reset();
        sys.delay(100);
        sensor_dis[0].setTimeout(500);
        sensor_dis[1].setTimeout(500);
        sensor_dis[2].setTimeout(500);
        sensor_dis[3].setTimeout(500);
        sensor_dis[0].init();
        sensor_dis[1].init();
        sensor_dis[2].init();
        sensor_dis[3].init();
        sensor_dis[0].startContinuous();
        sensor_dis[1].startContinuous();
        sensor_dis[2].startContinuous();
        sensor_dis[3].startContinuous();
        sys.taskCreate(std::bind(&ROSbot::rangesTask, this));
        break;
    case SENSOR_INFRARED:
        sensMUX = new hSensor(hSens_ID_3);
        sys.taskCreate(std::bind(&ROSbot::rangesTask, this));
        break;
    default:
        break;
    }
}

void ROSbot::readIRSensors()
{
    sensMUX->pin1.enableADC();
    sensMUX->pin2.setOut();
    sensMUX->pin3.setOut();
    sensMUX->pin4.setOut();
    for (size_t i = 0; i < 8; i++)
    {
        if (tMUX[i].active == true)
        {
            sensMUX->pin2.write(tMUX[i].p2);
            sensMUX->pin3.write(tMUX[i].p3);
            sensMUX->pin4.write(tMUX[i].p4);
            sys.delay(MUXStepTime);
            float temp = 20 * (1 / (sensMUX->pin1.analogReadRaw() * 0.001220703));
            if (temp > 30)
                temp = 30;
            if (temp < 4)
                temp = 4;
            *tMUX[i].dis = temp * 0.01;
        }
    }
}

int ROSbot::readLaserDistanceSensor(VL53L0X &s)
{
    return s.readRangeContinuousMillimeters();
}

std::vector<float> ROSbot::getRanges(SensorType s)
{
    ranges.clear();
    ranges.push_back(dis1);
    ranges.push_back(dis2);
    ranges.push_back(dis3);
    ranges.push_back(dis4);
    return ranges;
}

void ROSbot::rangesTask()
{
    while (true)
    {
        switch (sensor_type)
        {
        case SENSOR_LASER:
            dis2 = 0.001 * ((float)readLaserDistanceSensor(sensor_dis[0]));
            dis1 = 0.001 * ((float)readLaserDistanceSensor(sensor_dis[1]));
            dis3 = 0.001 * ((float)readLaserDistanceSensor(sensor_dis[2]));
            dis4 = 0.001 * ((float)readLaserDistanceSensor(sensor_dis[3]));
            break;
        case SENSOR_INFRARED:
            readIRSensors();
            break;
        }
        sys.delay(50);
    }
}

void ROSbot::initIMU(ImuType i)
{
    switch (i)
    {
    case MPU9250:
        imu.begin();
        imu.resetFifo();
        sys.taskCreate(std::bind(&ROSbot::eulerAnglesTask, this));
        break;
    case NO_IMU:
        break;
    }
}

std::vector<float> ROSbot::getRPY()
{
//#ifdef  MY_ROSBOT2	// compensate RPY values
//    imuArray.clear();
//    imuArray.push_back(roll - initRoll);
//    imuArray.push_back(pitch - initPitch);
//    imuArray.push_back(yaw - initYaw);
//    return imuArray;
//#else
    imuArray.clear();
    imuArray.push_back(roll);
    imuArray.push_back(pitch);
    imuArray.push_back(yaw);
    return imuArray;
//#endif
}

void ROSbot::eulerAnglesTask()
{
    while (true)
    {
        imu.getEulerAngles(&roll, &pitch, &yaw);
        sys.delay(50);
    }
}

#ifdef  MY_ROSBOT2      // Experimental function
void ROSbot::compensateRPY()
{
    initRoll = 0; initPitch = 0; initYaw = 0;
    for (int i=0; i<20; i++) {
        imu.getEulerAngles(&roll, &pitch, &yaw);
        initRoll += roll;
        initPitch += pitch;
        initYaw += yaw;
    	Serial.printf("R/P/Y reading: %f, %f, %f\r\n", roll, pitch, yaw);
        sys.delay(10);
    }
    initRoll = initRoll / 20;
    initPitch = initPitch / 20;
    initYaw = initYaw / 20;
    Serial.printf("init RPY: %f, %f, %f\r\n", initRoll, initPitch, initYaw);
}
#endif  

} // namespace hFramework
