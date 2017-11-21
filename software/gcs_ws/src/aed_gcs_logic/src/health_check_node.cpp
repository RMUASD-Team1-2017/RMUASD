#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/BatteryStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <aed_gcs_logic/HealthCheckService.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
int BatCondi = 0;
int GPSCondition = 0;
float MinCellVoltage = 11.5;
using namespace std;

void printBatteryInformation(sensor_msgs::BatteryState batteryState){
  std::cout << "Voltage: \t" << batteryState.voltage << std::endl;
  std::cout << "Current: \t" << batteryState.current << std::endl;
  std::cout << "Charge: \t" << batteryState.charge << std::endl;
  std::cout << "Capacity: \t" << batteryState.capacity << std::endl;
  std::cout << "Design capacity: \t" << batteryState.design_capacity << std::endl;
  std::cout << "Percentage: \t" << batteryState.percentage << std::endl;

  std::cout << "Power supply status: ";
  switch (batteryState.power_supply_status){
    case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN: std::cout << "unknown" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING: std::cout << "charging" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING: std::cout << "discharging" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING: std::cout << "not charging" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL: std::cout << "full" << std::endl;
  }

  std::cout << "Power supply health: ";
  switch (batteryState.power_supply_health){
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN: std::cout << "unknown" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD: std::cout << "good" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT: std::cout << "overheat" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD: std::cout << "dead" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE: std::cout << "overvoltage" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: std::cout << "unspec failure" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_COLD: std::cout << "cold" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: std::cout << "watchdog timer expire" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: std::cout << "safety timer expire" << std::endl;
  }

  std::cout << "Power supply technology: ";
  switch (batteryState.power_supply_technology){
    case sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN: std::cout << "unknown" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH: std::cout << "NIMH" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION: std::cout << "LION" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO: std::cout << "LIPO" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE: std::cout << "LIFE" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_NICD: std::cout << "NICD" << std::endl;
    case sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN: std::cout << "LIMN" << std::endl;
  }

  if (batteryState.present) std::cout << "Battery is present!" << std::endl;
  else std::cout << "Battery is not present!" << std::endl;

  for (int i = 0; i < sizeof(batteryState.cell_voltage) / sizeof(float) / 2; i++){
    std::cout << "Cell " << i << ": " << batteryState.cell_voltage[i] << std::endl;
  }

  std::cout << "Location: " << batteryState.location << std::endl;
  std::cout << "Serial number" << batteryState.serial_number << std::endl;
}


int batteryFLy(sensor_msgs::BatteryState batteryState){
  int BatteryStat = 0;
  if (batteryState.present){
    //std::cout << "Battery is present!" << std::endl;


          if (batteryState.voltage >= MinCellVoltage) {  // should be 4.2 in real life, the 4V is only for test.

            BatteryStat = 1;
          }
          else {
          //  std::cout<<"Battery are not fully charged!"<<std::endl;
             BatteryStat=0;
             return BatteryStat;
          }


  }
  else std::cout << "Battery is not present!" << std::endl, BatteryStat=0;

  //std::cout<<" stat! "<<BatteryStat<<std::endl;
  //std::cout << "Location: " << batteryState.location << std::endl;
  return BatteryStat;
}

int GPSstate(sensor_msgs::NavSatFix globalGpsState){
  int GPSConnection = 0;
  if (globalGpsState.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    GPSConnection = 0;
    std::cout << "There are No GPS fix! " << std::endl;
  }
  else {

    if (globalGpsState.status.status == sensor_msgs::NavSatStatus::STATUS_SBAS_FIX) {
      GPSConnection = 1;
      std::cout << "There are SBAS fix " << std::endl;
    }
    else if (globalGpsState.status.status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
      GPSConnection = 1;
      std::cout << "There are GBAS fix " << std::endl;
    }
    else {
      //std::cout << "There are GPS fix "  << std::endl;
      GPSConnection = 1;
    }

  }
  return GPSConnection;

}


void printGlobalGpsState(sensor_msgs::NavSatFix globalGpsState){
  std::cout << "GPS status: ";
  switch (globalGpsState.status.status){
    case sensor_msgs::NavSatStatus::STATUS_NO_FIX: std::cout << "no fix" << std::endl; break;
    case sensor_msgs::NavSatStatus::STATUS_FIX: std::cout << "fix" << std::endl; break;
    case sensor_msgs::NavSatStatus::STATUS_SBAS_FIX: std::cout << "SBAS fix" << std::endl; break;
    case sensor_msgs::NavSatStatus::STATUS_GBAS_FIX: std::cout << "GBAS fix" << std::endl; break;
  }
  std::cout << "Service type:";
  switch (globalGpsState.status.service){
    case sensor_msgs::NavSatStatus::SERVICE_GPS: std::cout << "GPS" << std::endl; break;
    case sensor_msgs::NavSatStatus::SERVICE_GLONASS: std::cout << "GLONASS" << std::endl; break;
    case sensor_msgs::NavSatStatus::SERVICE_COMPASS: std::cout << "COMPASS" << std::endl; break;
    case sensor_msgs::NavSatStatus::SERVICE_GALILEO: std::cout << "GALILEO" << std::endl; break;
  }
  std::cout << "Latitude: " << globalGpsState.latitude << std::endl;
  std::cout << "Longitude: " << globalGpsState.longitude << std::endl;
  std::cout << "Altitude: " << globalGpsState.altitude << std::endl;
}

// state subscriber
sensor_msgs::BatteryState batteryState;
void batteryStateSub(const sensor_msgs::BatteryState::ConstPtr &msg){
	batteryState = *msg;
  printBatteryInformation(batteryState);
}

nav_msgs::Odometry localGpsState;
void localGpsStateSub(const nav_msgs::Odometry::ConstPtr &msg){
	localGpsState = *msg;
}

sensor_msgs::NavSatFix globalGpsState;
void globalGpsStateSub(const sensor_msgs::NavSatFix::ConstPtr &msg){
	globalGpsState = *msg;

  //printGlobalGpsState(globalGpsState);
  GPSCondition = GPSstate(globalGpsState);
  //std::cout<<"GPS condition "<<GPSCondition<<std::endl;
}

bool isFlyingOkay(aed_gcs_logic::HealthCheckService::Request  &req, aed_gcs_logic::HealthCheckService::Response &res){


  if (BatCondi == 1 && GPSCondition == 1) {
    res.flight = true;
    return true;
  }
   else {
     res.flight = false;   //## Look here, it should be False !!!!!!!!!!!!!!!!!!!!!!!!!!!
    return true;
  }

}

int main(int argc, char **argv){

	// Init ros node
	ros::init(argc, argv, "Health_check_node");
	ros::NodeHandle nh;
	ros::Rate rate(20);

	// Setup ros subscribtions
	ros::Subscriber mavrosBatteryStateSub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 10, batteryStateSub);
	ros::Subscriber mavrosGlobalGpsStateSub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, globalGpsStateSub);
	//ros::Subscriber mavrosLocalGpsStateSub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, localGpsStateSub);


	ros::Subscriber mavrosLocalGpsStateSub = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1, localGpsStateSub);
  ros::ServiceServer service = nh.advertiseService("drone/Health_check_service", isFlyingOkay);


	// Save time stamp
	ros::Time last = ros::Time::now();

	while (ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
