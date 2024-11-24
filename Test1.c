#include <webots/robot.h>
#include <webots/light_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/led.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#define TIME_STEP 64  // Time step for simulation in ms
#define MAX_V 6.28 //Max Speed of Epuck in m/s

int main(int argc, char **argv)
{
  wb_robot_init();
  //Enabling distance sensors and storing distance sensors in an array
  int i;
  WbDeviceTag distance_sensors[8];
  for (i=0;i<8;i++)
  {
    char sensor_name[4];
    sprintf(sensor_name, "ps%d", i);
    distance_sensors[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(distance_sensors[i], TIME_STEP);
  }
  
  //Enabling light sensors and storing light sensors in an array
  WbDeviceTag light_sensors[9];
  for (i=0;i<9;i++)
  {
    char sensor_name[4];
    sprintf(sensor_name, "ls%d", i);
    light_sensors[i] = wb_robot_get_device(sensor_name);
    wb_light_sensor_enable(light_sensors[i], TIME_STEP);
  }
  
  //Storing LED in an array
  WbDeviceTag led[8];
  for (int i = 0; i < 8; i++) {
    char led_name[10];
    sprintf(led_name, "led%d", i); 
    led[i] = wb_robot_get_device(led_name);
  }
  
  // Enabling GPS sensor
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  //Enabling Motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor,INFINITY);
  wb_motor_set_position(right_motor,INFINITY);
  
  double max_light_intensity=0; //initializing max light intensity
  double avg_light_intensity=0; //initializing average 
  double left_v=MAX_V,right_v=MAX_V; //initializing left and right motor velocities
  int start=0; // variable to find initial position of robot
  int st=0; //variable to check if robot has reached the start point again
  double initial_x,initial_y,initial_z; //initial position of robot
  double final_x,final_y,final_z; //destination position of robot
  bool toggle=false; //led on/off variable
  while (wb_robot_step(TIME_STEP) != -1)
  {
    if (start==0){
    //Finds initial position of robot
    wb_motor_set_velocity(left_motor,left_v); 
    wb_motor_set_velocity(right_motor,right_v);
    const double *initial_pos=wb_gps_get_values(gps); //getting initial position values
    //setting initial position values
    initial_x=initial_pos[0]; 
    initial_y=initial_pos[1];
    initial_z=initial_pos[2];
    start=1;
    printf("Start>> %.2f %.2f %.2f\n",initial_pos[0],initial_pos[1],initial_pos[2]);
    }
    else if (start==1){
    //Wall Following Algorithm Starts
    double dsf=wb_distance_sensor_get_value(distance_sensors[0]); //front-right distance sensor
    double dsr=wb_distance_sensor_get_value(distance_sensors[2]); //right distance sensor
    double dsc=wb_distance_sensor_get_value(distance_sensors[1]); //front-right-corner distance sensor
    double dsl=wb_distance_sensor_get_value(distance_sensors[5]); //left distance sensor
    double sum=0; //initialiazing total light intensity at
    const double *gps_value=wb_gps_get_values(gps); //Getting position of robot
    //variables to check if walls are close to certain sensors
    int front=(dsf>100);
    int right=(dsr>100);
    int corner=(dsc>100);
    int left=(dsl>30);
    if (corner==1)
    //robot turns left if corner sensor is too close to the wall
    {
      left_v=MAX_V/10;
      right_v=MAX_V;
    }
    else if (right==0)
    //robot turns right if there is no right wall
    {
      left_v=MAX_V;
      right_v=MAX_V/10;
    }
    else if (right==1 && front==0)
    //robot goes straight if there is no front wall but there is a right wall
    {
      left_v=MAX_V;
      right_v=MAX_V;
    }
    else if (right==1 && front==1)
    //robot turns left if there is a front wall and a right wall
    {
      left_v=-MAX_V;
      right_v=MAX_V;
    }
    if (front==1 && right==1 && left==1 && st==0){
    // if robot is at dend end and scanning of map is not complete it checks the average light intensity
    // to find max light intensity
      for (int i = 0; i < 9; i++)
      {
        double light_value = wb_light_sensor_get_value(light_sensors[i]);
        sum+=light_value;
      }
      avg_light_intensity=sum/9; //finding average light intensity
      if (avg_light_intensity > max_light_intensity) //checking if average light intensity is greater than max light intensity
      {
        //if average light intensity is greater than the max light instensity then max light intensity will be updated
        max_light_intensity = avg_light_intensity;
        printf("New Max Light Intensity: %f\n", max_light_intensity);
        //Destination values are set at the point with max light intensity
        final_x=gps_value[0];
        final_y=gps_value[1];
        final_z=gps_value[2];
      }
    } 
    if (st==1){ //runs when scan is complete
      double df=sqrt(pow(gps_value[0]-final_x,2)+pow(gps_value[1]-final_y,2)+pow(gps_value[2]-final_z,2)); //finds distance of robot from final destination
      if (df<0.1){
      //if distance of robot from final destination is less than 0.1, then robot stops
        left_v=0;
        right_v=0;
        for (int i = 0; i < 8; i++) {
          wb_led_set(led[i], toggle); //led will start blinking
        }
        toggle=!toggle; //to change led value from on/off
      }
    }
    else{
      double d=sqrt(pow(gps_value[0]-initial_x,2)+pow(gps_value[1]-initial_y,2)+pow(gps_value[2]-initial_z,2)); //finds distance of robot from the starting point
      if (d<0.1 && front==1) //if distance is less than 0.1 and there is a front wall, the scanning is complete
        st=1;
    }
    wb_motor_set_velocity(left_motor,left_v); //setting velocity of left motor
    wb_motor_set_velocity(right_motor,right_v); //setting velocity of right motor
    printf("GPS>> %.2f %.2f %.2f \n",gps_value[0],gps_value[1],gps_value[2]); //prints the gps values at the current position
  }
  }
 wb_robot_cleanup();

 

 return 0;
}
