// MyLab5Controller.java
/*
 * MyLab5Controller Class Definition
 * File: MyLab5Controller.java
 * Date: 15th Oct 2022
 * Description: Simple Controller based on the Lab4 controller (2022)
 * Author: Terry Payne (trp@liv.ac.uk)
 */

import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Camera;

public class MyLab5Controller {

  public static void main(String[] args) {

    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    Pose robot_pose = new Pose(0.0, 0.0, 0.0);
    PioneerProxSensors1 prox_sensors = new PioneerProxSensors1(robot, "sensor_display", robot_pose);
    PioneerNav2 nav = new PioneerNav2(robot, robot_pose, prox_sensors);
    
    Camera camera = robot.getCamera("camera");
    if (camera != null)
      camera.enable(timeStep);

    double time_elapsed = 0;
    double target_time = 0;
    double robot_velocity = 0.3;
 
    // define schedule
    PioneerNav2.MoveState[] schedule = { PioneerNav2.MoveState.FOLLOW_WALL };
    int schedule_index = -1; // we increment before selecting the current action
    PioneerNav2.MoveState state; // current state

    while (robot.step(timeStep) != -1) {
      
      // Testing out the front proximity sensors
      System.out.println(String.format("Sensor readings for so3 | so4: %.3f | %.3f",
        prox_sensors.get_value(3), prox_sensors.get_value(4)));

      robot_pose = nav.get_real_pose();
      prox_sensors.set_pose(robot_pose);
      prox_sensors.paint();  // Render sensor Display
               
      state = nav.getState();
      System.out.println(state);
      if (time_elapsed > target_time) {
        time_elapsed = 0;
            
        // select next action in schedule if not stopped
        schedule_index = (schedule_index + 1) % schedule.length;
        state = schedule[schedule_index];
        
        if (state == PioneerNav2.MoveState.FORWARD) {
          target_time = nav.forward(0.8, robot_velocity);
        } else 
        if (state == PioneerNav2.MoveState.ARC) {
          target_time = nav.arc(Math.PI/2.0, 0.0, robot_velocity);
        } else
        if (state == PioneerNav2.MoveState.FOLLOW_WALL) {
          target_time = 0; // always refresh!
          nav.follow_wall(robot_velocity, 0.2, false);
        } else
        if (state == PioneerNav2.MoveState.STOP) {
          nav.stop();
          target_time = 60 * 1000; // This doesn't really stop, but pauses for 1 minute
        }
      } else
        time_elapsed += timeStep;    // Increment by the time state
      
    };
    // Enter here exit cleanup code.
  }
}
