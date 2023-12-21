// File:          DriveController.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
 public class DriveController {

  public final static double WHEEL_RADIUS = 0.0975;  // in meters
  public final static double AXEL_LENGTH = 0.31;     // in meters

  static Motor left_motor;
  static Motor right_motor;
  
  public enum MoveState { FORWARD, ROTATE };

  static MoveState state;
  
  public static double forward(double target_dist, double linear_velocity) {
    double av = (linear_velocity/WHEEL_RADIUS);
  
    left_motor.setVelocity(av);
    right_motor.setVelocity(av);
    state = MoveState.FORWARD;
    
    return 1000.0*(target_dist/linear_velocity);    
  }
  
  public static double rotate(double rads, double linear_velocity) {
    double circle_fraction = rads / (2.0 * Math.PI);
    double circumference = Math.PI * AXEL_LENGTH;
    double target_dist = circumference * circle_fraction;
    double av = linear_velocity/WHEEL_RADIUS;

    left_motor.setVelocity(av);
    right_motor.setVelocity(-av);
    state = MoveState.ROTATE;

    return 1000.0*(target_dist/linear_velocity);
  }
  
  public static void stop() {
    left_motor.setVelocity(0.0);
    right_motor.setVelocity(0.0);
  }

  // ----------------------------------------
  public static void main(String[] args) {

    double av = 2 * Math.PI;      // angular velocity
    int time_elapsed = 0;         // Monitor the time (in millisecs)
    double linear_velocity = 1.0; // Linear velocity in m/s 
    double target_time;

    Robot robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());

    // Set up motor devices
    left_motor = robot.getMotor("left wheel");
    right_motor = robot.getMotor("right wheel");
    
    // Set the target positions of the motors
    left_motor.setPosition(Double.POSITIVE_INFINITY);
    right_motor.setPosition(Double.POSITIVE_INFINITY);

    // Start by moving forward
    target_time = forward(2.0, linear_velocity);

    // Main loop:
    while (robot.step(timeStep) != -1) {
    
      if (time_elapsed > target_time) {
        stop();
        time_elapsed = 0;
        switch (state) {
          case FORWARD:
            target_time = rotate(Math.PI/2.0, linear_velocity);
            break;
          case ROTATE:
            target_time = forward(2.0, linear_velocity);
            break;
        }
      } else
        time_elapsed+=timeStep;      // Increment by the time state
    }
  }
}
