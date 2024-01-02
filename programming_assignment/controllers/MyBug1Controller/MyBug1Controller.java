import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Display;

public class MyBug1Controller {

  public static void main(String[] args) {

    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    Pose robot_pose = new Pose(-4.5, 3.0, 0.0);
    PioneerProxSensors1 prox_sensors = new PioneerProxSensors1(robot, "sensor_display", robot_pose);
    PioneerNav2 nav = new PioneerNav2(robot, robot_pose, prox_sensors);
    
    Camera camera = robot.getCamera("camera");
    if (camera != null)
      camera.enable(timeStep);

    double time_elapsed = 0;
    double target_time = 0;
    double robot_velocity = 0.3;
    
    // define the target position
    double robot_final_x = 2.75;
    double robot_final_y = -3.26;
    
    // define targetTheta and distanceToTarget
    double targetTheta;
    double distanceToTarget;
    double deltaTheta;
 
    // define schedule
    PioneerNav2.MoveState[] schedule = { PioneerNav2.MoveState.FOLLOW_OBSTACLE };
    int schedule_index = -1; // we increment before selecting the current action
    PioneerNav2.MoveState state; // current state
    
    // set up odometry display
    Display odometry_display = robot.getDisplay("odometryDisplay");
    String display_action = "";

    while (robot.step(timeStep) != -1) {
      // Testing out the front proximity sensors

      robot_pose = nav.get_real_pose();
      prox_sensors.set_pose(robot_pose);
      prox_sensors.paint();  // Render sensor Display
      
      // get the target theta
      targetTheta = Math.atan2((robot_final_y - robot_pose.getY()), (robot_final_x - robot_pose.getX()));
      deltaTheta = robot_pose.getDeltaTheta(targetTheta);
      System.out.println(deltaTheta);
      
      state = nav.getState();
      
      // if reach the target, then STOP;
      distanceToTarget = Math.sqrt(Math.pow((robot_final_y - robot_pose.getY()), 2) + Math.pow((robot_final_x - robot_pose.getX()), 2));
      if (distanceToTarget < 0.1) {
        nav.stop();
        break;
      }
      
      if (time_elapsed > target_time) {
        time_elapsed = 0;
        schedule_index = (schedule_index + 1) % schedule.length;
        state = schedule[schedule_index];
        
        if (state == PioneerNav2.MoveState.FORWARD) {
          target_time = nav.forward(0.8, robot_velocity);
          display_action = "Forward Action: 0.8m";
        } else 
        if (state == PioneerNav2.MoveState.ARC) {
          target_time = nav.arc(Math.PI/2.0, 0.0, robot_velocity);
          display_action = "Arc Action around itself";
        } else
        if (state == PioneerNav2.MoveState.FOLLOW_OBSTACLE) {
          target_time = 0; // always refresh!
          nav.follow_obstacle(robot_velocity, 0.2, false, deltaTheta);
          display_action = "Follow the Obstacle";
        } else
        if (state == PioneerNav2.MoveState.GO_TO_TARGET) {
          target_time = 0; // always refresh!
          nav.go_to_target(targetTheta, 0.2);
          display_action = "Go straight to the target";
        } else
        if (state == PioneerNav2.MoveState.STOP) {
          nav.stop();
          display_action = "Stop for 1 minute";
          target_time = 60 * 1000; // This doesn't really stop, but pauses for 1 minute
        } else
        if (state == PioneerNav2.MoveState.REACH_TARGET) {
          target_time = 0;
          nav.reach_target();
          display_action = "Target reached!";
        }
      } else
        time_elapsed += timeStep;    // Increment by the time state

      // if (odometry_display != null) {
        // odometry_display.setColor(0xFFFFFF);          // White
        // odometry_display.fillRectangle(0,0,
                // odometry_display.getWidth(),
                // odometry_display.getHeight());

        // odometry_display.setColor(0x000000);          // Black
        // odometry_display.setFont("Arial", 18, true);   // font size = 18, with antialiasing
        // odometry_display.drawText("Robot State", 1, 1);

        // odometry_display.setFont("Arial", 12, true);
        // if (display_action != "")
          // odometry_display.drawText(display_action, 1, 30);

        // Pose true_pose = nav.get_real_pose();
        // odometry_display.drawText("True Pose: " + true_pose, 1, 50);
      // }
      
      if (state == PioneerNav2.MoveState.REACH_TARGET) {
        break;  // This will break the main loop
      }
      
    };
    // Enter here exit cleanup code.
  }
}
