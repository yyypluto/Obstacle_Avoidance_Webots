import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Display;

public class MyBug2Controller {

  public static void main(String[] args) {

    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    // define the start position
    double robot_start_x = -4.5;
    double robot_start_y = 3.0;
    
    Pose robot_pose = new Pose(robot_start_x, robot_start_y, 0.0);
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
    double distanceToQH;
    double distanceToQL;
    double shortestDistance;
    final double mLineDistance = Math.sqrt(Math.pow(robot_final_x - robot_pose.getX(), 2) + Math.pow(robot_final_y - robot_pose.getY(), 2));
    double distanceToStart;
    
    // define the start point
    double[] qL = new double[2];
    qL[0] = robot_pose.getX();
    qL[1] = robot_pose.getY();
    double[] qH = new double[2];
    qH[0] = Double.MAX_VALUE;
    qH[1] = Double.MAX_VALUE;
    long start_system_time = System.currentTimeMillis();
    long current_system_time;
    
    // init shortest distance
    shortestDistance = Math.sqrt(Math.pow(robot_final_x - qL[0], 2) + Math.pow(robot_final_y - qL[1], 2));
 
    // define schedule
    // PioneerNav2.MoveState[] schedule = { PioneerNav2.MoveState.GO_TO_TARGET, PioneerNav2.MoveState.FOLLOW_OBSTACLE };
    // int schedule_index = 0;
    PioneerNav2.MoveState state; // current state
    
    // set up odometry display
    Display odometry_display = robot.getDisplay("odometryDisplay");
    String display_action = "";
    
    // define overall setpoint
    double set_point = 0.1;
    
    // define if qH is re-encountered
    boolean is_qH_re_encountered = false;

    while (robot.step(timeStep) != -1) {
      // Testing out the front proximity sensors

      robot_pose = nav.get_real_pose();
      prox_sensors.set_pose(robot_pose);
      prox_sensors.paint();  // Render sensor Display
      
      // get the target theta
      targetTheta = Math.atan2((robot_final_y - robot_pose.getY()), (robot_final_x - robot_pose.getX()));
      deltaTheta = robot_pose.getDeltaTheta(targetTheta);
      
      state = nav.getState();
      // System.out.println(state);
      // if reach the target, then STOP;
      distanceToTarget = Math.sqrt(Math.pow(robot_final_x - robot_pose.getX(), 2) + Math.pow(robot_final_y - robot_pose.getY(), 2));
      if (distanceToTarget < set_point) state = PioneerNav2.MoveState.REACH_TARGET;
      
      distanceToStart = Math.sqrt(Math.pow(robot_start_x - robot_pose.getX(), 2) + Math.pow(robot_start_y - robot_pose.getY(), 2));

      if (time_elapsed > target_time) {
        time_elapsed = 0;
        
        // select the state
        if (state == PioneerNav2.MoveState.GO_TO_TARGET) {
          if (nav.is_encountered_obstacle(set_point)) {
            qH[0] = robot_pose.getX();
            qH[1] = robot_pose.getY();
            start_system_time = System.currentTimeMillis();
            state = PioneerNav2.MoveState.FOLLOW_OBSTACLE;
          } else {
            
          }
        } else
        if (state == PioneerNav2.MoveState.FOLLOW_OBSTACLE) {
          if (is_qH_re_encountered) {
            distanceToQL = Math.sqrt(Math.pow(qL[0] - robot_pose.getX(), 2) + Math.pow(qL[1] - robot_pose.getY(), 2));
            System.out.println(state + ": Re-encountered, distanceToQL = " + distanceToQL);
            if (distanceToQL < 0.1) {
              is_qH_re_encountered = false;
              state = PioneerNav2.MoveState.GO_TO_TARGET;
            }
          } else {
            distanceToQH = Math.sqrt(Math.pow(qH[0] - robot_pose.getX(), 2) + Math.pow(qH[1] - robot_pose.getY(), 2));
            System.out.println(state + ": Not re-encountered, distanceToQH = " + distanceToQH);
            current_system_time = System.currentTimeMillis();
            if (current_system_time - start_system_time > 20 * timeStep) {
              if (distanceToQH < 0.1) {
                is_qH_re_encountered = true;
              } else {
                if (shortestDistance >= distanceToTarget) {
                  shortestDistance = distanceToTarget;
                  qL[0] = robot_pose.getX();
                  qL[1] = robot_pose.getY();
                }
              }
              
              if (Math.abs(distanceToTarget + distanceToStart - mLineDistance) < 0.5) {
                shortestDistance = distanceToTarget;
                qL[0] = robot_pose.getX();
                qL[1] = robot_pose.getY();
                is_qH_re_encountered = false;
                state = PioneerNav2.MoveState.GO_TO_TARGET;
              }
            }
          }
        }
        // robot move
        if (state == PioneerNav2.MoveState.FOLLOW_OBSTACLE) {
          target_time = 0; // always refresh!
          nav.follow_obstacle(robot_velocity, set_point, false, deltaTheta);
          display_action = "Follow the obstacle";
        } else
        if (state == PioneerNav2.MoveState.GO_TO_TARGET) {
          target_time = 0; // always refresh!
          nav.go_to_target(robot_velocity, 0.2, false, deltaTheta);
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
      
    };
    // Enter here exit cleanup code.
  }
}
