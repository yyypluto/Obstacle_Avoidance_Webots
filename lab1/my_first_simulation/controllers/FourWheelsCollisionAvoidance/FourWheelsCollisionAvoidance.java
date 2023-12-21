import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;

public class FourWheelsCollisionAvoidance {
  public static void main(String[] args) {
    int TIME_STEP = 64;
    Robot robot = new Robot();
    DistanceSensor[] ds = new DistanceSensor[2];
    String[] dsNames = {"ds_left", "ds_right"};
    for (int i = 0; i < 2; i++) {
      ds[i] = robot.getDistanceSensor(dsNames[i]);
      ds[i].enable(TIME_STEP);
    }
    Motor[] wheels = new Motor[4];
    String[] wheelsNames = {"wheel1", "wheel2", "wheel3", "wheel4"};
    for (int i = 0; i < 4; i++) {
      wheels[i] = robot.getMotor(wheelsNames[i]);
      wheels[i].setPosition(Double.POSITIVE_INFINITY);
      wheels[i].setVelocity(0.0);
    }
    int avoidObstacleCounter = 0;
    while (robot.step(TIME_STEP) != -1) {
      double leftSpeed = 1.0;
      double rightSpeed = 1.0;
      if (avoidObstacleCounter > 0) {
        avoidObstacleCounter--;
        leftSpeed = 1.0;
        rightSpeed = -1.0;
      }
      else {
        for (int i = 0; i < 2; i++) {
          if (ds[i].getValue() < 950.0) {
            avoidObstacleCounter = 100;
          }
        } 
      }
      wheels[0].setVelocity(leftSpeed);
      wheels[1].setVelocity(rightSpeed);
      wheels[2].setVelocity(leftSpeed);
      wheels[3].setVelocity(rightSpeed);
    }
  }

}