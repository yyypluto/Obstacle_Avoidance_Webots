import com.cyberbotics.webots.controller.Robot;

// Added a new include file
import com.cyberbotics.webots.controller.Motor;

public class EPuckGoForward {

 public static void main(String[] args) {

   int TIME_STEP = 64;
   
   double MAX_SPEED = 6.28;

   Robot robot = new Robot();

   // get the motor devices
   Motor leftMotor = robot.getMotor("left wheel motor");
   Motor rightMotor = robot.getMotor("right wheel motor");
   
   // set the target position of the motors
   leftMotor.setPosition(Double.POSITIVE_INFINITY);
   rightMotor.setPosition(Double.POSITIVE_INFINITY);
   
   //set up the motor speeds at 10% of the MAX_SPEED
   leftMotor.setVelocity(0.1 * MAX_SPEED);
   rightMotor.setVelocity(0.1 * MAX_SPEED);

   while (robot.step(TIME_STEP) != -1);
 }
}