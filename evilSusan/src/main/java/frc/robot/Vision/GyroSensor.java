package frc.robot.Vision;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;



public class GyroSensor {
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    private final SparkMax m_frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushed);
    private final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushed);
    private final SparkMax m_rearLeft = new SparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushed);
    private final SparkMax m_rearRight = new SparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushed);
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

    // Use gyro declaration from above here
    // The gain for a simple P loop
    double kP = 1;
    // The heading of the robot when starting the motion
    double heading;

    public void getGyroAngle() {
        // Set setpoint to current heading at start of auto
        heading = gyro.getAngle();
    }

    public void driveStraightWithGyro() {
        double error = heading - gyro.getAngle();
        // Drives forward continuously at half speed, using the gyro to stabilize the heading
       // drive.tankDrive(.5 + kP * error, .5 - kP * error);
    }

    public void driveTurnsWithGyro() {
        // Find the heading error; setpoint is 90
        double error = 90 - gyro.getAngle();
        // Turns the robot to face the desired direction
       // drive.tankDrive(kP * error, -kP * error);
    }


}
