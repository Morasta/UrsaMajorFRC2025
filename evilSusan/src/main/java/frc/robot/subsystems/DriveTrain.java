package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import edu.wpi.first.util.sendable.SendableRegistry;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.kWheels;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class DriveTrain extends SubsystemBase {
    // SparkMaxConfig
    private final SparkMaxConfig sparkInvertedConfig = new SparkMaxConfig();
    private final SparkMax m_frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushed);
    private final SparkMax m_frontRight = new SparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushed);
    private final SparkMax m_rearLeft = new SparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushed);
    private final SparkMax m_rearRight = new SparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushed);

    HashMap<kWheels, SparkMax> m_wheels = new HashMap<kWheels, SparkMax>();

    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final Encoder m_frontLeftEncoder = new Encoder(
        DriveConstants.kFrontLeftEncoderPorts[0],
        DriveConstants.kFrontLeftEncoderPorts[1],
        DriveConstants.kFrontLeftEncoderReversed
    );

    private final Encoder m_rearLeftEncoder = new Encoder(
        DriveConstants.kRearLeftEncoderPorts[0],
        DriveConstants.kRearLeftEncoderPorts[1],
        DriveConstants.kRearLeftEncoderReversed
    );

    private final Encoder m_frontRightEncoder = new Encoder(
        DriveConstants.kFrontRightEncoderPorts[0],
        DriveConstants.kFrontRightEncoderPorts[1],
        DriveConstants.kFrontRightEncoderReversed
    );

    private final Encoder m_rearRightEncoder = new Encoder(
        DriveConstants.kRearRightEncoderPorts[0],
        DriveConstants.kRearRightEncoderPorts[1],
        DriveConstants.kRearRightEncoderReversed
    );

    public DriveTrain() {
        SendableRegistry.addChild(m_robotDrive, m_frontLeft);
        SendableRegistry.addChild(m_robotDrive, m_frontRight);
        SendableRegistry.addChild(m_robotDrive, m_rearLeft);
        SendableRegistry.addChild(m_robotDrive, m_rearRight);

        // Populate HashMap with robot's wheels
        m_wheels.put(kWheels.frontLeft, m_frontLeft);
        m_wheels.put(kWheels.frontRight, m_frontRight);
        m_wheels.put(kWheels.rearLeft, m_rearLeft);
        m_wheels.put(kWheels.rearRight, m_rearRight);

        //Inverted
        sparkInvertedConfig.inverted(true);
        //m_FrontLeft.setInverted(true);
        //m_FrontRight.setInverted(true);
        //m_RearLeft.setInverted(true);
        //m_RearRight.setInverted(true);
        m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse); 
        
        //m_frontLeft.configure(sparkInvertedConfig, null, null);
        //m_rearLeft.configure(sparkInvertedConfig, null, null);

        m_robotDrive.feed();
    }
    
    public void setInverted (kWheels wheel) {
        m_wheels.get(wheel).configure(sparkInvertedConfig, null, null);
    }

    private void configureDashboard () {
        Shuffleboard.getTab("Drive")
            .add("Max Speed", 1)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .getEntry();
    }

    public void setMaxOutput(double maxOutput) {
        System.out.println("SetMaxOutput" + maxOutput);
        m_robotDrive.setMaxOutput(maxOutput);
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        m_frontLeft.set(leftSpeed);
        m_rearLeft.set(leftSpeed);
        m_frontRight.set(rightSpeed);
        m_rearRight.set(rightSpeed);
    }
  
    public void setMotors(double frontLeftSpeed, double rearLeftSpeed, double frontRightSpeed, double rearRightSpeed) {
        m_frontLeft.set(frontLeftSpeed);
        m_rearLeft.set(rearLeftSpeed);
        m_frontRight.set(frontRightSpeed);
        m_rearRight.set(rearRightSpeed);
    }

    public void setStrafeMotors(double leftSpeed, double rightSpeed) {
        m_frontLeft.set(leftSpeed);
        m_rearLeft.set(-leftSpeed);
        m_frontRight.set(-rightSpeed);
        m_rearRight.set(rightSpeed);
    }

    public Pose2d getPose2d() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/simpledifferentialdrivesimulation/Drivetrain.java#L125
        // m_odometry.resetPosition(m_gyro.getRotation2d(), getCurrentWheelDistances(), pose);
        System.out.println("resetOdometry called");
    }

    public void resetEncoders() {
        m_frontLeftEncoder.reset();
        m_rearLeftEncoder.reset();
        m_frontRightEncoder.reset();
        m_rearRightEncoder.reset();
    }

    public Encoder getFrontLeftEncoder() {
        return m_frontLeftEncoder;
    }

    public Encoder getRearLeftEncoder() {
        return m_rearLeftEncoder;
    }

    public Encoder getFrontRightEncoder() {
        return m_frontRightEncoder;
    }

    public Encoder getRearRightEncoder() {
        return m_rearRightEncoder;
    }

    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(), new MecanumDriveWheelPositions());

    Rotation2d desiredRotation = new Rotation2d();
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                m_frontLeftEncoder.getRate(),
                m_rearLeftEncoder.getRate(),
                m_frontRightEncoder.getRate(),
                m_rearRightEncoder.getRate());
    }

    public MecanumDriveWheelPositions getCurrentWheelDistances() {
        return new MecanumDriveWheelPositions(
                m_frontLeftEncoder.getDistance(),
                m_rearLeftEncoder.getDistance(),
                m_frontRightEncoder.getDistance(),
                m_rearRightEncoder.getDistance());
    }

    public MecanumDriveWheelPositions getCurrentWheelPositions() {
        return new MecanumDriveWheelPositions(
                m_frontLeftEncoder.getDistance(),
                m_rearLeftEncoder.getDistance(),
                m_frontRightEncoder.getDistance(),
                m_rearRightEncoder.getDistance());
    }

    /** Sets the wheel speeds */
    public void setOutputWheelSpeeds(MecanumDriveWheelSpeeds mdws) {
        m_frontLeft.set(mdws.frontLeftMetersPerSecond);
        m_rearRight.set(mdws.rearRightMetersPerSecond);
        m_frontRight.set(mdws.frontRightMetersPerSecond);
        m_rearLeft.set(mdws.rearLeftMetersPerSecond);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        //System.out.println("(xSpd, ySpd, rot, frInv, rrInv, flInv, frInv): " + xSpeed + " | " + ySpeed + " | " + rot + " | " + m_frontRight.configAccessor.getInverted() + " | " + m_rearRight.configAccessor.getInverted() + " | " + m_frontLeft.configAccessor.getInverted() + " | " + m_rearLeft.configAccessor.getInverted());
        /* if (fieldRelative) {
            m_robotDrive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
        }*/
    
        m_robotDrive.driveCartesian(xSpeed, ySpeed, rot);
        //this.setMotors(0.2, 0.2);
    }
}
