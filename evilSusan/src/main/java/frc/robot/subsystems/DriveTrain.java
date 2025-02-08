package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import java.io.ObjectInputFilter.Config;
import java.util.HashMap;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DriveConstants.kWheels;

public class DriveTrain extends SubsystemBase {
    //TODO: change m_ to lowercase
    // SparkMaxConfig
    private final SparkMaxConfig sparkInvertedConfig = new SparkMaxConfig();
    private final SparkMax m_FrontLeft = new SparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushed);
    private final SparkMax m_FrontRight = new SparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushed);
    private final SparkMax m_RearLeft = new SparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushed);
    private final SparkMax m_RearRight = new SparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushed);

    HashMap<kWheels, SparkMax> m_wheels = new HashMap<kWheels, SparkMax>();

    private final MecanumDrive m_robotDrive = new MecanumDrive(m_FrontLeft, m_RearLeft, m_FrontRight, m_RearRight);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final Encoder m_frontLeftEncoder = new Encoder(
            DriveConstants.kFrontLeftEncoderPorts[0],
            DriveConstants.kFrontLeftEncoderPorts[1],
            DriveConstants.kFrontLeftEncoderReversed);

    private final Encoder m_rearLeftEncoder = new Encoder(
            DriveConstants.kRearLeftEncoderPorts[0],
            DriveConstants.kRearLeftEncoderPorts[1],
            DriveConstants.kRearLeftEncoderReversed);

    private final Encoder m_frontRightEncoder = new Encoder(
            DriveConstants.kFrontRightEncoderPorts[0],
            DriveConstants.kFrontRightEncoderPorts[1],
            DriveConstants.kFrontRightEncoderReversed);

    private final Encoder m_rearRightEncoder = new Encoder(
            DriveConstants.kRearRightEncoderPorts[0],
            DriveConstants.kRearRightEncoderPorts[1],
            DriveConstants.kRearRightEncoderReversed);

    public DriveTrain() {
        SendableRegistry.addChild(m_robotDrive, m_FrontLeft);
        SendableRegistry.addChild(m_robotDrive, m_FrontRight);
        SendableRegistry.addChild(m_robotDrive, m_RearLeft);
        SendableRegistry.addChild(m_robotDrive, m_RearRight);

        m_wheels.put(kWheels.FrontLeft, m_FrontLeft);
        m_wheels.put(kWheels.FrontRight, m_FrontRight);
        m_wheels.put(kWheels.RearLeft, m_RearLeft);
        m_wheels.put(kWheels.RearRight, m_RearRight);

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
        
        m_FrontLeft.configure(sparkInvertedConfig,null, null);
        m_RearLeft.configure(sparkInvertedConfig, null, null);

        m_robotDrive.feed();
    }
    
    //public void setInverted (kWheels wheel) {
        //m_wheels.get(wheel).configure(sparkInvertedConfig, null, null);
   // }

    public void setMaxOutput(double maxOutput) {
        System.out.println("SetMaxOutput" + maxOutput);
        m_robotDrive.setMaxOutput(maxOutput);
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        m_FrontLeft.set(leftSpeed);
        m_RearLeft.set(leftSpeed);
        m_FrontRight.set(rightSpeed);
        m_RearRight.set(rightSpeed);
    }

    public void setStrafeMotors(double leftSpeed, double rightSpeed) {
        m_FrontLeft.set(leftSpeed);
        m_RearLeft.set(-leftSpeed);
        m_FrontRight.set(-rightSpeed);
        m_RearRight.set(rightSpeed);
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
        m_FrontLeft.set(mdws.frontLeftMetersPerSecond);
        m_RearRight.set(mdws.rearRightMetersPerSecond);
        m_FrontRight.set(mdws.frontRightMetersPerSecond);
        m_RearLeft.set(mdws.rearLeftMetersPerSecond);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        //System.out.println("xSpeed" + xSpeed);
        //System.out.println("ySpeed" + ySpeed);
        //System.out.println("rot" + rot);
        System.out.println("(xSpd, ySpd, rot, frInv, rrInv, flInv, frInv): " + xSpeed + " | " + ySpeed + " | " + rot + " | " + m_FrontRight.configAccessor.getInverted() + " | " + m_RearRight.configAccessor.getInverted() + " | " + m_FrontLeft.configAccessor.getInverted() + " | " + m_RearLeft.configAccessor.getInverted());
  /*       if (fieldRelative) {
            m_robotDrive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
        }*/
    
        m_robotDrive.driveCartesian(xSpeed, ySpeed, rot);
        //this.setMotors(0.2, 0.2);
    
    }
}