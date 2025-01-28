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

public class DriveTrain extends SubsystemBase {
    private final PWMSparkMax m_FrontLeft = new PWMSparkMax(DriveConstants.kFrontLeftMotorPort);
    private final PWMSparkMax m_FrontRight = new PWMSparkMax(DriveConstants.kFrontRightMotorPort);
    private final PWMSparkMax m_BackLeft = new PWMSparkMax(DriveConstants.kRearLeftMotorPort);
    private final PWMSparkMax m_BackRight = new PWMSparkMax(DriveConstants.kRearRightMotorPort);
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_FrontLeft, m_BackLeft, m_FrontRight, m_BackRight);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    public void setMaxOutput(double maxOutput) {
        m_robotDrive.setMaxOutput(maxOutput);
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        m_FrontLeft.set(leftSpeed);
        m_FrontRight.set(-rightSpeed);
    }

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
        SendableRegistry.addChild(m_robotDrive, m_BackLeft);
        SendableRegistry.addChild(m_robotDrive, m_BackRight);

        m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_frontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        m_FrontRight.setInverted(true);
        m_BackRight.setInverted(true);

        m_robotDrive.feed();
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

    /** Sets the front left drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages mdmv) {
        m_FrontLeft.setVoltage(mdmv.frontLeftVoltage);
        m_BackRight.setVoltage(mdmv.rearRightVoltage);
        m_FrontRight.setVoltage(mdmv.frontRightVoltage);
        m_BackRight.setVoltage(mdmv.rearRightVoltage);
        // m_FrontLeft.setVoltage(frontLeftVoltage);
        // m_BackLeft.setVoltage(rearLeftVoltage);
        // m_FrontRight.setVoltage(frontRightVoltage);
        // m_BackRight.setVoltage(rearRightVoltage);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            m_robotDrive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d());
        }
    }
}