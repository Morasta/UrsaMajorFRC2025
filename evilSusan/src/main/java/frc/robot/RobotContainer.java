package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.commands.IntakeSetOpenCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.commands.drive.DriveSidewaysCmd;
import frc.robot.commands.drive.MecanumDriveCmd;
import frc.robot.commands.ElevatorSlideCmd;
import frc.robot.commands.ElevatorVerticalCmd;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.kWheels;
import frc.robot.Constants.OIConstants;
import frc.robot.utils.GamepadAxisButton;


public class RobotContainer {
    // Drive Trains and Controllers
    private final DriveTrain m_robotDrive = new DriveTrain();
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverJoystickPort);
    CommandXboxController m_clawController = new CommandXboxController(OIConstants.kClawJoystickPort);

    // Robot Subsystems: create one instance of each
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    // Orientation Vars
    public static final Pose2d kZeroPose2d = new Pose2d();
    public static final Rotation2d kZeroRotation2d = new Rotation2d();
    private final GamepadAxisButton lClawUp = new GamepadAxisButton(this::axis1ThresholdGreaterThanPoints5);
    private final GamepadAxisButton lCrabwalk = new GamepadAxisButton(this::axis2Threshold);
    private final GamepadAxisButton rCrabwalk = new GamepadAxisButton(this::axis2Threshold);
    private final GamepadAxisButton rElevator = new GamepadAxisButton(this::axis3ThresholdGreaterThanPoints5);


    public RobotContainer() {
        configureWheels();
        //TODO: figure out what speed is best
        m_robotDrive.setMaxOutput(0.3);
        configureButtonBindings();

        // elevatorSubsystem.setDefaultCommand(new
        // ElevatorJoystickCmd(elevatorSubsystem, 0));
        // intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));

        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> m_robotDrive.drive(
                -m_driverController.getRawAxis(1),
                -m_driverController.getRawAxis(5),
                -m_driverController.getRawAxis(3),
                true), m_robotDrive
            )
        );

        /*
         * m_robotDrive.setDefaultCommand(
         * new RunCommand(() -> m_robotDrive.drive(
            * -m_driverController.getLeftY(),
            * -m_driverController.getRightX(),
            * -m_driverController.getLeftX(),
            * false), m_robotDrive)
         * );
         */
    }

    private void configureButtonBindings() {
        // Example of a command issued when both x and y are pressed at same time
        /*
         * new JoystickButton(m_driverController, XBoxController.Button.kX.value)
         * .and(new JoystickButton(m_driverController, XboxController.Button.kY.value))
         * .whenActive(new ExampleCommand());
         */

        // Testing: Trigger button controller
        // Trigger xButton = xc.x();
        // xc.x().onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)));

        System.out.println("Configuring Button Bindings");

        /*m_driverController.button(1).whileTrue(Commands.startEnd(
            () -> new PrintCommand("A button START"),
            () -> new PrintCommand("A button END"))
           // m_robotDrive)
            );
        */

        //m_driverController.a().whileTrue(new DriveForwardCmd(m_robotDrive, 5));

        //TODO: change to fixed position
        m_clawController.a().whileTrue(new ElevatorVerticalCmd(elevatorSubsystem, 0.5));
        rElevator.whileTrue(new ElevatorVerticalCmd(elevatorSubsystem, 0.5));
        m_clawController.leftTrigger().whileTrue(new IntakeSetOpenCmd(intakeSubsystem, true));
        m_clawController.rightTrigger().whileTrue(new IntakeSetOpenCmd(intakeSubsystem, true));
        //driveTrain Controls
        m_driverController.a().whileTrue(new DriveRoundTurnCmd(m_robotDrive, 0.5));
        lClawUp.whileTrue(new ElevatorSlideCmd(elevatorSubsystem, 0.5));
        lCrabwalk.whileTrue(new DriveSidewaysCmd(m_robotDrive, 0.5));
        rCrabwalk.whileTrue(new DriveSidewaysCmd(m_robotDrive, 0.5));

        //m_driverController.x().whileTrue(new PrintCommand("Getting X button"));
        //m_driverController.x().whileTrue(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true)));
        //m_driverController.x().onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true)));

        
    }

    private void configureWheels() {
        m_robotDrive.setInverted(kWheels.frontLeft);
        m_robotDrive.setInverted(kWheels.rearLeft);
    }

    public Command getAutonomousCommand() {
        new IntakeSetOpenCmd(intakeSubsystem, false);
        // Create config for trajectory
        // Add kinematics to ensure max speed is actually obeyed
        TrajectoryConfig config = new TrajectoryConfig(2.2, 2.2)
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            RobotContainer.kZeroPose2d,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, RobotContainer.kZeroRotation2d),
            config);

        // Position controllers
        PIDController kPXController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController kPYController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController kPThetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0,
            AutoConstants.kThetaControllerConstraints
        );

        // Velocity PID's
        // PIDController kFrontLeftVel = new PIDController(DriveConstants.kFrontLeftVel, 0, 0);
        // PIDController kRearLeftVel = new PIDController(DriveConstants.kRearLeftVel, 0, 0);
        // PIDController kPFrontRightVel = new PIDController(DriveConstants.kPFrontRightVel, 0, 0);
        // PIDController kPRearRightVel = new PIDController(DriveConstants.kPRearRightVel, 0, 0);

        MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
            exampleTrajectory, m_robotDrive::getPose2d, DriveConstants.kDriveKinematics, kPXController,
            kPYController, kPThetaController, AutoConstants.kMaxSpeedMetersPerSecond,
            m_robotDrive::setOutputWheelSpeeds, m_robotDrive
        );

        // Reset odometry to the initial pose of the trajectory, run path following command, then stop at the end.
        return Commands.sequence(
            new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
            mecanumControllerCommand,
            new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false))
        );
    }
    public boolean axis1ThresholdGreaterThanPoints5(){

        return Math.abs(m_clawController.getRawAxis(1)) > 0.5;
    }
    public boolean axis3ThresholdGreaterThanPoints5(){
        return Math.abs(m_clawController.getRawAxis(3)) > 0.5;
    }
    public boolean axis2Threshold(){
        return Math.abs(m_driverController.getRawAxis(2)) > 0.5;
    }
    public boolean axis3Threshold(){
        return Math.abs(m_driverController.getRawAxis(3)) > 0.5;
    }
}