package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
/*
import frc.robot.mecanumcontrollercommand.Constants.AutoConstants;
import frc.robot.mecanumcontrollercommand.Constants.DriveConstants;
import frc.robot.mecanumcontrollercommand.Constants.OIConstants;
import frc.robot.mecanumcontrollercommand.subsystems.DriveSubsystem;
*/
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import java.util.List;
import java.util.Timer;
import java.util.function.Supplier;

public class RobotContainer {
    private final DriveTrain m_robotDrive = new DriveTrain();
    XboxController m_driverController = new XboxController(OIConstants.kDriverJoystickPort);

    public RobotContainer() {
        // Set up the buttons and tell the robot what they need to do
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -m_driverController.getLeftY(),
                                -m_driverController.getRightX(),
                                -m_driverController.getLeftX(),
                                false)));

        /*
         * Configure joysticks example
         * 1) The subsystem added
         * 2) Mapped to our joystick class
         * 3) Refined to match our structure
         */
        /*
         * driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, //
         * () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
         * () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis))//
         * );
         */
    }

    private void configureButtonBindings() {
        // Add the controller button stuff here, as an example:
        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
        // Note this should map to Xbox/logi/ps4/5 controllers instead
    }

    public static final Pose2d kZeroPose2d = new Pose2d();
    public static final Rotation2d kZeroRotation2d = new Rotation2d();

    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(2.2, 2.2)
                // Add kinematics to ensure max speed is actually obeyed
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
        var kPXController = new PIDController(AutoConstants.kPXController, 0, 0);
        var kPYController = new PIDController(AutoConstants.kPYController, 0, 0);
        var kPThetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                AutoConstants.kThetaControllerConstraints);

        // Velocity PID's
        var kFrontLeftVel = new PIDController(DriveConstants.kFrontLeftVel, 0, 0);
        var kRearLeftVel = new PIDController(DriveConstants.kRearLeftVel, 0, 0);
        var kPFrontRightVel = new PIDController(DriveConstants.kPFrontRightVel, 0, 0);
        var kPRearRightVel = new PIDController(DriveConstants.kPRearRightVel, 0, 0);

        MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                DriveConstants.kFeedForward,
                DriveConstants.kDriveKinematics,

                kPXController,
                kPYController,
                kPThetaController,
                // Needed for normalizing wheel speeds
                AutoConstants.kMaxSpeedMetersPerSecond,

                kFrontLeftVel,
                kRearLeftVel,
                kPFrontRightVel,
                kPRearRightVel,
                m_robotDrive::getCurrentWheelSpeeds,
                m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                m_robotDrive);

        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.sequence(
                new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                mecanumControllerCommand,
                new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
    }

}
