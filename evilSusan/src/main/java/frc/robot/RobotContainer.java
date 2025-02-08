package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeSetCmd;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants.kWheels;
import frc.robot.Constants.DriveConstants.kWheels;
import frc.robot.commands.ElevatorJoystickCmd;
import frc.robot.commands.MecanumDriveCmd;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;

public class RobotContainer {
        private final DriveTrain m_robotDrive = new DriveTrain();
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverJoystickPort);

        // Robot Subsystems: create one instance of each
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        //private final Joystick joystick1 = new Joystick(OIConstants.kDriverJoystickPort);
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

        public RobotContainer() {
                configureWheels();
                m_robotDrive.setMaxOutput(0.1);
                configureButtonBindings();

        /*         m_robotDrive.setDefaultCommand(new MecanumDriveCmd(m_robotDrive, //
                        () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
                        () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis))//
                );
        */

         //Trigger xButton = xc.x();
        //TODO: Move xc to Xbox controller
         m_driverController.x().whileTrue(new PrintCommand("Getting X button"));
         //xc.x().onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)));
         //xc.x().onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));

         m_driverController.x().whileTrue(new InstantCommand(() -> m_robotDrive.drive(0.05, 0.5, 0, true)));
         m_driverController.x().onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true)));

        //new JoystickButton(m_driverController, Button.kLeftStick.value).whileTrue(new Command());

/*      Example of a command issued when both x and y are pressed at same time
        new JoystickButton(m_driverController, XBoxController.Button.kX.value)
        .and(new JoystickButton(m_driverController, XboxController.Button.kY.value))
        .whenActive(new ExampleCommand());
*/

                //elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, 0));
                //intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));

                m_robotDrive.setDefaultCommand(
                        new RunCommand(() -> m_robotDrive.drive(
                                -m_driverController.getRawAxis(1),
                                -m_driverController.getRawAxis(5),
                                -m_driverController.getRawAxis(3),
                        true), m_robotDrive));

        /*   m_robotDrive.setDefaultCommand(
                        new RunCommand(
                                () -> m_robotDrive.drive(
                                        -m_driverController.getLeftY(),
                                        -m_driverController.getRightX(),
                                        -m_driverController.getLeftX(),
                                        false), m_robotDrive));
        */                      
        }

         private void configureButtonBindings() {
                 //TODO: fix to not be CommandXboxController
  /*              new JoystickButton(m_driverController, Button.kRightBumper.value)
                        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)))
                        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0)));
*/
                // Note this should map to Xbox/logi/ps4/5 controllers instead
        }

    private void configureWheels() {
        //m_robotDrive.setInverted(kWheels.FrontLeft);
        //m_robotDrive.setInverted(kWheels.FrontRight);
        //m_robotDrive.setInverted(kWheels.RearLeft);
        //m_robotDrive.setInverted(kWheels.RearRight);
    }
        public static final Pose2d kZeroPose2d = new Pose2d();
        public static final Rotation2d kZeroRotation2d = new Rotation2d();

    public Command getAutonomousCommand() {
        // Create config for trajectory
        new IntakeSetCmd(intakeSubsystem, false); //
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
                config
        );

        // Position controllers
        PIDController kPXController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController kPYController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController kPThetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                AutoConstants.kThetaControllerConstraints);

        // Velocity PID's
        PIDController kFrontLeftVel = new PIDController(DriveConstants.kFrontLeftVel, 0, 0);
        PIDController kRearLeftVel = new PIDController(DriveConstants.kRearLeftVel, 0, 0);
        PIDController kPFrontRightVel = new PIDController(DriveConstants.kPFrontRightVel, 0, 0);
        PIDController kPRearRightVel = new PIDController(DriveConstants.kPRearRightVel, 0, 0);

        MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
            exampleTrajectory
            , m_robotDrive::getPose2d
            , DriveConstants.kDriveKinematics
            , kPXController
            , kPYController
            , kPThetaController
            , AutoConstants.kMaxSpeedMetersPerSecond
            , m_robotDrive::setOutputWheelSpeeds
            , m_robotDrive
        );

        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        return Commands.sequence(
                new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                mecanumControllerCommand,
                new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
    }


}
