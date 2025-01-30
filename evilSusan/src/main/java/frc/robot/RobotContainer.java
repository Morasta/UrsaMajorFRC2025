package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.DriveForwardCmd;


public class RobotContainer {
    public RobotContainer() {
        // Set up the buttons and tell the robot what they need to do
        configureButtonBindings();

        /*
         * Configure joysticks example
         * 1) The subsystem added
         * 2) Mapped to our joystick class
         * 3) Refined to match our structure
         */
        /*driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, //
                () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
                () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis))//
        );*/
    }

    private void configureButtonBindings() {
        // Add the controller button stuff here, as an example: 
        // new JoystickButton(joystick1, OIConstants.kElevatorPIDRaiseButtonIdx)
        //        .whileActiveOnce(new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kRaisedPosition));
        // Note this should map to Xbox/logi/ps4/5 controllers instead
    }

    public Command getAutonomousCommand() {
        System.out.println("Autonomous Command Set!");
        /*return new SequentialCommandGroup(
                new DriveForwardCmd(driveSubsystem, DriveConstants.kAutoDriveForwardDistance),
                new ParallelCommandGroup(
                        new IntakeSetCmd(intakeSubsystem, false),
                        new ElevatorPIDCmd(elevatorSubsystem, ElevatorConstants.kRaisedPosition)
                )
        );*/
       // return new PrintCommand("Executed autotonomous command!");
       return new DriveForwardCmd(null, 0);
=======
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeSetCmd;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ElevatorJoystickCmd;
import frc.robot.commands.MecanumDriveCmd;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
        private final DriveTrain m_robotDrive = new DriveTrain();
        XboxController m_driverController = new XboxController(OIConstants.kDriverJoystickPort);

private final DriveTrain driveSubsystem = new DriveTrain();
        // Robot Subsystems: create one instance of each
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final Joystick joystick1 = new Joystick(OIConstants.kDriverJoystickPort);
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

        public RobotContainer() {
                configureButtonBindings();

                driveSubsystem.setDefaultCommand(new MecanumDriveCmd(driveSubsystem, //
                        () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
                        () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis))//
                );
                elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, 0));
                intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));

                // Set up the buttons and tell the robot what they need to do
                configureButtonBindings();
                
        m_robotDrive.setDefaultCommand(new MecanumDriveCmd(m_robotDrive, 
                () -> m_driverController.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
                () -> m_driverController.getRawAxis(OIConstants.kArcadeDriveTurnAxis)));

                /*  m_robotDrive.setDefaultCommand(new MecanumDriveCmd(m_robotDrive, 
                        () -> m_driverController.getRawAxis(OIConstants.leftStrafe),
                        () -> m_driverController.getRawAxis(OIConstants.rightStrafe))
                );
                */

        /*   m_robotDrive.setDefaultCommand(
                        new RunCommand(
                                () -> m_robotDrive.drive(
                                        -m_driverController.getLeftY(),
                                        -m_driverController.getRightX(),
                                        -m_driverController.getLeftX(),
                                        false), m_robotDrive));
        */                      

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
                        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)))
                        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0)));
                        new JoystickButton(joystick1, OIConstants.kIntakeCloseButtonIdx)
                        .whileTrue(new IntakeSetCmd(intakeSubsystem, false));
                // Note this should map to Xbox/logi/ps4/5 controllers instead
        }

        public static final Pose2d kZeroPose2d = new Pose2d();
        public static final Rotation2d kZeroRotation2d = new Rotation2d();

    //
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
                exampleTrajectory,
                m_robotDrive::getPose2d,
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
>>>>>>> Stashed changes
    }
}
