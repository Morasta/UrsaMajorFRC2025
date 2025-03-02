package frc.robot;

import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.lib.LimelightHelpers;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.intake.IntakeDropCoralCmd;
import frc.robot.commands.intake.IntakeSetOpenCmd;
import frc.robot.commands.auto.FindAprilTagCmd;
import frc.robot.commands.auto.RotateTillTagFoundCmd;
import frc.robot.commands.drive.DriveForwardCmd;
import frc.robot.commands.drive.DriveLeftDiagonalCmd;
import frc.robot.commands.drive.DriveLeftSidewaysCmd;
import frc.robot.commands.drive.DriveBackwardCmd;
import frc.robot.commands.drive.DriveRightDiagonalCmd;
import frc.robot.commands.drive.WideRightTurnCmd;
import frc.robot.commands.drive.WideLeftTurnCmd;
import frc.robot.commands.drive.DriveRearTurnCmd;
import frc.robot.commands.drive.DriveRoundTurnCmd;
import frc.robot.commands.drive.DriveRightSidewaysCmd;
//import frc.robot.commands.drive.MecanumDriveCmd;
import frc.robot.commands.drive.StopCmd;
import frc.robot.commands.elevator.ElevatorSlideCmd;
import frc.robot.commands.elevator.ElevatorSlideExtendedCommand;
import frc.robot.commands.elevator.ElevatorSlideRetractedCommand;
import frc.robot.commands.elevator.ElevatorVerticalCmd;
import frc.robot.commands.elevator.ElevatorVerticalSetBottomCmd;
import frc.robot.commands.elevator.ElevatorVerticalSetTopCmd;
import frc.robot.commands.groups.DepositCoralCmdGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.AprilTagDists;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.kWheels;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LimelightVisionConstants.LimelightCamera;
import frc.robot.Constants.OIConstants;
import frc.robot.utils.GamepadAxisButton;
import frc.robot.utils.RobotCameraPose;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;


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
    private final GamepadAxisButton lClawUp = new GamepadAxisButton(() -> axisOverThreshold(m_clawController, 1, 0.5, false));
    private final GamepadAxisButton lClawDown = new GamepadAxisButton(() -> axisOverThreshold(m_clawController, 1, -0.5, true));
    private final GamepadAxisButton lCrabwalk = new GamepadAxisButton(() -> axisOverThreshold(m_driverController, 2, 0.5, false));
    private final GamepadAxisButton rCrabwalk = new GamepadAxisButton(() -> axisOverThreshold(m_driverController, 3, 0.5, false));
    //private final GamepadAxisButton rElevator = new GamepadAxisButton(() -> axisOverThreshold(m_clawController, 3, 0.5, false));
    
    private static final boolean toggleDefaultAutoButtons = false;
    
    public static final LimelightVisionSubsystem limelightVisionSubsystem = new LimelightVisionSubsystem();
    
    public RobotContainer() {
        configureWheels();
        //TODO: figure out what speed is best
        m_robotDrive.setMaxOutput(0.3);
        if(toggleDefaultAutoButtons == true) {
            configureButtonsForAutoTesting();
        } else {
            configureButtonBindings();
        }
        
        // elevatorSubsystem.setDefaultCommand(new
        // ElevatorJoystickCmd(elevatorSubsystem, 0));
        // intakeSubsystem.setDefaultCommand(new IntakeSetCmd(intakeSubsystem, true));
        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> m_robotDrive.drive(
                -m_driverController.getRawAxis(1),
                -m_driverController.getRawAxis(5),
                -m_driverController.getRawAxis(4),
                true), m_robotDrive
            )
        );
    }
    
    private void configureButtonBindings() {
        // Example of a command issued when both x and y are pressed at same time
        /*
        * new JoystickButton(m_driverController, XBoxController.Button.kX.value)
        * .and(new JoystickButton(m_driverController, XboxController.Button.kY.value))
        * .whenActive(new ExampleCommand());
        */
        System.out.println("Configuring Button Bindings");
        //TODO: change to fixed position

        m_clawController.a().whileTrue(new ElevatorSlideCmd(elevatorSubsystem, 0.5));
        m_clawController.b().whileTrue(new ElevatorSlideCmd(elevatorSubsystem, -0.5));

        m_clawController.leftBumper().whileTrue(new IntakeDropCoralCmd(intakeSubsystem, true));
        m_clawController.rightBumper().whileTrue(new IntakeDropCoralCmd(intakeSubsystem, false));
        //rElevator.whileTrue(new ElevatorVerticalCmd(elevatorSubsystem, 0.5));
        m_clawController.leftTrigger().whileTrue(new IntakeSetOpenCmd(intakeSubsystem, true));
        m_clawController.rightTrigger().whileTrue(new IntakeSetOpenCmd(intakeSubsystem, false));

        lClawUp.whileTrue(new ElevatorVerticalCmd(elevatorSubsystem, 0.5));
        lClawDown.whileTrue(new ElevatorSlideCmd(elevatorSubsystem, -0.5));
        
        m_clawController.leftBumper().onTrue(m_robotDrive.runOnce(() -> m_robotDrive.setMaxOutput(0.3)));
        m_clawController.leftBumper().onFalse(m_robotDrive.runOnce(() -> m_robotDrive.setMaxOutput(1.0)));
        //driveTrain Controls

        m_driverController.a().whileTrue(new DriveRoundTurnCmd(m_robotDrive, 0.5));
        m_driverController.leftBumper().onTrue(m_robotDrive.runOnce(() -> m_robotDrive.setMaxOutput(0.3)));
        m_driverController.leftBumper().onFalse(m_robotDrive.runOnce(() -> m_robotDrive.setMaxOutput(1.0)));
        lCrabwalk.whileTrue(new DriveRightSidewaysCmd(m_robotDrive, 0.5));
        rCrabwalk.whileTrue(new DriveRightSidewaysCmd(m_robotDrive, 0.5));
    }
    
    private void configureButtonsForAutoTesting() {
        m_clawController.a().whileTrue(new ElevatorVerticalCmd(elevatorSubsystem, 0.5));
        m_clawController.x().whileTrue(new ElevatorSlideCmd(elevatorSubsystem, 0.5));
        m_clawController.y().whileTrue(new IntakeSetOpenCmd(intakeSubsystem, true));
        m_clawController.b().whileTrue(new IntakeSetOpenCmd(intakeSubsystem, false));
        m_clawController.leftTrigger().whileTrue(new ElevatorSlideRetractedCommand(elevatorSubsystem, 0.5));
        m_clawController.rightTrigger().whileTrue(new ElevatorSlideExtendedCommand(elevatorSubsystem, 0.5));
        m_clawController.leftBumper().whileTrue(new ElevatorVerticalSetTopCmd(elevatorSubsystem, 0.5));
        m_clawController.rightBumper().whileTrue(new ElevatorVerticalSetBottomCmd(elevatorSubsystem, 0.5));
        
        //Went Forward
        m_driverController.rightTrigger().whileTrue(new DriveForwardCmd(m_robotDrive, 1));
        //Went Backward
        m_driverController.leftTrigger().whileTrue(new DriveBackwardCmd(m_robotDrive, 1));
        //Sorta Works
        m_driverController.rightBumper().whileTrue(new WideRightTurnCmd(m_robotDrive, 1));
        //Sorta Works
        m_driverController.leftBumper().whileTrue(new WideLeftTurnCmd(m_robotDrive, 1));
        //TODO: Add anther file for Left?
        //Test in Library. Looks OK
        m_driverController.back().whileTrue(new DriveRearTurnCmd(m_robotDrive, 1));
        //TODO: Add anther file for Left?
        //Test in Library. Looks OK
        m_driverController.start().whileTrue(new DriveRoundTurnCmd(m_robotDrive, 1));
        //Forward Right
        m_driverController.a().whileTrue(new DriveRightDiagonalCmd(m_robotDrive, 1));
        //Sideways Right
        m_driverController.x().whileTrue(new DriveRightSidewaysCmd(m_robotDrive, 1));
        //TODO: Check that this moves Left
        m_driverController.b().whileTrue(new DriveLeftDiagonalCmd(m_robotDrive, -1));
        //TODO: Check that this moves Left
        m_driverController.y().whileTrue(new DriveLeftSidewaysCmd(m_robotDrive, -1));
        m_clawController.rightTrigger().whileTrue(new StopCmd(m_robotDrive, 0));
    }
    private void configureWheels() {
        m_robotDrive.setInverted(kWheels.frontLeft);
        m_robotDrive.setInverted(kWheels.rearLeft);
    }
    public Command getAutonomousCommand() {
        //Dummy test sequence
        return Commands.sequence(
            new DriveForwardCmd(m_robotDrive, 0).withTimeout(1.5),
            new FindAprilTagCmd(m_robotDrive, limelightVisionSubsystem, 0, 0, 0),
            new RotateTillTagFoundCmd(m_robotDrive, limelightVisionSubsystem, 0, 0.3),
            new IntakeDropCoralCmd(intakeSubsystem, true),
            new DriveBackwardCmd(m_robotDrive, 0).withTimeout(1.0)
        );


        //TODO: Figure out
        //new IntakeSetOpenCmd(intakeSubsystem, false);
        // Create config for trajectory
        // Add kinematics to ensure max speed is actually obeyed
        /*TrajectoryConfig config = new TrajectoryConfig(2.2, 2.2)
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
        );*/
    }
    
    /**
    * Create a gamepad axis for triggering commands as if it were a button.
    *
    * @param controller    The controller whose axis is being monitored for input
    * @param axisNumber    The controller's stick axis number
    * @param threshold     How far the stick must move to trigger a command (0.0 - 1.0)
    * @param isDownDir    Is this measurement in the down direction? (Note the threshold should likely be negated as well
    */ 
    private boolean axisOverThreshold(CommandXboxController controller, int axis, double threshold, boolean isDownDir) {
        if(isDownDir == true){
            return controller.getRawAxis(axis) <= threshold;
        }
        
        return controller.getRawAxis(axis) >= threshold;
    }
    
    
    public void testVisionCoordinates() {
        System.out.println("****Poses:  ");
        // System.out.println(llVisionSubsystem.getKnownPose("RobotBluReef1Left"));
        // System.out.println(LimelightVisionSubsystem.getKnownPose("RobotBluReef1Right"));
        
        List<String> keys = new ArrayList<>();
        for(String k : RobotPoseConstants.visionRobotPoses.keySet()) {
            keys.add(k);
        }
        for (String key : keys) { 
            System.out.println(key + RobotPoseConstants.visionRobotPoses.get(key));
        }   
    }    
   
 }
