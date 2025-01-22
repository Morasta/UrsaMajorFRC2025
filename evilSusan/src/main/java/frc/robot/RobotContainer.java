package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
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
    }
}
