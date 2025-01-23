package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/*
import frc.robot.mecanumcontrollercommand.Constants.AutoConstants;
import frc.robot.mecanumcontrollercommand.Constants.DriveConstants;
import frc.robot.mecanumcontrollercommand.Constants.OIConstants;
import frc.robot.mecanumcontrollercommand.subsystems.DriveSubsystem;
*/
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import java.util.List;

public class RobotContainer {
    private final DriveTrain m_robotDrive = new DriveTrain();

    public RobotContainer() {

        XboxController m_driverController = new XboxController(OIConstants.kDriverJoystickPort);
        // Set up the buttons and tell the robot what they need to do
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () ->
                m_robotDrive.drive(
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
        /*driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, //
                () -> -joystick1.getRawAxis(OIConstants.kArcadeDriveSpeedAxis),
                () -> joystick1.getRawAxis(OIConstants.kArcadeDriveTurnAxis))//
        );*/
    }

    private void configureButtonBindings() {
        // Add the controller button stuff here, as an example: 
        new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
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
       //return new PrintCommand("Executed autotonomous command!");
       return new DriveForwardCmd(m_robotDrive, 1);
    }
}
