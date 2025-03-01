package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotChassis;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {
    // TODO: Revise these to use the newly installed motors, talons?
    private SparkMax intakeLeftMotor = new SparkMax(IntakeConstants.kLeftMotorPort, MotorType.kBrushless);
    private SparkMax intakeRightMotor = new SparkMax(IntakeConstants.kRightMotorPort, MotorType.kBrushless);

        Encoder enc;

    public IntakeSubsystem() {
        enc = new Encoder(IntakeConstants.kLeftEncoderA, IntakeConstants.kLeftEncoderB);
        enc = new Encoder(IntakeConstants.kRightEncoderA, IntakeConstants.kRightEncoderB);
        enc.setDistancePerPulse(Math.PI*RobotChassis.wheelDiameter/RobotChassis.SRXMagEncoderCPR);
    }

    @Override
    public void periodic() {
        double dist;
        //SmartDashboard.putNumber("Encoder", dist);
    }

    // TODO: make sure this goes to fully open or fully closed, based on encoders
    private void setPosition(boolean open) {
        if (open) {
            System.out.println("setting intake to open");
            intakeLeftMotor.set(IntakeConstants.kOpenSpeed);
            intakeRightMotor.set(IntakeConstants.kOpenSpeed);
        } else {
            System.out.println("setting intake to closed");
            intakeLeftMotor.set(IntakeConstants.kCloseSpeed);
            intakeRightMotor.set(IntakeConstants.kCloseSpeed);
        }
    }
    
    public void setOpen() {
        this.setPosition(true);
    }

    public void setClosed() {
        this.setPosition(false);
    }
}
