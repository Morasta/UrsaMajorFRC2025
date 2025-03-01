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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class IntakeSubsystem extends SubsystemBase {
    // TODO: Revise these to use the newly installed motors, talons?
    private TalonSRX intakeUpMotor = new TalonSRX(IntakeConstants.kLeftMotorPort);
    private TalonSRX intakeDownMotor = new TalonSRX(IntakeConstants.kRightMotorPort);

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
            intakeUpMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
            intakeDownMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
        } else {
            System.out.println("setting intake to closed");
            intakeUpMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
            intakeDownMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
        }
    }

    private void setCoralPosition(boolean open) {
        if (open) {
            System.out.println("setting intake to open");
            intakeUpMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
        } else {
            System.out.println("setting intake to closed");
            intakeUpMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
        }
    }
    
    public void setOpen() {
        this.setPosition(true);
        this.setCoralPosition(true);
    }

    public void setClosed() {
        this.setPosition(false);
        this.setCoralPosition(false);
    }
}
