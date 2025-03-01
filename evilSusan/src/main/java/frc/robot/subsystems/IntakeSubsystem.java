package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class IntakeSubsystem extends SubsystemBase {
    // TODO: Revise this to the number of motors, and their config, that the build team uses
    private TalonSRX intakeLeftMotor = new TalonSRX(IntakeConstants.kLeftMotorPort);
    private TalonSRX intakeRightMotor = new TalonSRX(IntakeConstants.kRightMotorPort);

        Encoder enc;

    public IntakeSubsystem() {
        enc = new Encoder(IntakeConstants.kLeftEncoderA, IntakeConstants.kLeftEncoderB);
        enc = new Encoder(IntakeConstants.kRightEncoderA, IntakeConstants.kRightEncoderB);
        enc.setDistancePerPulse(Math.PI*OIConstants.wheelDiameter/OIConstants.SRXMagEncoderCPR);
    }

    @Override
    public void periodic() {
        double dist;
        //SmartDashboard.putNumber("Encoder", dist);
    }

    private void setPosition(boolean open) {
        if (open) {
            System.out.println("setting intake to open");
            intakeLeftMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
            intakeRightMotor.set(ControlMode.Position, IntakeConstants.kOpenSpeed);
        } else {
            System.out.println("setting intake to closed");
            intakeLeftMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
            intakeRightMotor.set(ControlMode.Position, IntakeConstants.kCloseSpeed);
        }
    }
    
    public void setOpen() {
        this.setPosition(true);
    }

    public void setClosed() {
        this.setPosition(false);
    }
}
