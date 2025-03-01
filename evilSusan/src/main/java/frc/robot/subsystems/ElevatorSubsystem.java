package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.Physics;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorSlidePositions;


public class ElevatorSubsystem extends SubsystemBase{
    private final TalonSRX m_verticalMotorB = new TalonSRX(ElevatorConstants.kVerticalMotorPortB);
    private final TalonSRX m_verticalMotorA = new TalonSRX(ElevatorConstants.kVerticalMotorPortA);
    private final SparkMax m_slideMotor = new SparkMax(ElevatorConstants.kSlideMotorPort, MotorType.kBrushed);

    //private final SparkRelativeEncoder sparkEncoder = m_verticalMotor.get
    Encoder enc;

    public ElevatorSubsystem() {
        enc = new Encoder(ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);
        enc.setDistancePerPulse(Math.PI*OIConstants.wheelDiameter/OIConstants.SRXMagEncoderCPR);

        //sparkEncoder.getPosition();
    }

    @Override
    public void periodic() {
        double dist = enc.getDistance();
        //SmartDashboard.puNumber("Encoder", dist);
        SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    }

    public void setSlideMotor(double speed) {
        m_slideMotor.set(speed);
    }

    public void setVerticalMotor(double speed) {
        m_verticalMotorA.set(ControlMode.Position, speed);
        m_verticalMotorB.set(ControlMode.Position, speed);
    }

    public double getEncoderMeters() {
        return enc.get() * ElevatorConstants.kEncoderTick2Meter;
    }
}
