package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotChassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.Physics;
import frc.robot.Constants.ElevatorConstants.ElevatorVerticalPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorSlidePositions;


public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax m_slideMotor = new SparkMax(ElevatorConstants.kSlideMotorPort, MotorType.kBrushed);
    private final TalonSRX m_verticalLeftMotor = new TalonSRX(ElevatorConstants.kVerticalLeftMotorPort);
    private final TalonSRX m_verticalRightMotor = new TalonSRX(ElevatorConstants.kVerticalRightMotorPort);

    //private final SparkRelativeEncoder sparkEncoder = m_verticalMotor.get
    Encoder enc;

    public ElevatorSubsystem() {
        enc = new Encoder(ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);
        enc.setDistancePerPulse(Math.PI*RobotChassis.wheelDiameter/RobotChassis.SRXMagEncoderCPR);

        //sparkEncoder.getPosition();
    }

    @Override
    public void periodic() {
        double dist = enc.getDistance();
        //SmartDashboard.puNumber("Encoder", dist);
        //SmartDashboard.putNumber("Elevator encoder value", getEncoderMeters());
    }

    public void setSlideMotor(double speed) { 
        m_slideMotor.set(speed);
    }

    public void setVerticalMotor(double speed) {
        if (speed > 0.3) 
            speed = 0.3;
        m_verticalLeftMotor.set(ControlMode.Position, speed);
        m_verticalRightMotor.set(ControlMode.Position, speed);
    }

    public double getEncoderMeters() {
        return enc.get() * ElevatorConstants.kEncoderTick2Meter;
    }

    // TODO: make this set the vertical position as needed, stopping at the proper level
    public void setVerticalPosition(double targetPosition) {
        //double pidOutput = pid.calculate(getPosition(), targetPosition);

        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        //double motorOutput = pidOutput + Physics.GRAVITY_COMPENSATION;

        // Clamp the output to valid range
        //motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);

        //setSlideMotor(motorOutput);
    }

    // TODO: make this set the vertical position as needed, stopping at the proper level
    public void setSlidePosition(double targetPosition) {
        //double pidOutput = pid.calculate(getPosition(), targetPosition);

        // Add gravity compensation
        // The sign is positive because we need to work against gravity
        // You might need to flip the sign depending on your motor polarity
        //double motorOutput = pidOutput + Physics.GRAVITY_COMPENSATION;

        // Clamp the output to valid range
        //motorOutput = Math.min(Math.max(motorOutput, -1.0), 1.0);

        //setSlideMotor(motorOutput);
    }


    
}
