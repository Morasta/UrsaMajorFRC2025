package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.SlideConstants;


public class SlideSubsystem extends SubsystemBase{
    private final SparkMax m_slideMotor = new SparkMax(SlideConstants.kSlideMotorPort, MotorType.kBrushed);

    public SlideSubsystem() {
        
        // enc = new Encoder(ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);
        // enc.setDistancePerPulse(Math.PI*RobotChassis.wheelDiameter/RobotChassis.SRXMagEncoderCPR);
        // sparkEncoder.getPosition();
    }

    public void setSlideMotor(double speed) { 
        m_slideMotor.set(speed);
    }

    public void stopSlideMotors() {
        m_slideMotor.set(0);
    }

    // TODO: tie this to encoders
    /*public double getEncoderMeters() {
        return enc.get() * ElevatorConstants.kEncoderTick2Meter;
    }*/

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