package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonSRX m_verticalLeftMotor = new TalonSRX(ElevatorConstants.kVerticalLeftMotorPort);
    private final TalonSRX m_verticalRightMotor = new TalonSRX(ElevatorConstants.kVerticalRightMotorPort);
    ElevatorFeedforward feedForward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    // private final SparkRelativeEncoder sparkEncoder = m_verticalMotor.get
    // Encoder enc;

    public ElevatorSubsystem() {
        
        // enc = new Encoder(ElevatorConstants.kEncoderChannelA, ElevatorConstants.kEncoderChannelB);
        // enc.setDistancePerPulse(Math.PI*RobotChassis.wheelDiameter/RobotChassis.SRXMagEncoderCPR);
        // sparkEncoder.getPosition();
        
        // Note: the orientation of the motors in the gearbox is such that they should both be spinning the same way
        m_verticalLeftMotor.setInverted(false);
        m_verticalRightMotor.setInverted(false);

        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#elevatorfeedforward
        // Create a new ElevatorFeedforward with gains kS, kG, kV, and kA
        //ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        //feedforward.calculate(0);
        
        //serenityFeedForward1();
    }

    public void serenityFeedForward1(double leftVelocity, double rightVelocity) {
        feedForward.calculate(rightVelocity);
        feedForward.calculate(leftVelocity);
    }

    public void setMotorBrakeMode(NeutralMode mode) {
        m_verticalLeftMotor.setNeutralMode(mode);
        m_verticalRightMotor.setNeutralMode(mode);
    }

    public void setVerticalMotor(double speed) {
        m_verticalLeftMotor.set(ControlMode.PercentOutput, speed);
        m_verticalRightMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopVerticalMotors() {
        m_verticalLeftMotor.set(ControlMode.PercentOutput, 0);
        m_verticalRightMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setIdleElevator(boolean holdingAlgae) {
        double idleSpeed = -0.18;
        double algaeIdleSpeed = -0.187;
        if (holdingAlgae) {
            System.out.println("setting ElevatorMotors to holdingAlgae");
            m_verticalLeftMotor.set(ControlMode.PercentOutput, algaeIdleSpeed);
            m_verticalRightMotor.set(ControlMode.PercentOutput, algaeIdleSpeed);
        } else {
            System.out.println("setting ElevatorMotors to NOT holdingAlgae");
            m_verticalLeftMotor.set(ControlMode.PercentOutput, idleSpeed);
            m_verticalRightMotor.set(ControlMode.PercentOutput, idleSpeed);
        }
    }

    // TODO: tie this to encoders
    /*public double getEncoderMeters() {
        return enc.get() * ElevatorConstants.kEncoderTick2Meter;
    }*/

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

}
