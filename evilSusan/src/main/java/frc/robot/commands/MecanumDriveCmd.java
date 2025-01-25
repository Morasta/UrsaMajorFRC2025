package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class MecanumDriveCmd extends Command {
    private final DriveTrain driveTrain;
    private final Supplier<Double> speedFunction, turnFunction;

    public MecanumDriveCmd(DriveTrain driveTrain, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        System.out.println("started MecanumDrive");
    }

    @Override
    public void execute() {
        double realTimeSpeed = speedFunction.get();
        double realTimeTurn = turnFunction.get();

        double left = realTimeSpeed + realTimeTurn;
        double right = realTimeSpeed - realTimeTurn;
        driveTrain.setMotors(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        
        System.out.println("end of MecanumDrive");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
