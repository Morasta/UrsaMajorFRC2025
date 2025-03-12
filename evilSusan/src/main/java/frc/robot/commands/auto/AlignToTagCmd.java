package frc.robot.commands.auto;

import frc.robot.commands.drive.DriveLeftSidewaysCmd;
import frc.robot.commands.drive.DriveRightSidewaysCmd;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimelightVisionSubsystem;

public class AlignToTagCmd {
    private final LimelightVisionSubsystem visionSubsystem = new LimelightVisionSubsystem();
    private final DriveTrain driveTrain = new DriveTrain();
    
        public void alignToAprilTag() {
        double Tx = visionSubsystem.getXValue();
        double tolerance = 0; // TODO: update tolerance

        System.out.println("alignment x value: " + Tx);
        // IF TX+tolerance > ???
        // strafe another way
        // IF TX+tolerance > ???
        // strafe one way
        if (Tx + tolerance > 1) {
            driveTrain.runOnce(() -> new DriveRightSidewaysCmd(driveTrain, 1, 0.3));
        } else if (Tx + tolerance < 1) {
            driveTrain.runOnce(() -> new DriveLeftSidewaysCmd(driveTrain, 1, 0.3));
        }
    }
}
