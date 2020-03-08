/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutoAlign extends CommandBase {
    private boolean isLeft;
    private final Drive drive;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * Creates a new autoAlign.
     */
    public AutoAlign(Drive driveSubsystem) {
        drive = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        table.getEntry("ledMode").forceSetNumber(3);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        isLeft = (table.getEntry("tx").getDouble(0.0) > 0);
        if(isLeft){
            if (table.getEntry("tx").getDouble(0.0) < 15){
                drive.spin(400);
            } else {
                drive.spin(1000);
            }
        } else {
            if (table.getEntry("tx").getDouble(0.0) > -15){
                drive.spin(-400);
            } else {
                drive.spin(-1000);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        table.getEntry("ledMode").forceSetNumber(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (isLeft){
            return (table.getEntry("tx").getDouble(0.0) >= 0);
        } else {
            return (table.getEntry("tx").getDouble(0.0) < 0);
        }
    }
}
