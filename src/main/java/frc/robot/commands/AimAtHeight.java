/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Actuator;

public class AimAtHeight extends CommandBase {
    private final Actuator actuator;
    private final double position;
    
    /**
     * Creates a new AimAtHeight.
     */
    public AimAtHeight(Actuator actuatorSubsystem, double positionVal) {
        // Use addRequirements() here to declare subsystem dependencies.
        actuator = actuatorSubsystem;
        position = positionVal;
        addRequirements(actuatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        actuator.aimAtHeight(position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
