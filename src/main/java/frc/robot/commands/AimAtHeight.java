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
    private boolean isHigher;
    
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
        isHigher = (position > actuator.getActuatorPosition());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        actuator.setActuatorSpeed(-0.2);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        actuator.setActuatorSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(isHigher){
            return (position <= actuator.getActuatorPosition());
        } else {
            return (position >= actuator.getActuatorPosition());
        }

    }
}
