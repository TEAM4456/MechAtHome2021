/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Actuator;

public class SetActuatorSpeed extends CommandBase {
    private final Actuator actuator;
    private final double percent;

    /**
     * Creates a new SetActuatorSpeed.
     */
    public SetActuatorSpeed(Actuator actuatorSubsystem, double percentOutput) {
        // Use addRequirements() here to declare subsystem dependencies.
        actuator = actuatorSubsystem;
        percent = percentOutput;
        addRequirements(actuatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        actuator.setActuatorSpeed(percent);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        actuator.setActuatorSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return SmartDashboard.getBoolean("Endgame", false);
    }
}
