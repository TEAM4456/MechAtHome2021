/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;

public class ToggleEndGame extends CommandBase {
  /**
   * Creates a new SetEndGame.
   */
  public ToggleEndGame() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotMap.ENDGAME = !RobotMap.ENDGAME;    
    SmartDashboard.putBoolean("Endgame", RobotMap.ENDGAME);
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
