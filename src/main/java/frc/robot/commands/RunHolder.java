/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.Timer;
>>>>>>> 2107a0fa566913e5df26bafe89d9d1b636503b6e
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Holder;

public class RunHolder extends CommandBase {
  private final Holder holder;
  private final double percent;
  //private final double time;
  //private final Timer timer;
  private double direction;
  
  /**
   * Creates a new RunHolder.
   */
  public RunHolder(Holder holderSubsystem, double percentOutput/*, double runTime*/) {
    holder = holderSubsystem;
    percent = percentOutput;
    //time = runTime;
    //timer = new Timer();
    addRequirements(holderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      holder.RunHolder(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    direction = holder.getDirection();
    if (direction < 0) {
        SmartDashboard.putBoolean("Holder Closed", false);
    } else {
        SmartDashboard.putBoolean("Holder Closed", true);
    }
    holder.RunHolder(0);
    //timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        /*if(SmartDashboard.getBoolean("Holder Closed", true)){
            return timer.hasPeriodPassed(time) && (holder.getDirection() < 0);
        } else {
            return timer.hasPeriodPassed(time) && (holder.getDirection() > 0);
        }*/
        return false;
  }
}
