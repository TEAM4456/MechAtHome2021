/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class BasicAutoCommand extends SequentialCommandGroup {
    /**
     * Creates a new BasicAutoCommand.
     */
    public BasicAutoCommand(Drive driveSubsystem, Actuator actuatorSubsystem, Shooter shooterSubsystem, Intake intakeSubsystem) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(new SetDriveVelocity(driveSubsystem, 1000.0, 0.5), new AutoAlign(driveSubsystem), new AimAtHeight(actuatorSubsystem, -5000.0), 
              new TimedShoot(shooterSubsystem, 3.0), new RunIntake(intakeSubsystem, 0.2));
    }

}
