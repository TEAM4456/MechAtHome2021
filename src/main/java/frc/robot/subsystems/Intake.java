/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final WPI_TalonSRX intake;

    public Intake(WPI_TalonSRX intakeTalon) {
        intake = intakeTalon;
    }

    public void runIntake(double percent){
        intake.set(ControlMode.PercentOutput, percent);
    }

    public boolean getIntakeRunning(){
        return (intake.get() != 0);
    }
}
