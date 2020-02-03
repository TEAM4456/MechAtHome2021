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

public class Shooter extends SubsystemBase {
    private static WPI_TalonSRX top;
    private static WPI_TalonSRX bottom;

    /**
     * Creates a new Shooter.
     */
    public Shooter(WPI_TalonSRX topFlywheel, WPI_TalonSRX bottomFlywheel) {
        top = topFlywheel;
        bottom = bottomFlywheel;
    }

    public void shoot(double topVal, double bottomVal){
        top.set(ControlMode.PercentOutput, topVal);
        bottom.set(ControlMode.PercentOutput, bottomVal);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
