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
    private static WPI_TalonSRX topShooter;
    private static WPI_TalonSRX bottomShooter;
	public static double currentDistance;

    /**
     * Creates a new Shooter.
     */
    public Shooter(WPI_TalonSRX topTalon, WPI_TalonSRX bottomTalon) {
        topShooter = topTalon;
        bottomShooter = bottomTalon;
    }

    public void shoot(double topVal, double bottomVal){
        topShooter.set(ControlMode.PercentOutput, topVal);
        bottomShooter.set(ControlMode.PercentOutput, bottomVal);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
