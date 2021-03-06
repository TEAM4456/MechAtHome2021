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

public class LeftWinch extends SubsystemBase {
  private final WPI_TalonSRX leftWinch;
  /**
   * Creates a new LeftWinch.
   */
  public LeftWinch(WPI_TalonSRX leftWinchTalon) {
    leftWinch = leftWinchTalon;

  }
  public void setLeftWinchSpeed(double percent){
		leftWinch.set(ControlMode.PercentOutput, percent);
  }
}
 
