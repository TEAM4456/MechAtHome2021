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

public class RightWinch extends SubsystemBase {
  private final WPI_TalonSRX rightWinch;
  /**
   * Creates a new RightWinch.
   */
  public RightWinch(WPI_TalonSRX rightWinchTalon) {
    rightWinch = rightWinchTalon;


  }
  public void setRightWinchSpeed(double percent){
		rightWinch.set(ControlMode.PercentOutput, percent);
  }
}
  
