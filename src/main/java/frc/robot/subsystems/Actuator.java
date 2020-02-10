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

public class Actuator extends SubsystemBase {
    private final WPI_TalonSRX actuator;

    /**
    * Creates a new Actuator.
    */
    public Actuator(WPI_TalonSRX actuatorTalon) {
        actuator = actuatorTalon;
    }

    public void aimAtHeight(double position){
        actuator.set(ControlMode.Position, position);
    }

    public void setActuatorSpeed(double percent){
        actuator.set(ControlMode.PercentOutput, percent);
    }
}
