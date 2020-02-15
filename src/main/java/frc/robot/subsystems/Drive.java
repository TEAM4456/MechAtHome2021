/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * The drivetrain of the robot.
 */
public class Drive extends SubsystemBase {
    private final WPI_TalonSRX leftDrive, rightDrive;
    
    /**
     * The drivetrain of the robot. Takes in two {@link WPI_TalonSRX}s: a right master talon,
     * and a left master talon.
     * 
     * @param leftTalon - the left master {@link WPI_TalonSRX} of the robot
     * @param rightTalon - the right master {@link WPI_TalonSRX} of the robot 
     */
	public Drive(WPI_TalonSRX leftTalon, WPI_TalonSRX rightTalon) {
		leftDrive = leftTalon;
        rightDrive = rightTalon;
	}

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Position", leftDrive.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Position", rightDrive.getSelectedSensorPosition());
        SmartDashboard.putNumber("Actuator Position", RobotMap.actuator.getSelectedSensorPosition());
    }

    public void autoAlign(NetworkTableEntry x){
        double leftVal, rightVal;

        if (Math.abs(x.getDouble(0.0)) > 10)
        {
            leftVal = x.getDouble(0.0) * 50;
            rightVal = x.getDouble(0.0) * -50;
        }
        else if (Math.abs(x.getDouble(0.0)) > 4) 
        {
            leftVal = x.getDouble(0.0) * 100;
            rightVal = x.getDouble(0.0) * -100;
        }
        else
        {
            leftVal = x.getDouble(0.0) * 200;
            rightVal = x.getDouble(0.0) * -200;
        }

        SmartDashboard.putNumber("Left Velocity", leftVal);
        SmartDashboard.putNumber("Right Velocity", rightVal);

        leftDrive.set(ControlMode.Velocity, leftVal);
        rightDrive.set(ControlMode.Velocity, rightVal);
    }
}