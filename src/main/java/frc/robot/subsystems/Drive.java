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

	/**
	 * Controls the {@link Drive} using joysticks. This is for tank style controls, with one controller axis
	 * mapped to the speed of the left side of the drivetrain and one mapped to the speed of the right side.
	 * 
	 * @param leftAxis - the controller axis controlling the speed of the left wheels 
	 * @param rightAxis - the controller axis controlling the speed of the right wheels
	 */
	public void tankDrive(DoubleSupplier leftAxis, DoubleSupplier rightAxis){ 
		leftDrive.set(ControlMode.PercentOutput, leftAxis.getAsDouble());
		rightDrive.set(ControlMode.PercentOutput, rightAxis.getAsDouble());
	}

	/**
	 * Controls the {@link Drive} using joysticks. This is for arcade or split-arcade style controls, with a 
	 * controller axis mapped to the x-axis motion of the robot, and one mapped to the y-axis motion.
	 * 
	 * @param xAxis - the controller axis controlling the x-axis motion of the robot
	 * @param yAxis - the controller axis controlling the y-axis motion of the robot
	 */
	public void arcadeDrive(DoubleSupplier xAxis, DoubleSupplier yAxis){
		double leftVal, rightVal;

		double x = xAxis.getAsDouble();
		double y = yAxis.getAsDouble();
		leftVal = (y + (x/2)) / 1.5;
		rightVal = (y - (x/2)) / 1.5;

		leftDrive.set(ControlMode.PercentOutput, leftVal);
		rightDrive.set(ControlMode.PercentOutput, rightVal);
	}

	/**
	 * Controls the {@link Drive} using the triggers and a joystick. This is for RL-style controls, with the 
	 * triggers controlling the y-axis (one being positive and the other negative) and a joystick controlling 
	 * the x-axis.
	 * 
	 * @param xAxis - the controller axis controlling the x-axis motion of the robot 
	 * @param yAxisNeg - the controller axis (usually a trigger) controlling the backwards motion of the robot
	 * @param yAxisPos - the controller axis (usually a trigger) controlling the forwards motion of the robot
	 */
	public void arcadeDrive(DoubleSupplier xAxis, DoubleSupplier yAxisNeg, DoubleSupplier yAxisPos){
		double leftVal, rightVal;

		double x = xAxis.getAsDouble();
		double y = yAxisPos.getAsDouble() - yAxisNeg.getAsDouble();
		leftVal = (y + (x/2)) / 1.5;
		rightVal = (y - (x/2)) / 1.5;

		leftDrive.set(ControlMode.PercentOutput, leftVal);
		rightDrive.set(ControlMode.PercentOutput, rightVal);
    }

    public void autoAlign(NetworkTableEntry x){
        double leftVal = x.getDouble(0.0)/30;
        double rightVal = -leftVal;

        SmartDashboard.putNumber("Left %Output", leftVal);
        SmartDashboard.putNumber("Right %Output", rightVal);

        leftDrive.set(ControlMode.PercentOutput, leftVal);
        rightDrive.set(ControlMode.PercentOutput, rightVal);
    }
}