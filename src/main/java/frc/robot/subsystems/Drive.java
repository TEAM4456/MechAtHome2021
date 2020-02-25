/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
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
/*
    public void followPath(String path){
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            RamseteCommand ramseteCommand = new RamseteCommand(trajectory, 
                                                               new Pose2d(-10.26, 1.0, new Rotation2d(0.0)), 
                                                               new RamseteController(b, zeta), 
                                                               new SimpleMotorFeedforward(ks, kv), 
                                                               new , 
                                                               wheelSpeeds, 
                                                               leftController, 
                                                               rightController, 
                                                               outputVolts, 
                                                               requirements);
          } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }
    }
    */
}