/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.*;

/**
 * The drivetrain of the robot.
 */
public class Drive extends SubsystemBase {
    private final WPI_TalonSRX leftDrive, rightDrive;
    private final AHRS gyro = new AHRS();
    
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

        SmartDashboard.putNumber("Left Distance (meters)", getPositionLeft());
        SmartDashboard.putNumber("Right Distance (meters)", getPositionRight());
        SmartDashboard.putNumber("Left Velocity", leftDrive.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse / 1000 * 10);
        SmartDashboard.putNumber("Right Velocity", rightDrive.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse / 1400 * 10);

        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("Turn Rate", getTurnRate());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("output", leftDrive.getMotorOutputVoltage());
        
    }

    public void autoAlign(NetworkTableEntry x){
        if (x.getDouble(0.0) > 0){
            leftDrive.set(ControlMode.Velocity, 1000);
            rightDrive.set(ControlMode.Velocity, -1000);
        } else {
            leftDrive.set(ControlMode.Velocity, -1000);
            rightDrive.set(ControlMode.Velocity, 1000);
        }

        /*double leftVal, rightVal;

        
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
        
        leftVal = leftDrive.getSelectedSensorPosition() - x.getDouble(0.0) * 30;
        rightVal = rightDrive.getSelectedSensorPosition() + x.getDouble(0.0) * 30;

        leftDrive.set(ControlMode.Position, leftVal);
        rightDrive.set(ControlMode.Position, rightVal);
        */
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

    public void setVelocity(double velocity){
        leftDrive.set(ControlMode.Velocity, velocity);
        rightDrive.set(ControlMode.Velocity, velocity);
    }

    public void spin(double velocity){
        leftDrive.set(ControlMode.Velocity, velocity);
        rightDrive.set(ControlMode.Velocity, -velocity);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
      }
     
      public double getTurnRate() {
        return -gyro.getRate();
      }

      public double getPositionLeft() {
        return leftDrive.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse / 910;
      }
    
      public double getPositionRight() {
        return rightDrive.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse / 1290;
      }
}