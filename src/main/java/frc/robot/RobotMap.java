/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Compressor;

/**
 * Class that holds all of the {@link WPI_TalonSRX}, {@link DoubleSolenoid}, and {@link Compressor} bindings 
 * for the robot.
 */
public class RobotMap {
    public static WPI_TalonSRX leftMaster;
    public static WPI_TalonSRX leftFollower;
    public static WPI_TalonSRX rightMaster;
    public static WPI_TalonSRX rightFollower;
    public static WPI_TalonSRX intake;
    public static WPI_TalonSRX topShooter;
    public static WPI_TalonSRX bottomShooter;
    public static WPI_TalonSRX actuator;
<<<<<<< HEAD
    public static WPI_TalonSRX leftWinch;
    public static WPI_TalonSRX rightWinch;
=======
    public static WPI_TalonSRX rotator;
>>>>>>> f0973b4c82133300a7d77784a9525249f02a4d23

    public static void init(){
        leftMaster = new WPI_TalonSRX(4);
        leftMaster.setInverted(true);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        leftMaster.config_kP(0, 0.001, 10);
        leftMaster.config_kI(0, 0.000001, 10);
        leftMaster.config_kD(0, 1.0, 10);
        leftMaster.config_kF(0, 0.3, 10);

        leftFollower = new WPI_TalonSRX(3);
        leftFollower.setInverted(false);
        leftFollower.set(ControlMode.Follower, 4);

        rightMaster = new WPI_TalonSRX(1);
        rightMaster.setInverted(true);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        rightMaster.config_kP(0, 0.001, 10);
        rightMaster.config_kI(0, 0.000001, 10);
        rightMaster.config_kD(0, 1.0, 10);
        rightMaster.config_kF(0, 0.3, 10);

        rightFollower = new WPI_TalonSRX(2);
        rightFollower.setInverted(false);
        rightFollower.set(ControlMode.Follower, 1);

        topShooter = new WPI_TalonSRX(6);
        topShooter.setInverted(false);
        bottomShooter = new WPI_TalonSRX(11);
        bottomShooter.setInverted(false);
        
        intake = new WPI_TalonSRX(8);
        intake.setInverted(true);

        actuator = new WPI_TalonSRX(7);
        actuator.setInverted(true);

<<<<<<< HEAD
        leftWinch = new WPI_TalonSRX(9);
        leftWinch.setInverted(true);

        rightWinch = new WPI_TalonSRX(10);
        rightWinch.setInverted(true);
=======
        rotator = new WPI_TalonSRX(5);
        rotator.setInverted(true);
>>>>>>> f0973b4c82133300a7d77784a9525249f02a4d23
    }
}
