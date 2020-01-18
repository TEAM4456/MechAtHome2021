/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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

    public static void init(){
        leftMaster = new WPI_TalonSRX(1);
        leftMaster.setInverted(false);
        leftFollower = new WPI_TalonSRX(2);
        leftFollower.setInverted(false);
        leftFollower.set(ControlMode.Follower, 1);

        rightMaster = new WPI_TalonSRX(3);
        rightMaster.setInverted(false);
        rightFollower = new WPI_TalonSRX(4);
        rightFollower.setInverted(false);
        rightFollower.set(ControlMode.Follower, 3);

    }
}
