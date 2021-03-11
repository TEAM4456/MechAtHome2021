// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.*;


public class AutonomousSubsystem extends SubsystemBase{
  /*comment out testing
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(
          new WPI_TalonSRX(4),
          new WPI_TalonSRX(3));

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(
          new WPI_TalonSRX(1),
          new WPI_TalonSRX(2));

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(RobotMap.leftMaster, RobotMap.rightMaster);

  private final WPI_TalonSRX m_leftEncoder = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);

// The right-side drive encoder
  private final WPI_TalonSRX m_rightEncoder = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);

/*  
  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1];
          //DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1];
          //DriveConstants.kRightEncoderReversed);
*/
/*commenting out testing
  // The gyro sensor
  private final Gyro m_gyro = new AHRS();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
/*
public AutonomousSubsystem() { 
  /* Sets the
   * distance per pulse for the encoders
   * //RobotMap.leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
   * 0, 0);
   * //RobotMap.rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
   * 0, 0); //m_leftEncoder.configPulseWidthPeriod_EdgesPerRot(4096, 0);
   * //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
   * 
   * @return
   */
  /*
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }
  
  private void resetEncoders() {
    m_rightEncoder.setSelectedSensorPosition(0);
    m_leftEncoder.setSelectedSensorPosition(0);
  }
  
/*comment out testing
@Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), getPositionLeft(), getPositionRight());
  }
  

  
  public double getPositionLeft() {
      return m_leftEncoder.getSelectedSensorPosition() / 8570;
  }

  public double getPositionRight() {
      return m_rightEncoder.getSelectedSensorPosition() / 8570;
  }

  public double getAverageDistance() {
      return (getPositionLeft() + getPositionRight()) / 2.0;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  /* commented out testing
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  /*comment out testing
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getSelectedSensorVelocity(), m_rightEncoder.getSelectedSensorVelocity());
  }
  

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  /*comment out testing
  public void resetOdometry(Pose2d pose) {
  //  resetEncoders();
  m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  /*comment out testing
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  /*comment out testing
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  /*
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
*/
  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }
*/
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  /*Comment out testing
   public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  /*comment out testing
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  /*comment out testing
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  /*
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
  */
}