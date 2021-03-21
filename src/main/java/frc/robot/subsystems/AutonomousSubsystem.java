// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.*;

public class AutonomousSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final WPI_TalonSRX m_right = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_left = new WPI_TalonSRX(4);
  private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(3);
  /*
   * 3/12 edit testing new code private final SpeedControllerGroup m_leftMotors =
   * new SpeedControllerGroup( new WPI_TalonSRX(4), new WPI_TalonSRX(3));
   * 
   * // The motors on the right side of the drive. private final
   * SpeedControllerGroup m_rightMotors = new SpeedControllerGroup( new
   * WPI_TalonSRX(1), new WPI_TalonSRX(2));
   */
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_right, m_rightFollower);
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_left, m_leftFollower);
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(RobotMap.leftMaster, RobotMap.rightMaster);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public AutonomousSubsystem() {
    /*
     * Sets the distance per pulse for the encoders
     * //RobotMap.leftMaster.configSelectedFeedbackSensor(FeedbackDevice.
     * QuadEncoder, 0, 0);
     * //RobotMap.rightMaster.configSelectedFeedbackSensor(FeedbackDevice.
     * QuadEncoder, 0, 0); //m_leftEncoder.configPulseWidthPeriod_EdgesPerRot(4096,
     * 0);
     * //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse)
     * ;
     * 
     * @return
     */

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(getHeading());

    m_right.setSensorPhase(true);
    m_left.setSensorPhase(false);

    m_right.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
    m_left.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
    m_right.configVelocityMeasurementWindow(1);
    m_left.configVelocityMeasurementWindow(1);

    m_right.set(ControlMode.Velocity, 0);
    m_left.set(ControlMode.Velocity, 0);

    m_drive.setRightSideInverted(DriveConstants.kRightInverted);

    m_drive.setSafetyEnabled(false);
  }

  private void resetEncoders() {
    m_right.setSelectedSensorPosition(0);
    m_left.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PosX", m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("PosY", m_odometry.getPoseMeters().getTranslation().getY());

    SmartDashboard.putNumber("Left Distance (meters)", getPositionLeft());
    SmartDashboard.putNumber("Right Distance (meters)", getPositionRight());
    SmartDashboard.putNumber("Left Velocity", m_left.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse / 1000 * 10);
    SmartDashboard.putNumber("Right Velocity", m_right.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse / 1400 * 10);


    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Turn Rate", getTurnRate());

    // Update the odometry in the periodic block
    m_odometry.update(getHeading(), getPositionLeft(), getPositionRight());
  }

  public double getPositionLeft() {
    return m_left.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse / 910;
  }

  public double getPositionRight() {
    return m_right.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse / 1290;
  }

  public double getAverageDistance() {
    return (getPositionLeft() + getPositionRight()) / 2.0;
  }

  // public DifferentialDriveKinematics getKinematics() {
  // return kinematics;
  // }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_left.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * 10,
        m_right.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse * 10);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  public void resetOdometry(Pose2d pose) {
  //  resetEncoders();
  m_odometry.resetPosition(pose, getHeading());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  /*
   * public void resetEncoders() { m_leftEncoder.reset(); m_rightEncoder.reset();
   * }
   * 
   * 
   * /** Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   * 
   * public double getAverageEncoderDistance() { return
   * (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0; }
   */
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   * 
   *         public Encoder getLeftEncoder() { return m_leftEncoder; }
   * 
   * 
   *         Gets the right drive encoder.
   *
   * @return the right drive encoder
   * 
   *         public Encoder getRightEncoder() { return m_rightEncoder; }
   */
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */

  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.resetDisplacement();
  }


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
 
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
