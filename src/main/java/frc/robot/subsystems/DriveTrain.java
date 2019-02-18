/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  AHRS ahrs;
  WPI_VictorSPX left, right;
  DifferentialDrive  m_drive;
  Encoder left_Encoder, right_Encoder;

  public DriveTrain() {
    super();
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      
      System.out.println(e);
    }
    left = new WPI_VictorSPX(0);
    right = new WPI_VictorSPX(1);
    

    m_drive = new DifferentialDrive (left,right);
    left_Encoder = new Encoder(RobotMap.DRIVETRAIN_ENCODER_LEFT_A,RobotMap.DRIVETRAIN_ENCODER_LEFT_B);
    right_Encoder = new Encoder(RobotMap.DRIVETRAIN_ENCODER_RIGHT_A, RobotMap.DRIVETRAIN_ENCODER_RIGHT_B);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoystick());
  }

  public void drive(double leftSpeed, double rightSpeed){
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void driveCurve(double x, double rot, boolean isQuickTurn) {

    m_drive.curvatureDrive(x, rot, isQuickTurn);

  }

  public void stop() {
    m_drive.curvatureDrive(0, 0, false);;
  }

  public void resetLeftEncoder() {
    left_Encoder.reset();
  }

  public void resetRightEncoder() {
    right_Encoder.reset();
  }

  public void resetGyro() {
    ahrs.reset();
  }

  public double getLeftDistance() {
    return left_Encoder.getDistance();
  }

  public double getRightDistance() {
    return right_Encoder.getDistance();
  }

  public double getAverageDistance() {
    return (getRightDistance() + getLeftDistance()) /2;
  }

  public double getAngle() {
    return ahrs.getAngle();
  }


  }

