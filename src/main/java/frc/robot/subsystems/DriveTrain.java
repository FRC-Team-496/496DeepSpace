/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;


/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  AHRS ahrs;
  Talon leftFront,leftRear, rightFront,rightRear;
  DifferentialDrive  m_drive;

  public DriveTrain() {
    super();
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      
      System.out.println(e);
    }
    leftFront = new Talon(0);
    leftRear = new Talon(1);
    rightFront = new Talon(2);
    rightRear = new Talon(3);
    SpeedControllerGroup m_right = new SpeedControllerGroup(leftFront, leftRear);
    SpeedControllerGroup m_left = new SpeedControllerGroup(rightFront, rightRear);
    m_drive = new DifferentialDrive (m_left,m_right);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithJoystick());
  }

  public void drive(double x, double rot, boolean isQuickTurn) {

    m_drive.curvatureDrive(x, rot, isQuickTurn);

  }

  public void stop() {
    m_drive.curvatureDrive(0, 0, false);;
  }
}
