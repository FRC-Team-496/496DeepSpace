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
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.Talon;


/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  AHRS ahrs;
  Talon left, right;
  DifferentialDrive  m_drive;

  public DriveTrain() {
    super();
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      
      System.out.println(e);
    }
    left = new Talon(0);
    right = new Talon(1);
    m_drive = new DifferentialDrive (left,right);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
