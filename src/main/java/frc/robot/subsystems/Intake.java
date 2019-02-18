/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX m_intake;
  DigitalInput ballLoaded;
  public Intake() {
    m_intake = new WPI_TalonSRX(1);
    ballLoaded = new DigitalInput(RobotMap.BALL_LOADED);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void set(double speed) {
    m_intake.set(speed);
  }

  public void stop() {
    m_intake.set(0);
  }

  public boolean isBallLoaded() {
    return ballLoaded.get();
  }

  public void log() {
    SmartDashboard.putData(ballLoaded);
  }
}
