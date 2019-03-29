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
import frc.robot.commands.LIftWithStick;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX elevator;
  DigitalInput isUp, isDown;
  public Elevator(){

    elevator = new WPI_TalonSRX(0);
    isUp = new DigitalInput(3);
    isDown = new DigitalInput(2);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LIftWithStick());
  }

  public void lift(double speed) {
    elevator.set(speed);
  }

  public void stop() {
    elevator.set(0);
  }

  public boolean isUp() {
    return isUp.get();
  }

  public boolean isDown() {
    return isDown.get();
  }

}
