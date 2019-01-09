/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LineTracker extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private int[] lines = new int[3];
  private DigitalInput leftLineTracker, centerLineTracker, rightLineTracker;

  public LineTracker() {
    leftLineTracker = new DigitalInput(RobotMap.leftLineTracker);
    centerLineTracker = new DigitalInput(RobotMap.centerLineTracker);
    rightLineTracker = new DigitalInput(RobotMap.rightLineTracker);
  }

  public int getLinePostion() {
    lines[0] = leftLineTracker.get() ? 1 :0;
    lines[1] = centerLineTracker.get() ? 1 : 0;
    lines[2] = rightLineTracker.get() ? 1 : 0;
    return Integer.parseInt(""+lines[0] + lines[1] + lines[2]);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
