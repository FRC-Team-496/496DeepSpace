/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RotateTo extends Command {
  PIDController angleController;
  private double angle;
  public RotateTo(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.angle = angle;
    requires(Robot.m_driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_driveTrain.resetGyro();
    PIDSource angleSource = new PIDSource(){
    
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        
      }
    
      @Override
      public double pidGet() {
        return Robot.m_driveTrain.getAngle();
      }
    
      @Override
      public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
      }
    };

    PIDOutput angleOut = new PIDOutput(){
    
      double output;

      public double get() {
        return output;
      }
      @Override
      public void pidWrite(double output) {
        this.output = output;
      }
    };

    angleController = new PIDController(0.1, 0, 0, angleSource, angleOut);
    angleController.setAbsoluteTolerance(1);
    angleController.setOutputRange(-.5, .5);
    angleController.setInputRange(-180f, 180f);

    angleController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_driveTrain.drive(angleController.get(), -angleController.get());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return angleController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
