// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.PIDshootingRotate;

/** An example command that uses an example subsystem. */
public class LimeLightAutoAdjustCommand extends CommandBase { 
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PIDshootingRotate m_subsystem;
  double Angle;
  boolean isFinished = false;
  boolean inErrorZone = false;
  int count;

  public LimeLightAutoAdjustCommand(PIDshootingRotate subsystem, double angle) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
    Angle = angle;
  }

  @Override
  public void initialize() {
    ShootingRotate.rotateDegrees(Angle);
  }

  @Override
  public void execute() {
    double error = ShootingRotate.turnController.getError();
    inErrorZone = Math.abs(error) < PIDshootingRotate.turretController.setAbsoluteTolerance();
    if(inErrorZone){
      count++;
      if(count >= 3){
        isFinished = true;
      }
    }
    else{
      count = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    PIDshootingRotate.turretController.disable();
  }
}
