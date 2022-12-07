// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.LimeLight;
import frc.robot.subsystems.Shooting;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShootingVisionCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooting m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootingVisionCommand(Shooting subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.shootTop(LimeLight.getShootSpeedValue());
    System.out.println ("ShootingVisionCommand execute.");
    SmartDashboard.putNumber("tx", LimeLight.limelightTrackingX()); //HC
    SmartDashboard.putNumber("ty", LimeLight.limelightTrackingY());
    SmartDashboard.putNumber("ta", LimeLight.limelightTrackingArea());

    //private NetworkTableEntry txEntry = Shuffleboard.getTab("SmartDashboard").add("tx", 0).getEntry();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
    System.out.println ("ShootingVisionCommand end.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
