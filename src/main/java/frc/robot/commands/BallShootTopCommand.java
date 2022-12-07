// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooting;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;

/** An example command that uses an example subsystem. */
public class BallShootTopCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooting m_subsystem;

  /**
   * Creates a new BallShootTopCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BallShootTopCommand(Shooting subsystem) {
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
    System.out.println("BallShootTopCommand execute.");
    m_subsystem.shootTop(LimeLight.calcSpeed()/*.65[p7lk*/); 
    //idea: write code in ////RobotContainer that gets vision value. Rahul Rajaram 1/11/2022 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("BallShootTopCommand end.");
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
