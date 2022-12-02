// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IndexTwo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_subsystem;
  private final BallIntake m_subsystem_1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexTwo(Elevator subsystem, BallIntake subsystem1) {
    m_subsystem = subsystem;
    m_subsystem_1 = subsystem1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.moveUp2();
    m_subsystem.moveUp3();
    m_subsystem_1.ballTake();
    m_subsystem.moveUp();
    //if(!RobotContainer.getBallLimitSwitch()){
      
    }
   //ds or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop2();
    m_subsystem_1.stop();
    m_subsystem.stop3();
    m_subsystem.stop1();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
