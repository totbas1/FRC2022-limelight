// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BallIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.*; // flagged as unused by VSCode -PB

/** An example command that uses an example subsystem. */
public class BallIntakeCommand extends CommandBase{
  private static boolean commandCalled = false; // flagged as unused by VSCode -PB
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final BallIntake m_subsystem1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BallIntakeCommand(BallIntake subsystem) {
    m_subsystem1 = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandCalled = true;
    System.out.println("BallIntakeCommand execute.");
    //HC - I just added a boolean for this command to test shuffleboard.
    //If this is useful we can do this for the rest of the commands otherwise, we can just delete this.
    //SmartDashboard.putBoolean("BallIntake", commandCalled);
    m_subsystem1.ballTake();//rr
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandCalled = false;
    System.out.println("BallIntakeCommand end.");
    m_subsystem1.stop();//rr
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
