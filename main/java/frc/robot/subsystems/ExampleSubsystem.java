// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {}

  // Motors
  private final TalonFX motor = new TalonFX(7);
  public void runMotor(float power) {
    if (power > 1.0 || power < 0.0) return;
    System.out.println("Running motor at " + power * 100 + "% power...");
    motor.set(TalonFXControlMode.PercentOutput, 0.5); // runs the motor at 50% power
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
