// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSub extends SubsystemBase {
  /** Creates a new CoralSub. */
  TalonFX leftElevator = new TalonFX(9);
  TalonFX rightElevator = new TalonFX(8);
  TalonFX intake = new TalonFX(10);
  TalonFX claw = new TalonFX(12);


  public CoralSub() {
    claw.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setElevator(double power) {
    leftElevator.set(power);
    rightElevator.set(power);
  }

  public void setIntake(double power) {
    intake.set(power);
  }

  public void setClaw(double power) {
    claw.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
