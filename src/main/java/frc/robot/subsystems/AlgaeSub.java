// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSub extends SubsystemBase {

  SparkMax groundIntakeRotate = new SparkMax(15, MotorType.kBrushless);
  TalonFX groundIntake = new TalonFX(11);
  /** Creates a new AlgaeSub. */
  public AlgaeSub() {}

  public void setRotate(double power) {
    groundIntakeRotate.set(power);
  }

  public void setGroundIntake(double power) {
    groundIntake.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
