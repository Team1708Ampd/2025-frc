// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSub extends SubsystemBase {

  SparkMax climber;

  /** Creates a new ClimberSub. */
  public ClimberSub() {
    climber = new SparkMax(14, MotorType.kBrushless);
  }

  public void setPower(double power) {
      climber.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
