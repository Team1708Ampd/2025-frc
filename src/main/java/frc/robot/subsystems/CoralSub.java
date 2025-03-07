// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSub extends SubsystemBase {
  /** Creates a new CoralSub. */
  public TalonFX leftElevator = new TalonFX(9);
  public TalonFX rightElevator = new TalonFX(8);
  TalonFX intake = new TalonFX(10);
  public TalonFX wrist = new TalonFX(12);
  public DigitalInput beam = new DigitalInput(2);
  SparkMax leftIntake = new SparkMax(15, MotorType.kBrushless);
  SparkMax rightIntake = new SparkMax(23, MotorType.kBrushless);

  public DigitalInput wristSwitch = new DigitalInput(4);
  public DigitalInput elevatorSwitch = new DigitalInput(7);

  public CoralSub() {
    wrist.setNeutralMode(NeutralModeValue.Brake);
    leftElevator.setNeutralMode(NeutralModeValue.Brake);
    rightElevator.setNeutralMode(NeutralModeValue.Brake);
    intake.setNeutralMode(NeutralModeValue.Brake);
  }

  public boolean getBeam() {
    return beam.get();
  }

  public void setElevator(double power) {
    leftElevator.set(power);
    rightElevator.set(power);
  }

  public void setIntake(double power) {
    intake.set(power);
    leftIntake.set(-power);
    rightIntake.set(power);
  }

  public void setWrist(double power) {
    wrist.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
