// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBase extends SubsystemBase {
  private Swerve m_drive;
  private Limelight m_limeLight;
  private Pigeon m_pigeon;

  public DriveBase(Swerve drive, Limelight limeLight, Pigeon pigeon) {
    m_drive = drive;
    m_limeLight = limeLight;
    m_pigeon = pigeon;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
