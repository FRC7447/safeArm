// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;

public class extendTelescope extends CommandBase {
  Telescope m_telescope;
  boolean finish;
  /** Creates a new extendTelescope. */
  public extendTelescope(Telescope t) {
    m_telescope = t;
    addRequirements(m_telescope);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_telescope.getTopSwitch()) {
      m_telescope.stopTelescope();
      finish = true;
    }
    else {
      m_telescope.moveTelescope(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescope.stopTelescope();
    finish = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
