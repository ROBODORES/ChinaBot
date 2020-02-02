/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Spinner;
import edu.wpi.first.wpilibj.DriverStation;

public class Spin extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Spinner m_spinner;
  String gameData = null;
  /**
   * Creates a new Spin.
   */
  public Spin(Spinner spinner) {
    m_spinner = spinner;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    m_spinner.setSpinnerSolenoid(true);
    m_spinner.spinTime(0.2);
    if(gameData.length() > 0)
    {
    switch (gameData.charAt(0))
    {
    case 'B' :
      //Blue case code
      if(m_spinner.getCurrentColor() == m_spinner.getBlue()){
        m_spinner.spinTime(0.0);
      }
      break;
    case 'G' :
      //Green case code
      if(m_spinner.getCurrentColor() == m_spinner.getGreen()){
        m_spinner.spinTime(0.0);
      }
      break;
    case 'R' :
      //Red case code
      if(m_spinner.getCurrentColor() == m_spinner.getRed()){
        m_spinner.spinTime(0.0);
      }
      break;
    case 'Y' :
      //Yellow case code
      if(m_spinner.getCurrentColor() == m_spinner.getYellow()){
        m_spinner.spinTime(0.0);
      }
      break;
    default :
      //This is corrupt data
      break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_spinner.spinTime(0);
    m_spinner.setSpinnerSolenoid(false);
    return false;
  }
}
