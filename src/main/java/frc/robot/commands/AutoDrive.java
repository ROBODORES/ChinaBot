/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.controller.PIDController;

public class AutoDrive extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;
  private final Limelight m_limelight;
  private double Kp = 0.4;
  private double Ki = 0.0;
  private double Kd = 0.0;
  private double turningAdjust;
  private double input;
  private PIDController m_PIDController = new PIDController(Kp, Ki, Kd);
  /**
   * Creates a new AutoDrive.
   */
  public AutoDrive(Drivetrain drivetrain, Limelight limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PIDController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    input = m_limelight.getTx()/25;
    turningAdjust = (m_PIDController.calculate(input));
    m_drivetrain.exponentialDrive(RobotContainer.m_driverController.getRawAxis(1), -turningAdjust, 0.005, 0.5, 1.0, 1.0);
    System.out.print("Input: " + input);
    System.out.println(", turningAdjust: " +  turningAdjust);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
