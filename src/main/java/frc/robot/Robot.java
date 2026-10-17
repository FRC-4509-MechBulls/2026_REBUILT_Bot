// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.HubShiftUtil;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged private final RobotContainer m_robotContainer;
  private Optional<Alliance> lastAlliance = Optional.empty();


  public Robot() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    m_robotContainer = new RobotContainer();
    Epilogue.configure(config -> config.backend = new FileBackend(DataLogManager.getLog()));
    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    m_robotContainer.updateDriveInputs();
    CommandScheduler.getInstance().run();
    m_robotContainer.updateRumble();
    m_robotContainer.updateAlerts();
    m_robotContainer.publishSelectedAuto();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getShiftInfo();
    SmartDashboard.putBoolean("HubActive", shiftInfo.active());
    SmartDashboard.putNumber("ShiftTimeLeft", Math.max(shiftInfo.remainingTime(), 0));
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();

    if(currentAlliance.isPresent() && !currentAlliance.equals(lastAlliance)) {
      if(currentAlliance.get() == Alliance.Red) {
        m_robotContainer.updateControlsForAlliance(Alliance.Red);
      } else if (currentAlliance.get() == Alliance.Blue) {
        m_robotContainer.updateControlsForAlliance(Alliance.Blue);
      }
      lastAlliance = currentAlliance;
    }

  }

  @Override
  public void disabledInit() {
    m_robotContainer.stateController.resetState();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.previewStartingPose();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    HubShiftUtil.initialize();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    HubShiftUtil.initialize();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.stateController.resetState();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
