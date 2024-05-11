package frc.robot

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.RobotContainer.controller
import frc.robot.RobotContainer.drivetrain
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGWriter

object Robot : LoggedRobot() {
  override fun robotInit() {
    RobotContainer

    Logger.addDataReceiver(NT4Publisher())
    Logger.addDataReceiver(WPILOGWriter())
    PowerDistribution()

    Logger.start()

    CommandScheduler.getInstance().onCommandInitialize { command -> Logger.recordOutput("/ActiveCommands/${command.name}", true) }
    CommandScheduler.getInstance().onCommandFinish { command -> Logger.recordOutput("/ActiveCommands/${command.name}", false) }
    CommandScheduler.getInstance().onCommandInterrupt { command -> Logger.recordOutput("/ActiveCommands/${command.name}", false) }
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()
  }

  override fun autonomousInit() {

  }

  override fun autonomousPeriodic() {

  }

  override fun teleopInit() {
    drivetrain.setDefaultCommand(controller)
  }

  override fun teleopPeriodic() {

  }
}