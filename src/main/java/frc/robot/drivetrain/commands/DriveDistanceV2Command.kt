package frc.robot.drivetrain.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.drivetrain.DrivetrainSubsystem

class DriveDistanceV2Command(private val drivetrain: DrivetrainSubsystem) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetDriveEncoders()
    drivetrain.resetLoopState()
  }

  override fun execute() = drivetrain.setDesiredVoltsV2(drivetrain.getDistanceSetpoint())

  override fun isFinished() = drivetrain.atDesiredDistanceV2()

  override fun end(interrupted: Boolean) = drivetrain.stop()
}