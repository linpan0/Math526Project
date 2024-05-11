package frc.robot.drivetrain.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.drivetrain.DrivetrainSubsystem

class DriveDistanceV1Command(private val drivetrain: DrivetrainSubsystem) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetDriveEncoders()
    drivetrain.resetDistancePID()
  }

  override fun execute() = drivetrain.setDesiredVoltsV1(drivetrain.getDistanceSetpoint())

  override fun isFinished() = drivetrain.atDesiredDistanceV1()

  override fun end(interrupted: Boolean) = drivetrain.stop()
}