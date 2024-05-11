package frc.robot.drivetrain.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.drivetrain.DrivetrainSubsystem

class TankDriveCommand(private val drivetrain: DrivetrainSubsystem, private val controller: CommandPS5Controller) : Command() {
  init {
    addRequirements(drivetrain)
  }

  override fun execute() {
    drivetrain.tankDrive(-controller.leftY, -controller.rightY)
  }
}