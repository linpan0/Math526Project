package frc.robot.drivetrain.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.drivetrain.DrivetrainSubsystem

class DriveDistanceV1Command(private val distance: Double, private val drivetrain: DrivetrainSubsystem) : Command() {
  private val leftPID = PIDController(25.0, 0.0, 1.0).apply {
    setTolerance(0.05)
    reset()
    setpoint = distance
  }

  private val rightPID = PIDController(25.0, 0.0, 1.0).apply {
    setTolerance(0.05)
    reset()
    setpoint = distance
  }

  init {
    addRequirements(drivetrain)
  }

  override fun initialize() {
    drivetrain.resetDriveEncoders()
  }

  override fun execute() {
    val leftVolts = MathUtil.clamp(leftPID.calculate(drivetrain.getLeftDistance()), -12.0, 12.0)
    val rightVolts = MathUtil.clamp(rightPID.calculate(drivetrain.getRightDistance()), -12.0, 12.0)
    drivetrain.setVolts(leftVolts, rightVolts)
  }

  override fun isFinished() = leftPID.atSetpoint() && rightPID.atSetpoint()

  override fun end(interrupted: Boolean) = drivetrain.stop()
}