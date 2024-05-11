package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.drivetrain.DrivetrainConstants
import frc.robot.drivetrain.DrivetrainIORomi
import frc.robot.drivetrain.DrivetrainSubsystem
import frc.robot.drivetrain.commands.DriveDistanceV1Command
import frc.robot.drivetrain.gyro.GyroIORomi

object RobotContainer {
  val drivetrain = DrivetrainSubsystem(DrivetrainIORomi(
    DrivetrainConstants.LEFT_MOTOR_PWM,
    DrivetrainConstants.RIGHT_MOTOR_PWM,
    DrivetrainConstants.LEFT_ENCODER,
    DrivetrainConstants.RIGHT_ENCODER
  ), GyroIORomi())

  val controller = CommandPS5Controller(GlobalConstants.CONTROLLER_PORT)

  init {
    configureButtonBindings()

    DriverStation.silenceJoystickConnectionWarning(true)
  }

  private fun configureButtonBindings() {
    SmartDashboard.putData("Drive Distance", DriveDistanceV1Command(1.0, drivetrain))
  }
}