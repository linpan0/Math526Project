package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.drivetrain.DrivetrainConstants
import frc.robot.drivetrain.DrivetrainIORomi
import frc.robot.drivetrain.DrivetrainSubsystem
import frc.robot.drivetrain.commands.DriveDistanceV1Command
import frc.robot.drivetrain.commands.DriveDistanceV2Command
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
    SmartDashboard.putData("DriveDistanceV1", DriveDistanceV1Command(drivetrain))
    SmartDashboard.putData("DriveDistanceV2", DriveDistanceV2Command(drivetrain))
  }
}