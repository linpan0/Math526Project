package frc.robot.drivetrain

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.drivetrain.commands.TankDriveCommand
import frc.robot.drivetrain.gyro.GyroIO
import org.littletonrobotics.junction.Logger

class DrivetrainSubsystem(private val drivetrainIO: DrivetrainIO, private val gyroIO: GyroIO) : SubsystemBase() {
  private val drivetrainInputs = DrivetrainIO.DrivetrainIOInputs()
  private val gyroInputs = GyroIO.GyroIOInputs()

  init {
    resetDriveEncoders()
  }

//  private val test = LoggedTunableNumber("test", 25.0)
//  private val test2 = LoggedTunableNumber("test2", 5.0)

  override fun periodic() {
//    LoggedTunableNumber.ifChanged(test.get().toInt() + test2.get().toInt(), { a: DoubleArray -> println(test.get()); println(a.contentToString()) }, test, test2)

    drivetrainIO.updateInputs(drivetrainInputs)
    gyroIO.updateInputs(gyroInputs)

    Logger.processInputs("Drivetrain", drivetrainInputs)
    Logger.processInputs("Gyro", gyroInputs)
  }

  fun tankDrive(leftSpeed: Double, rightSpeed: Double) = drivetrainIO.tankDrive(leftSpeed, rightSpeed)

  fun setVolts(leftVolts: Double, rightVolts: Double) {
    drivetrainIO.setLeftVolts(leftVolts)
    drivetrainIO.setRightVolts(rightVolts)
  }

  fun stop() = setVolts(0.0, 0.0)

  fun getLeftDistance() = drivetrainInputs.leftDistance

  fun getRightDistance() = drivetrainInputs.rightDistance

  fun resetGyro() = gyroIO.reset()

  fun resetDriveEncoders() = drivetrainIO.resetDriveEncoders()

  fun setDefaultCommand(controller: CommandPS5Controller) {
    defaultCommand = TankDriveCommand(this, controller)
  }
}