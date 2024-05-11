package frc.robot.drivetrain

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.romi.RomiMotor

class DrivetrainIORomi(leftPWM: Int, rightPWM: Int, leftEncoderID: Pair<Int, Int>, rightEncoderID: Pair<Int, Int>) : DrivetrainIO {
  private val leftMotor = RomiMotor(leftPWM)
  private val rightMotor = RomiMotor(rightPWM)

  private val leftEncoder = Encoder(leftEncoderID.first, leftEncoderID.second)
  private val rightEncoder = Encoder(rightEncoderID.first, rightEncoderID.second)

  private val diffDrive = DifferentialDrive(leftMotor, rightMotor)

  init {
    leftEncoder.distancePerPulse = DrivetrainConstants.DISTANCE_CONVERSION_FACTOR
    rightEncoder.distancePerPulse = DrivetrainConstants.DISTANCE_CONVERSION_FACTOR

    rightMotor.inverted = true

    diffDrive.isSafetyEnabled = false
  }

  override fun updateInputs(inputs: DrivetrainIO.DrivetrainIOInputs) {
    inputs.leftSpeed = leftMotor.get()
    inputs.leftDistance = leftEncoder.distance
    inputs.leftVelocity = leftEncoder.rate
    inputs.leftInverted = leftMotor.inverted

    inputs.rightSpeed = rightMotor.get()
    inputs.rightDistance = rightEncoder.distance
    inputs.rightVelocity = rightEncoder.rate
    inputs.rightInverted = rightMotor.inverted
  }

  override fun setLeftSpeed(speed: Double) = leftMotor.set(speed)

  override fun setLeftVolts(volts: Double) = leftMotor.setVoltage(volts)

  override fun setRightSpeed(speed: Double) = rightMotor.set(speed)

  override fun setRightVolts(volts: Double) = rightMotor.setVoltage(volts)

  override fun tankDrive(leftSpeed: Double, rightSpeed: Double) = diffDrive.tankDrive(leftSpeed, rightSpeed)

  override fun resetDriveEncoders() {
    leftEncoder.reset()
    rightEncoder.reset()
  }
}