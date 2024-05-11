package frc.robot.drivetrain

import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface DrivetrainIO {
  class DrivetrainIOInputs : LoggableInputs {
    var leftSpeed = 0.0
    var leftDistance = 0.0
    var leftVelocity = 0.0
    var leftInverted = false

    var rightSpeed = 0.0
    var rightDistance = 0.0
    var rightVelocity = 0.0
    var rightInverted = false

    override fun toLog(table: LogTable) {
      table.put("Left Speed", leftSpeed)
      table.put("Left Distance", leftDistance)
      table.put("Left Velocity", leftVelocity)
      table.put("Left Inverted", leftInverted)

      table.put("Right Speed", rightSpeed)
      table.put("Right Distance", rightDistance)
      table.put("Right Velocity", rightVelocity)
      table.put("Right Inverted", rightInverted)
    }

    override fun fromLog(table: LogTable) {
      leftSpeed = table.get("Left Speed", leftSpeed)
      leftDistance = table.get("Left Distance", leftDistance)
      leftVelocity = table.get("Left Velocity", leftVelocity)
      leftInverted = table.get("Left Inverted", leftInverted)

      rightSpeed = table.get("Right Speed", rightSpeed)
      rightDistance = table.get("Right Distance", rightDistance)
      rightVelocity = table.get("Right Velocity", rightVelocity)
      rightInverted = table.get("Right Inverted", rightInverted)
    }
  }

  fun updateInputs(inputs: DrivetrainIOInputs)

  fun setLeftSpeed(speed: Double)

  fun setLeftVolts(volts: Double)

  fun setRightSpeed(speed: Double)

  fun setRightVolts(volts: Double)

  fun tankDrive(leftSpeed: Double, rightSpeed: Double)

  fun resetDriveEncoders()
}