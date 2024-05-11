package frc.robot.drivetrain

class DrivetrainConstants {
  companion object {
    private const val COUNTS_PER_REVOLUTION = 1440.0
    const val WHEEL_DIAMETER_METERS = 0.07
    const val DISTANCE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS / COUNTS_PER_REVOLUTION

    const val MAX_LINEAR_VELOCITY = 0.63

    const val LEFT_MOTOR_PWM = 0
    const val RIGHT_MOTOR_PWM = 1

    val LEFT_ENCODER = (4 to 5)
    val RIGHT_ENCODER = (6 to 7)
  }
}