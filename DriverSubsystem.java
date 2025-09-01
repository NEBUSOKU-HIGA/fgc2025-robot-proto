package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriverSubsystem {
	private final DcMotor leftMotor;
	private final DcMotor rightMotor;

	public DriverSubsystem(HardwareMap hw) {
		// モーターの初期化
		leftMotor = hw.get(DcMotor.class, "left_motor");
		rightMotor = hw.get(DcMotor.class, "right_motor");

		// モーターの設定
		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public void drive(double forward, boolean turnLeft, boolean turnRight) {
		double leftPower = 0.0;
		double rightPower = 0.0;

		// 前後運動制御 (Right Joystick Y-axis) - まっすぐ進む
		if (Math.abs(forward) >= 0.1) {
			leftPower = forward;   // 左モーター正方向
			rightPower = forward;  // 右モーター正方向
		}

		// 回転制御 (L1/R1 buttons)
		if (turnLeft) {
			// 右回転 (L1)
			leftPower += 0.5;
			rightPower -= 0.5;
		}

		if (turnRight) {
			// 左回転 (R1)
			leftPower -= 0.5;
			rightPower += 0.5;
		}

		// パワー値を-1.0から1.0の範囲に制限
		leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
		rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

		// モーターにパワーを設定
		leftMotor.setPower(leftPower);
		rightMotor.setPower(rightPower);
	}
}