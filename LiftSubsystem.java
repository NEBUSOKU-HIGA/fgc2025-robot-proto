package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem {
	private final DcMotor liftMotor;

	public LiftSubsystem(HardwareMap hw) {
		// リフトモーターの初期化
		liftMotor = hw.get(DcMotor.class, "liftmotor");

		// モーターの設定
		if (liftMotor != null) {
			liftMotor.setDirection(DcMotor.Direction.REVERSE);
			liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}
	}

	public void moveLift(double power) {
		if (liftMotor != null) {
			// デッドゾーン設定（0.1以下は無視）
			if (Math.abs(power) < 0.1) {
				liftMotor.setPower(0.0);
			} else {
				// 左Joystick Y軸の値をモーターパワーとして使用
				liftMotor.setPower(power);
			}
		}
	}
}