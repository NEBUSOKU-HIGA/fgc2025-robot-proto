package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TopMotorSubsystem {
	private final DcMotorEx topMotor;

	public TopMotorSubsystem(HardwareMap hw) {
		// トップモーターの初期化
		topMotor = hw.get(DcMotorEx.class, "top_motor");

		// モーターの設定
		if (topMotor != null) {
			topMotor.setDirection(DcMotor.Direction.REVERSE);
			topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}
	}

	public void moveTop(double power) {
		if (topMotor == null) return;
		
		// デッドゾーン設定（0.1以下は無視）
		if (power < 0.1) {
			topMotor.setPower(0);
		} else {
			// R2 trigger値をモーターパワーとして使用
			topMotor.setPower(power);
		}
	}
}