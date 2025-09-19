package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem {
	private final DistanceSensor distanceSensor;
	private static final double WARNING_DISTANCE_CM = 30.0; // 30cmで警告
	private static final double CRITICAL_DISTANCE_CM = 15.0; // 15cmで危険

	public DistanceSensorSubsystem(HardwareMap hw) {
		// 距離センサーの初期化 initialization
		distanceSensor = hw.get(DistanceSensor.class, "distanceSensor");
	}

	public double readDistanceCm() {
		if (distanceSensor != null) {
			try {
				return distanceSensor.getDistance(DistanceUnit.CM);
			} catch (Exception e) {
				return -1.0; // エラーの場合は-1を返す
			}
		}
		return -1.0;
	}

	public String getWarningLevel(double cm) {
		if (cm < 0) {
			return "ERROR";
		} else if (cm < CRITICAL_DISTANCE_CM) {
			return "CRITICAL";
		} else if (cm < WARNING_DISTANCE_CM) {
			return "WARNING";
		} else {
			return "SAFE";
		}
	}
}
