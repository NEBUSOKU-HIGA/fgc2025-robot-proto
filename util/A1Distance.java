package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class A1Distance {
	private final DistanceSensor distanceSensor;
	private boolean isConnected = false;
	private static final double WARNING_DISTANCE_CM = 30.0; // 30cmで警告
	private static final double CRITICAL_DISTANCE_CM = 15.0; // 15cmで危険

	public A1Distance(HardwareMap hw) {
		// I2CBus0のdistanceSensorを初期化
		DistanceSensor tempSensor = null;
		try {
			tempSensor = hw.get(DistanceSensor.class, "distanceSensor");
			if (tempSensor != null) {
				System.out.println("A1Distance - Distance sensor initialized successfully on I2CBus0");
			} else {
				System.out.println("A1Distance - Distance sensor not found on I2CBus0");
			}
		} catch (Exception e) {
			tempSensor = null;
			System.out.println("A1Distance - Error initializing distance sensor: " + e.getMessage());
		}
		
		// finalフィールドに一度だけ代入
		this.distanceSensor = tempSensor;
		this.isConnected = (tempSensor != null);
	}

	public double readDistanceCm() {
		if (distanceSensor != null && isConnected) {
			try {
				return distanceSensor.getDistance(DistanceUnit.CM);
			} catch (Exception e) {
				System.out.println("A1Distance - Error reading distance: " + e.getMessage());
				return -1.0; // エラーの場合は-1を返す
			}
		}
		return -1.0;
	}

	public double readDistanceInches() {
		if (distanceSensor != null && isConnected) {
			try {
				return distanceSensor.getDistance(DistanceUnit.INCH);
			} catch (Exception e) {
				System.out.println("A1Distance - Error reading distance in inches: " + e.getMessage());
				return -1.0;
			}
		}
		return -1.0;
	}

	public double readDistanceMm() {
		if (distanceSensor != null && isConnected) {
			try {
				return distanceSensor.getDistance(DistanceUnit.MM);
			} catch (Exception e) {
				System.out.println("A1Distance - Error reading distance in mm: " + e.getMessage());
				return -1.0;
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

	public boolean isDistanceSensorConnected() {
		return isConnected;
	}

	public boolean isObjectClose(double thresholdCm) {
		double distance = readDistanceCm();
		return distance >= 0 && distance < thresholdCm;
	}

	public boolean isObjectVeryClose() {
		return isObjectClose(CRITICAL_DISTANCE_CM);
	}

	public boolean isObjectClose() {
		return isObjectClose(WARNING_DISTANCE_CM);
	}

	public double getWarningDistance() {
		return WARNING_DISTANCE_CM;
	}

	public double getCriticalDistance() {
		return CRITICAL_DISTANCE_CM;
	}

	public String getStatusMessage() {
		if (!isConnected) {
			return "Sensor not connected";
		}
		
		double distance = readDistanceCm();
		if (distance < 0) {
			return "Reading error";
		}
		
		String warningLevel = getWarningLevel(distance);
		switch (warningLevel) {
			case "CRITICAL":
				return String.format("CRITICAL: %.1f cm", distance);
			case "WARNING":
				return String.format("WARNING: %.1f cm", distance);
			case "SAFE":
				return String.format("SAFE: %.1f cm", distance);
			default:
				return String.format("ERROR: %.1f cm", distance);
		}
	}
}
