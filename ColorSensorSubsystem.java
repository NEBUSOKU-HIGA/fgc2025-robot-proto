package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorSubsystem {
	private final ColorSensor colorSensor;

	public ColorSensorSubsystem(HardwareMap hw) {
		// カラーセンサーの初期化
		colorSensor = hw.get(ColorSensor.class, "colorSensor");
		
		// LEDを有効化
		if (colorSensor != null) {
			colorSensor.enableLed(true);
		}
	}

	public int[] readRGB() {
		if (colorSensor != null) {
			return new int[]{
				colorSensor.red(),
				colorSensor.green(),
				colorSensor.blue(),
				colorSensor.alpha()
			};
		}
		return new int[]{0, 0, 0, 0};
	}

	public String readColorName() {
		if (colorSensor == null) {
			return "NO_SENSOR";
		}

		int[] rgb = readRGB();
		int r = rgb[0];
		int g = rgb[1];
		int b = rgb[2];

		// HSVに変換
		float[] hsv = new float[3];
		android.graphics.Color.RGBToHSV(r, g, b, hsv);
		
		float hue = hsv[0];
		float saturation = hsv[1];
		float value = hsv[2];

		// 色名の判定
		if (value < 0.1) {
			return "BLACK";
		} else if (saturation < 0.2) {
			return "WHITE";
		} else if (hue < 30 || hue >= 330) {
			return "RED";
		} else if (hue < 90) {
			return "YELLOW";
		} else if (hue < 150) {
			return "GREEN";
		} else if (hue < 210) {
			return "CYAN";
		} else if (hue < 270) {
			return "BLUE";
		} else if (hue < 330) {
			return "MAGENTA";
		} else {
			return "UNKNOWN";
		}
	}
}