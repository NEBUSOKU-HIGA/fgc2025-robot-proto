package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class A1Color {
	private final ColorSensor colorSensor1;  // I2C Bus1
	private final ColorSensor colorSensor2;  // I2C Bus2
	private boolean isSensor1Connected = false;
	private boolean isSensor2Connected = false;

	public A1Color(HardwareMap hw) {
		// I2CBus1のcolorSensorを初期化
		ColorSensor tempSensor1 = null;
		try {
			tempSensor1 = hw.get(ColorSensor.class, "colorSensor");
			if (tempSensor1 != null) {
				tempSensor1.enableLed(true);
				System.out.println("A1Color - Color sensor 1 initialized successfully on I2CBus1");
				isSensor1Connected = true;
			} else {
				System.out.println("A1Color - Color sensor 1 not found on I2CBus1");
			}
		} catch (Exception e) {
			System.out.println("A1Color - Error initializing color sensor 1: " + e.getMessage());
		}
		
		// I2CBus2のcolorSensor2を初期化
		ColorSensor tempSensor2 = null;
		try {
			tempSensor2 = hw.get(ColorSensor.class, "colorSensor2");
			if (tempSensor2 != null) {
				tempSensor2.enableLed(true);
				System.out.println("A1Color - Color sensor 2 initialized successfully on I2CBus2");
				isSensor2Connected = true;
			} else {
				System.out.println("A1Color - Color sensor 2 not found on I2CBus2");
			}
		} catch (Exception e) {
			System.out.println("A1Color - Error initializing color sensor 2: " + e.getMessage());
		}
		
		// finalフィールドに一度だけ代入
		this.colorSensor1 = tempSensor1;
		this.colorSensor2 = tempSensor2;
	}

	public int[] readRGB1() {
		if (colorSensor1 != null && isSensor1Connected) {
			try {
				return new int[]{
					colorSensor1.red(),
					colorSensor1.green(),
					colorSensor1.blue(),
					colorSensor1.alpha()
				};
			} catch (Exception e) {
				System.out.println("A1Color - Error reading RGB values from sensor 1: " + e.getMessage());
				return new int[]{0, 0, 0, 0};
			}
		}
		return new int[]{0, 0, 0, 0};
	}

	public String readColorName1() {
		if (!isSensor1Connected || colorSensor1 == null) {
			return "NO_SENSOR";
		}

		try {
			int[] rgb = readRGB1();
			int r = rgb[0];
			int g = rgb[1];
			int b = rgb[2];

			// シンプルな色判定ロジック
			if (r < 50 && g < 50 && b < 50) {
				return "BLACK";
			} else if (r > 200 && g > 200 && b > 200) {
				return "WHITE";
			} else if (r > g && r > b && r > 100) {
				return "RED";
			} else if (g > r && g > b && g > 100) {
				return "GREEN";
			} else if (b > r && b > g && b > 100) {
				return "BLUE";
			} else if (r > 100 && g > 100 && b < 100) {
				return "YELLOW";
			} else if (r > 100 && b > 100 && g < 100) {
				return "MAGENTA";
			} else if (g > 100 && b > 100 && r < 100) {
				return "CYAN";
			} else {
				return "UNKNOWN";
			}
		} catch (Exception e) {
			System.out.println("A1Color - Error determining color name from sensor 1: " + e.getMessage());
			return "ERROR";
		}
	}

	public boolean isColorSensor1Connected() {
		return isSensor1Connected;
	}

	public void enableLed1(boolean enable) {
		if (colorSensor1 != null && isSensor1Connected) {
			try {
				colorSensor1.enableLed(enable);
			} catch (Exception e) {
				System.out.println("A1Color - Error setting LED for sensor 1: " + e.getMessage());
			}
		}
	}

	public int getRed1() {
		if (colorSensor1 != null && isSensor1Connected) {
			try {
				return colorSensor1.red();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading red from sensor 1: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	public int getGreen1() {
		if (colorSensor1 != null && isSensor1Connected) {
			try {
				return colorSensor1.green();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading green from sensor 1: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	public int getBlue1() {
		if (colorSensor1 != null && isSensor1Connected) {
			try {
				return colorSensor1.blue();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading blue from sensor 1: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	public int getAlpha1() {
		if (colorSensor1 != null && isSensor1Connected) {
			try {
				return colorSensor1.alpha();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading alpha from sensor 1: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	// Sensor 2 (I2C Bus2) のメソッド
	public int[] readRGB2() {
		if (colorSensor2 != null && isSensor2Connected) {
			try {
				return new int[]{
					colorSensor2.red(),
					colorSensor2.green(),
					colorSensor2.blue(),
					colorSensor2.alpha()
				};
			} catch (Exception e) {
				System.out.println("A1Color - Error reading RGB values from sensor 2: " + e.getMessage());
				return new int[]{0, 0, 0, 0};
			}
		}
		return new int[]{0, 0, 0, 0};
	}

	public String readColorName2() {
		if (!isSensor2Connected || colorSensor2 == null) {
			return "NO_SENSOR";
		}

		try {
			int[] rgb = readRGB2();
			int r = rgb[0];
			int g = rgb[1];
			int b = rgb[2];

			// シンプルな色判定ロジック
			if (r < 50 && g < 50 && b < 50) {
				return "BLACK";
			} else if (r > 200 && g > 200 && b > 200) {
				return "WHITE";
			} else if (r > g && r > b && r > 100) {
				return "RED";
			} else if (g > r && g > b && g > 100) {
				return "GREEN";
			} else if (b > r && b > g && b > 100) {
				return "BLUE";
			} else if (r > 100 && g > 100 && b < 100) {
				return "YELLOW";
			} else if (r > 100 && b > 100 && g < 100) {
				return "MAGENTA";
			} else if (g > 100 && b > 100 && r < 100) {
				return "CYAN";
			} else {
				return "UNKNOWN";
			}
		} catch (Exception e) {
			System.out.println("A1Color - Error determining color name from sensor 2: " + e.getMessage());
			return "ERROR";
		}
	}

	public boolean isColorSensor2Connected() {
		return isSensor2Connected;
	}

	public void enableLed2(boolean enable) {
		if (colorSensor2 != null && isSensor2Connected) {
			try {
				colorSensor2.enableLed(enable);
			} catch (Exception e) {
				System.out.println("A1Color - Error setting LED for sensor 2: " + e.getMessage());
			}
		}
	}

	public int getRed2() {
		if (colorSensor2 != null && isSensor2Connected) {
			try {
				return colorSensor2.red();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading red from sensor 2: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	public int getGreen2() {
		if (colorSensor2 != null && isSensor2Connected) {
			try {
				return colorSensor2.green();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading green from sensor 2: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	public int getBlue2() {
		if (colorSensor2 != null && isSensor2Connected) {
			try {
				return colorSensor2.blue();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading blue from sensor 2: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	public int getAlpha2() {
		if (colorSensor2 != null && isSensor2Connected) {
			try {
				return colorSensor2.alpha();
			} catch (Exception e) {
				System.out.println("A1Color - Error reading alpha from sensor 2: " + e.getMessage());
				return 0;
			}
		}
		return 0;
	}

	// 後方互換性のためのメソッド
	public boolean isColorSensorConnected() {
		return isSensor1Connected || isSensor2Connected;
	}

	public int[] readRGB() {
		return readRGB1(); // デフォルトでSensor 1を使用
	}

	public String readColorName() {
		return readColorName1(); // デフォルトでSensor 1を使用
	}

	public void enableLed(boolean enable) {
		enableLed1(enable); // デフォルトでSensor 1を使用
	}
}
