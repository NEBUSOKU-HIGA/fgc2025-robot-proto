package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.A1Drive;
import org.firstinspires.ftc.teamcode.A1Flask;
import org.firstinspires.ftc.teamcode.A1Arm;
import org.firstinspires.ftc.teamcode.A1Lift;
import org.firstinspires.ftc.teamcode.A1Potension;
import org.firstinspires.ftc.teamcode.A1Servo;
import org.firstinspires.ftc.teamcode.A1Top2;
import org.firstinspires.ftc.teamcode.A1Imu;
import org.firstinspires.ftc.teamcode.A1ImuMotor;

@TeleOp(name="A1Task", group="Test")
public class A1Task extends LinearOpMode {

	// A1 system instances
	private A1ImuMotor imuMotorSystem;
	private A1Flask flaskSystem = null; // Flask is optional
	private A1Arm armSystem;
	private A1Lift liftSystem;
	private A1Potension potensionSystem;
	private A1Servo servoSystem;
	private A1Top2 topSystem;
	
	// Button state (debounce)
	private boolean lastShareButton = false;
	private boolean lastOptionsButton = false;
	
	// Triangle forward speed control
	private double triangleForwardSpeed = 0.3; // Default forward speed (0.0–1.0)
	private static final double TRIANGLE_SPEED_INCREMENT = 0.1; // Increment step
	
	// R1 arm position control
	private double r1ArmTargetHeight = 0.55; // Default arm target height (0.0–1.0)
	private static final double R1_ARM_HEIGHT_INCREMENT = 0.05; // Increment step
	
	// Arm gravity compensation (lift hold power)
	private double armHoldPower = -0.15; // Lift motor power to hold arm (default -0.15)
	private static final double ARM_HOLD_POWER_INCREMENT = 0.02; // Increment step
	
	// R2-button lift control (simulate left-stick Y)
	private double r2LiftValue = -0.55; // Default simulated left-stick Y for R2
	private static final double R2_LIFT_VALUE_INCREMENT = 0.05; // Increment step
	
	// Nonlinear arm hold tuning
	private double armHoldMaxPower = -0.15; // Max output for arm position hold (default -0.15 = 15%)
	private static final double ARM_HOLD_MAX_POWER_INCREMENT = 0.02; // Increment step
	private static final double ARM_HOLD_MIN_HEIGHT = 0.34; // Start hold at this height
	private static final double ARM_HOLD_MAX_HEIGHT = 0.56; // Max hold output above this height
	
	// R2 → delayed servo action
	private long r2ServoTimer = 0; // R2 servo delay timer
	private boolean r2ServoTriggered = false; // R2 servo trigger state
	
	// Track Servo1 position changes
	private double lastServo1Position = 0.0;
	
	/**
	 * Arm position hold (nonlinear): prevents arm from sagging due to gravity by driving the lift.
	 * @param currentHeight current normalized arm height (potentiometer)
	 * @return lift_motor output required to hold position
	 */
	private double calculateArmHoldPower(double currentHeight) {
		// Below min height: no hold needed
		if (currentHeight < ARM_HOLD_MIN_HEIGHT) {
			return 0.0;
		}
		
		// Above max height: use max hold power
		if (currentHeight >= ARM_HOLD_MAX_HEIGHT) {
			return armHoldMaxPower;
		}
		
		// Between min and max: nonlinear curve
		double heightRatio = (currentHeight - ARM_HOLD_MIN_HEIGHT) / (ARM_HOLD_MAX_HEIGHT - ARM_HOLD_MIN_HEIGHT);
		
		// Nonlinear curve: sqrt to give more power at lower heights
		double nonLinearRatio = Math.sqrt(heightRatio);
		
		// Extra bump near ~0.42 height to counterbalance geometry
		double targetHeight = 0.42;
		double targetRatio = (targetHeight - ARM_HOLD_MIN_HEIGHT) / (ARM_HOLD_MAX_HEIGHT - ARM_HOLD_MIN_HEIGHT);
		
		if (Math.abs(heightRatio - targetRatio) < 0.1) {
			// +30% in this band
			nonLinearRatio *= 1.3;
		}
		
		double holdPower = armHoldMaxPower * nonLinearRatio;
		return holdPower;
	}
	

	@Override
	public void runOpMode() {
		// Initialize A1 subsystems
		imuMotorSystem = new A1ImuMotor(hardwareMap);
		
		// Initialize Flask (optional)
		try {
			flaskSystem = new A1Flask();
			System.out.println("A1Task - Flask system initialized successfully");
		} catch (Exception e) {
			flaskSystem = null;
			System.out.println("A1Task - Flask system initialization failed: " + e.getMessage());
			System.out.println("A1Task - Robot will continue without Flask functionality");
		}
		
		armSystem = new A1Arm(hardwareMap);
		liftSystem = new A1Lift(hardwareMap);
		potensionSystem = new A1Potension(hardwareMap);
		servoSystem = new A1Servo(hardwareMap);
		topSystem = new A1Top2(hardwareMap);
		
		// Wait for start
		waitForStart();

		while (opModeIsActive()) {
			// Read gamepad inputs
			double leftStickX = gamepad1.left_stick_x;
			double leftStickY = gamepad1.left_stick_y;
			double rightStickX = gamepad1.right_stick_x;
			double rightStickY = gamepad1.right_stick_y;
			
			double leftTrigger = gamepad1.left_trigger;
			double rightTrigger = gamepad1.right_trigger;
			
			boolean leftBumper = gamepad1.left_bumper;
			boolean rightBumper = gamepad1.right_bumper;
			
			boolean aButton = gamepad1.a;
			boolean bButton = gamepad1.b;
			boolean xButton = gamepad1.x;
			boolean yButton = gamepad1.y;
			
			boolean dpadUp = gamepad1.dpad_up;
			boolean dpadDown = gamepad1.dpad_down;
			boolean dpadLeft = gamepad1.dpad_left;
			boolean dpadRight = gamepad1.dpad_right;

			// Update IMU data and run drive control
			imuMotorSystem.updateImuData();
			
			// Cancel auto heading if manual input exists
			boolean hasManualInput = (Math.abs(rightStickY) > 0.1) || (Math.abs(rightStickX) > 0.15) || leftBumper || rightBumper;
			imuMotorSystem.cancelAutoTurn(hasManualInput);
			
			// Get auto-turn control
			double[] autoTurnControl = imuMotorSystem.calculateAutoTurnControl();
			double autoTurnLeftPower = autoTurnControl[0];
			double autoTurnRightPower = autoTurnControl[1];
			
			// If auto-turn is active, use its control; else do manual drive
			if (imuMotorSystem.isAutoTurnActive()) {
				if (imuMotorSystem.isLeftMotorConnected()) {
					imuMotorSystem.getLeftMotor().setPower(autoTurnLeftPower);
				}
				if (imuMotorSystem.isRightMotorConnected()) {
					imuMotorSystem.getRightMotor().setPower(autoTurnRightPower);
				}
			} else {
				// Manual drive control in A1Task
				
				// Tight deadzones
				if (Math.abs(rightStickY) < 0.05) rightStickY = 0.0;
				if (Math.abs(rightStickX) < 0.05) rightStickX = 0.0;

				double leftPower = 0.0;
				double rightPower = 0.0;

				// Forward/back (right stick Y prioritized; supports gamepad1 & gamepad2)
				if (Math.abs(rightStickY) >= 0.05) {
					// gamepad1 right stick Y has priority
					leftPower = rightStickY;	// left motor (forward = positive)
					rightPower = rightStickY;   // right motor (forward = positive)
					
					// If moving forward, run top_motor at 20%
					if (rightStickY < 0) {
						topSystem.moveTop(0.2, false);
						System.out.println("A1Task - Forward detected, Top motor 20% power");
					}
				} else if (Math.abs(gamepad2.right_stick_y) >= 0.05) {
					// If no gamepad1 input, use gamepad2 right stick Y
					double gamepad2RightStickY = gamepad2.right_stick_y;
					leftPower = gamepad2RightStickY;
					rightPower = gamepad2RightStickY;
					
					if (gamepad2RightStickY < 0) {
						topSystem.moveTop(0.2, false);
						System.out.println("A1Task - Gamepad2 Forward detected, Top motor 20% power");
					}
				} else {
					// If neither right stick Y is active: use gamepad2 L1 for gentle forward
					if (gamepad2.left_bumper && !gamepad2.right_bumper) {
						leftPower = -0.3;   // ~30% forward
						rightPower = -0.3;
						System.out.println("A1Task - L1 Button Forward Control: -0.3 (30% power)");
					} else {
						// Otherwise stop and stop top motor
						leftPower = 0.0;
						rightPower = 0.0;
						topSystem.moveTop(0.0, false);
						System.out.println("A1Task - Robot stopped, Top motor stopped");
					}
				}

				// Turn (right stick X; supports gamepad1 & gamepad2)
				if (Math.abs(rightStickX) >= 0.05) {
					leftPower += -rightStickX;  
					rightPower -= -rightStickX; 
				} else if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
					double gamepad2RightStickX = gamepad2.right_stick_x;
					leftPower += -gamepad2RightStickX;
					rightPower -= -gamepad2RightStickX;
				}

				// Turn (L1/R1)
				if (leftBumper) {
					leftPower += 0.5;   // turn left
					rightPower -= 0.5;
				}
				if (rightBumper) {
					leftPower -= 0.5;   // turn right
					rightPower += 0.5;
				}

				// Clamp power
				leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
				rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

				// Safety: if no input on either gamepad, force 0
				boolean hasGamepad1Input = (Math.abs(rightStickY) >= 0.05) || (Math.abs(rightStickX) >= 0.05) || leftBumper || rightBumper;
				boolean hasGamepad2Input = (Math.abs(gamepad2.right_stick_y) >= 0.05) || (Math.abs(gamepad2.right_stick_x) >= 0.05);
				if (!hasGamepad1Input && !hasGamepad2Input) {
					leftPower = 0.0;
					rightPower = 0.0;
				}

				// Apply power
				if (imuMotorSystem.isLeftMotorConnected()) {
					imuMotorSystem.getLeftMotor().setPower(leftPower);
				}
				if (imuMotorSystem.isRightMotorConnected()) {
					imuMotorSystem.getRightMotor().setPower(rightPower);
				}
			}
			
			// Heading memory: gamepad1 SHARE/BACK
			boolean sharePressed = gamepad1.back;
			imuMotorSystem.updateHeadingMemory(sharePressed);
			
			// Auto-turn toggle: gamepad1 OPTIONS/START
			boolean optionsPressed = gamepad1.start;
			imuMotorSystem.startAutoTurn(optionsPressed);
			
			// Adjust R1 arm target height with gamepad1 L1/R1
			if (gamepad1.left_bumper && !lastShareButton) {
				r1ArmTargetHeight = Math.max(0.0, r1ArmTargetHeight - R1_ARM_HEIGHT_INCREMENT);
				System.out.println("A1Task - R1 Arm Target Height Decreased: " + r1ArmTargetHeight);
			}
			if (gamepad1.right_bumper && !lastOptionsButton) {
				r1ArmTargetHeight = Math.min(1.0, r1ArmTargetHeight + R1_ARM_HEIGHT_INCREMENT);
				System.out.println("A1Task - R1 Arm Target Height Increased: " + r1ArmTargetHeight);
			}
			lastShareButton = gamepad1.left_bumper;
			lastOptionsButton = gamepad1.right_bumper;
			
			// Adjust arm gravity hold power with gamepad1 X/Y
			if (gamepad1.x && !lastShareButton) {
				armHoldPower = Math.max(-0.5, armHoldPower - ARM_HOLD_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Power Decreased: " + armHoldPower);
			}
			if (gamepad1.y && !lastOptionsButton) {
				armHoldPower = Math.min(0.0, armHoldPower + ARM_HOLD_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Power Increased: " + armHoldPower);
			}
			lastShareButton = gamepad1.x;
			lastOptionsButton = gamepad1.y;
			
			// Adjust arm hold max output with gamepad1 X/Y (same keys; keep as original behavior)
			if (gamepad1.x && !lastShareButton) {
				armHoldMaxPower = Math.max(-0.5, armHoldMaxPower - ARM_HOLD_MAX_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Max Power Decreased: " + armHoldMaxPower);
			}
			if (gamepad1.y && !lastOptionsButton) {
				armHoldMaxPower = Math.min(0.0, armHoldMaxPower + ARM_HOLD_MAX_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Max Power Increased: " + armHoldMaxPower);
			}
			
			// R1 (gamepad2 right bumper) auto arm positioning to target (disabled while driving backward)
			if (gamepad2.right_bumper && (Math.abs(rightStickY) < 0.05 || rightStickY <= 0)) {
				double currentArmHeight = potensionSystem.getCurrentHeight();
				double heightError = r1ArmTargetHeight - currentArmHeight;
				
				// Proportional control (clamped)
				double armControlPower = Math.max(-0.3, Math.min(0.3, heightError * 2.0));
				
				// Step-based final power with 20% reduction applied overall
				double finalArmPower;
				if (currentArmHeight >= 0.45) {
					finalArmPower = 0.5 * 0.8; // 40%
					System.out.println("A1Task - Near target position, using 40% power (20% reduced from 50%)");
				} else if (currentArmHeight >= 0.35) {
					finalArmPower = 0.3 * 0.8; // 24%
					System.out.println("A1Task - Middle position, using 24% power (20% reduced from 30%)");
				} else {
					finalArmPower = armControlPower * 1.3 * 0.8; // proportional *1.3 then -20%
					finalArmPower = Math.max(-1.0, Math.min(1.0, finalArmPower));
					System.out.println("A1Task - Low position, using proportional control (20% reduced)");
				}
				
				// Gravity compensation on lift
				liftSystem.moveLift(armHoldPower);
				
				// Debug info
				System.out.println("A1Task - R1 Button Pressed");
				System.out.println("A1Task - Current Arm Height: " + currentArmHeight);
				System.out.println("A1Task - Target Arm Height: " + r1ArmTargetHeight);
				System.out.println("A1Task - Original Arm Control Power: " + armControlPower);
				System.out.println("A1Task - Final Arm Power: " + finalArmPower);
				System.out.println("A1Task - Arm Hold Power: " + armHoldPower);
				
				telemetry.addData("R1 ARM CONTROL", "Target: " + r1ArmTargetHeight + ", Current: " + currentArmHeight + ", Original: " + armControlPower + ", Final: " + finalArmPower + ", Hold: " + armHoldPower);
			}
			
			// R2-button based lift control (simulate left-stick Y if left-stick idle)
			if (gamepad2.right_bumper) {
				if (Math.abs(gamepad2.left_stick_y) > 0.1) {
					double leftStickPower = potensionSystem.getAdjustedLiftPower(gamepad2.left_stick_y);
					liftSystem.moveLift(leftStickPower);
					
					System.out.println("A1Task - R2 Button + Left Stick Y Active");
					System.out.println("A1Task - Left Stick Y Input: " + gamepad2.left_stick_y);
					System.out.println("A1Task - Adjusted Power: " + leftStickPower);
					System.out.println("A1Task - Lift Motor Connected: " + liftSystem.isLiftMotorConnected());
					
					telemetry.addData("R2 LIFT CONTROL", "Left Stick Y Priority: " + gamepad2.left_stick_y + ", Power: " + leftStickPower);
				} else {
					// Use fixed simulated value (r2LiftValue), then boost torque by 20%
					double r2LiftPower = potensionSystem.getAdjustedLiftPower(r2LiftValue);
					double boostedPower = r2LiftPower * 1.2;
					boostedPower = Math.max(-1.0, Math.min(1.0, boostedPower));
					
					liftSystem.moveLift(boostedPower);
					
					System.out.println("A1Task - R2 Button Pressed (Fixed Value)");
					System.out.println("A1Task - Simulated Left Stick Y: " + r2LiftValue);
					System.out.println("A1Task - Original Adjusted Power: " + r2LiftPower);
					System.out.println("A1Task - Boosted Power (20% up): " + boostedPower);
					System.out.println("A1Task - Lift Motor Connected: " + liftSystem.isLiftMotorConnected());
					
					telemetry.addData("R2 LIFT CONTROL", "Fixed Value: " + r2LiftValue + ", Original: " + r2LiftPower + ", Boosted: " + boostedPower);
				}
			}
			
			// Servo control (D-pad) — moved to gamepad2
			// Triangle-button servo control removed — top motor and servos are independent
			servoSystem.controlServosWithDpad(gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.dpad_up, gamepad2.dpad_down);
			
			// Servo control (buttons) — moved to gamepad2
			// Triangle-button control removed — top motor and servos are independent
			if (gamepad2.a) {
				System.out.println("A1Task - A button pressed");
				servoSystem.moveBothServosToMin(); // A → set to min positions
			}
			if (gamepad2.b) {
				System.out.println("A1Task - B button pressed");
				servoSystem.moveBothServosToMax(); // B → set to max positions
			}
			
			// Triangle-button servo control removed — top motor and servos are independent
			
			// Extra mapping logs for debugging
			if (gamepad2.square) {
				System.out.println("A1Task - Square button detected: " + gamepad2.square);
			}
			if (gamepad2.circle) {
				System.out.println("A1Task - Circle button detected: " + gamepad2.circle);
			}
			
			// Servo presets via gamepad2 Share (back) and Options (start)
			if (gamepad2.back) { // Share button
				System.out.println("A1Task - Share Button Pressed: Setting servo1 to 0.25, servo2 to 0.85");
				if (servoSystem.isServo1Connected()) {
					servoSystem.getServo1().setPosition(0.25);
					System.out.println("A1Task - Direct servo1.setPosition(0.25) executed from Share button");
				}
				servoSystem.setServo2Position(0.85);
				System.out.println("A1Task - Share Button: Servo1(0.25) + Servo2(0.85)");
			} else if (gamepad2.start) { // Options button
				System.out.println("A1Task - Options Button Pressed: Setting servo1 to 0.7, servo2 to 0.5");
				if (servoSystem.isServo1Connected()) {
					servoSystem.getServo1().setPosition(0.7);
					System.out.println("A1Task - Direct servo1.setPosition(0.7) executed from Options button");
				}
				servoSystem.setServo2Position(0.5);
				System.out.println("A1Task - Options Button: Servo1(0.7) + Servo2(0.5)");
			}
			
			// Top motor control via A1Top2 (R2 trigger with M1 combo; Triangle = reverse)
			if (gamepad2.right_trigger > 0.05) {
				if (gamepad2.triangle) {
					// Reverse rotation when Triangle held with R2
					topSystem.moveTopWithM1ComboReverse(gamepad2.right_trigger);
					System.out.println("A1Task - R2 + Triangle: Top motor reverse rotation with power: " + gamepad2.right_trigger);
				} else {
					// Normal rotation
					topSystem.moveTopWithM1Combo(gamepad2.right_trigger);
					System.out.println("A1Task - R2 only: Top motor normal rotation with power: " + gamepad2.right_trigger);
				}
				
				// Start delayed servo action
				if (!r2ServoTriggered) {
					r2ServoTimer = System.currentTimeMillis();
					r2ServoTriggered = true;
				}
				
				// After 3 seconds on R2, set servos (servo1: 0.7, servo2: 0.5) if Triangle not pressed
				if (r2ServoTriggered && (System.currentTimeMillis() - r2ServoTimer) >= 3000) {
					System.out.println("A1Task - R2 3-second delay completed, setting servos");
					servoSystem.setServo1Position(0.7);
					servoSystem.setServo2Position(0.5);
					r2ServoTriggered = false; // run once
				}
			} else {
				// If R2 not pressed: stop top motor immediately (when not actively reversing)
				if (Math.abs(rightStickY) < 0.05 || rightStickY >= 0) {
					topSystem.moveTopWithM1Combo(0.0);
				}
				// Reset servo delay timer
				r2ServoTriggered = false;
			}
			
			// Debug
			System.out.println("A1Task - Gamepad2 R2 Trigger: " + gamepad2.right_trigger);
			System.out.println("A1Task - Gamepad2 Triangle Button: " + gamepad2.triangle);
			System.out.println("A1Task - Top Motor Power: " + topSystem.getTopMotorPower());
			System.out.println("A1Task - Top Motor Connected: " + topSystem.isTopMotorConnected());
			
			// Arm motor direct control (simple5MotorTest style) — gamepad2 square
			if (armSystem.isArmMotorConnected()) {
				double armPower = 0.0;
				
				// Square → 100% power
				if (gamepad2.square) {
					armPower = 1.0;
				}
				
				armSystem.getArmMotor().setPower(armPower);
				System.out.println("A1Task - Arm Motor Power: " + armPower + " (Simple5MotorTest Style)");
			}
			
			// === Unified ARM MOTOR control logic ===
			double finalArmPower = 0.0; // final arm motor power
			String armControlSource = "None"; // source tag
			
			// 1) L2 trigger (highest priority)
			if (gamepad2.left_trigger > 0.05) {
				finalArmPower = 1.0; // 100%
				armControlSource = "L2 Trigger (100%)";
				System.out.println("A1Task - L2 Trigger: Arm motor 100% power");
			}
			// 2) Square (2nd)
			else if (gamepad2.square) {
				finalArmPower = 0.442; // 0.340 * 1.3
				armControlSource = "Square Button (44.2%)";
				System.out.println("A1Task - Square Button: Arm motor 44.2% power");
			}
			// 3) R1 (auto position; 3rd)
			else if (gamepad2.right_bumper && (Math.abs(rightStickY) < 0.05 || rightStickY <= 0)) {
				double currentArmHeight = potensionSystem.getCurrentHeight();
				double heightError = r1ArmTargetHeight - currentArmHeight;
				double armControlPower = Math.max(-0.3, Math.min(0.3, heightError * 2.0));
				
				if (currentArmHeight >= 0.45) {
					finalArmPower = 0.4; // 40%
				} else if (currentArmHeight >= 0.35) {
					finalArmPower = 0.24; // 24%
				} else {
					finalArmPower = armControlPower * 1.3 * 0.8;
				}
				finalArmPower = Math.max(-1.0, Math.min(1.0, finalArmPower));
				armControlSource = "R1 Button (Auto Position)";
				System.out.println("A1Task - R1 Button: Arm motor " + finalArmPower + " power");
			}
			// 4) Backward drive present (4th)
			else if (rightStickY > 0 || gamepad2.right_stick_y > 0) {
				finalArmPower = 0.4; // 40%
				armControlSource = "Backward Control (40%)";
				System.out.println("A1Task - Backward: Arm motor 40% power");
			}
			
			// Apply final arm motor power
			if (armSystem.isArmMotorConnected()) {
				armSystem.getArmMotor().setPower(finalArmPower);
			}
			
			// Debug
			System.out.println("A1Task - Final Arm Power: " + finalArmPower + " (" + armControlSource + ")");
			System.out.println("A1Task - Arm Motor Connected: " + armSystem.isArmMotorConnected());
			System.out.println("A1Task - Arm Motor Power: " + armSystem.getArmMotorPower());
			
			// Lift control via A1Lift (gamepad2 left stick)
			double currentHeight = potensionSystem.getCurrentHeight();
			double adjustedLiftPower = potensionSystem.getAdjustedLiftPower(gamepad2.left_stick_y);
			
			// Adjust L1 forward speed (gamepad1 L1/R1)
			if (gamepad1.left_bumper && !lastShareButton) {
				triangleForwardSpeed = Math.max(0.0, triangleForwardSpeed - TRIANGLE_SPEED_INCREMENT);
				System.out.println("A1Task - L1 Forward Speed Decreased: " + triangleForwardSpeed);
			}
			if (gamepad1.right_bumper && !lastOptionsButton) {
				triangleForwardSpeed = Math.min(1.0, triangleForwardSpeed + TRIANGLE_SPEED_INCREMENT);
				System.out.println("A1Task - L1 Forward Speed Increased: " + triangleForwardSpeed);
			}
			lastShareButton = gamepad1.left_bumper;
			lastOptionsButton = gamepad1.right_bumper;
			
			// gamepad2 L1 combo (lift runs regardless of gamepad1 joystick)
			if (gamepad2.left_bumper && !gamepad2.right_bumper) {
				double l1LiftPower = potensionSystem.getAdjustedLiftPower(-0.800);
				liftSystem.moveLift(l1LiftPower);
				
				// Set servos to preset positions (servo1: 0.7, servo2: 0.5)
				System.out.println("A1Task - L1 button pressed, setting servos");
				servoSystem.setServo1Position(0.7);
				servoSystem.setServo2Position(0.5);
				sleep(100); // brief delay for servos
				
				// Run top motor M1 combo (independent of gamepad1)
				topSystem.moveTopM1Combo();
				
				// Debug
				System.out.println("A1Task - L1 Button Pressed (Lift+Top+Servo Control)");
				System.out.println("A1Task - Top Motor Power: " + topSystem.getTopMotorPower());
				System.out.println("A1Task - Top Motor Connected: " + topSystem.isTopMotorConnected());
				System.out.println("A1Task - L1 Top Motor Power: -1.0");
				
				telemetry.addData("LIFT+TOP COMBO", "L1 Button - Lift: -0.800 (Adjusted: " + l1LiftPower + "), Top: -1.0");
			} else if (!gamepad2.right_bumper) {
				// Normal lift control if L1 not pressed and R2 logic not overriding
				if (Math.abs(gamepad2.left_stick_y) > 0.1) {
					liftSystem.moveLift(adjustedLiftPower);
					System.out.println("A1Task - Normal Lift Control: " + adjustedLiftPower);
				} else {
					// If stick idle: hold arm position via lift
					double holdPower = calculateArmHoldPower(currentHeight);
					liftSystem.moveLift(holdPower);
					System.out.println("A1Task - Arm Position Hold Control: Height=" + currentHeight + ", HoldPower=" + holdPower);
				}
			}
			
			// Flask telemetry (if available)
			telemetry.addData("=== GAMEPAD INPUT VALUES ===", "");
			telemetry.addData("Left Stick X", "%.3f", leftStickX);
			telemetry.addData("Left Stick Y", "%.3f", leftStickY);
			telemetry.addData("Right Stick X", "%.3f", rightStickX);
			telemetry.addData("Right Stick Y", "%.3f", rightStickY);
			
			telemetry.addData("Left Trigger", "%.3f", leftTrigger);
			telemetry.addData("Right Trigger", "%.3f", rightTrigger);
			
			telemetry.addData("Left Bumper", leftBumper ? "PRESSED" : "Released");
			telemetry.addData("Right Bumper", rightBumper ? "PRESSED" : "Released");
			
			telemetry.addData("A Button", aButton ? "PRESSED" : "Released");
			telemetry.addData("B Button", bButton ? "PRESSED" : "Released");
			telemetry.addData("X Button", xButton ? "PRESSED" : "Released");
			telemetry.addData("Y Button", yButton ? "PRESSED" : "Released");
			
			telemetry.addData("D-pad Up", dpadUp ? "PRESSED" : "Released");
			telemetry.addData("D-pad Down", dpadDown ? "PRESSED" : "Released");
			telemetry.addData("D-pad Left", dpadLeft ? "PRESSED" : "Released");
			telemetry.addData("D-pad Right", dpadRight ? "PRESSED" : "Released");
			
			// Gamepad2 inputs
			telemetry.addData("=== GAMEPAD2 INPUT VALUES ===", "");
			telemetry.addData("Gamepad2 Left Stick X", "%.3f", gamepad2.left_stick_x);
			telemetry.addData("Gamepad2 Left Stick Y", "%.3f", gamepad2.left_stick_y);
			telemetry.addData("Gamepad2 Right Stick X", "%.3f", gamepad2.right_stick_x);
			telemetry.addData("Gamepad2 Right Stick Y", "%.3f", gamepad2.right_stick_y);
			telemetry.addData("Gamepad2 Left Trigger", "%.3f", gamepad2.left_trigger);
			telemetry.addData("Gamepad2 Right Trigger", "%.3f", gamepad2.right_trigger);
			telemetry.addData("Gamepad2 Left Bumper", gamepad2.left_bumper ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Right Bumper", gamepad2.right_bumper ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 A Button", gamepad2.a ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 B Button", gamepad2.b ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 X Button", gamepad2.x ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Y Button", gamepad2.y ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Square Button", gamepad2.square ? "PRESSED - Arm Motor Control" : "Released");
			telemetry.addData("Gamepad2 Circle Button", gamepad2.circle ? "PRESSED - Top Motor Reverse Control" : "Released");
			telemetry.addData("Gamepad2 Triangle Button", gamepad2.triangle ? "PRESSED - Top Motor Control" : "Released");
			telemetry.addData("Gamepad2 D-pad Up", gamepad2.dpad_up ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Down", gamepad2.dpad_down ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Left", gamepad2.dpad_left ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Right", gamepad2.dpad_right ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Share (Back)", gamepad2.back ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Options (Start)", gamepad2.start ? "PRESSED" : "Released");
			
			telemetry.addData("=== DRIVE CONTROL ===", "");
			telemetry.addData("Right Stick Y", "%.3f", rightStickY);
			telemetry.addData("Right Stick X", "%.3f", rightStickX);
			telemetry.addData("Left Bumper (L1)", leftBumper ? "PRESSED" : "Released");
			telemetry.addData("Right Bumper (R1)", rightBumper ? "PRESSED" : "Released");
			telemetry.addData("Left Motor Power", "%.3f", imuMotorSystem.getLeftMotorPower());
			telemetry.addData("Right Motor Power", "%.3f", imuMotorSystem.getRightMotorPower());
			telemetry.addData("Left Motor Connected", imuMotorSystem.isLeftMotorConnected());
			telemetry.addData("Right Motor Connected", imuMotorSystem.isRightMotorConnected());
			
			boolean noInput = (Math.abs(rightStickY) < 0.05) && (Math.abs(rightStickX) < 0.05) && !leftBumper && !rightBumper;
			telemetry.addData("No Input Detected", noInput ? "YES - Motors should be stopped" : "NO - Input detected");
			
			telemetry.addData("=== ARM CONTROL ===", "");
			telemetry.addData("Arm Motor Power", "%.3f", armSystem.getArmMotorPower());
			telemetry.addData("Arm Square Button (Gamepad2)", gamepad2.square ? "PRESSED - Fixed Power 0.442 (Simple5MotorTest Style)" : "Released");
			telemetry.addData("R1 Arm Control Active", gamepad2.right_bumper ? "YES - Auto Position Control" : "NO");
			telemetry.addData("R1 Arm Target Height", "%.2f (P1 L1/R1 to adjust)", r1ArmTargetHeight);
			telemetry.addData("R2 Button Lift Control", gamepad2.right_bumper ? "YES - Simulated Left Stick Y: " + r2LiftValue : "NO");
			telemetry.addData("Arm Hold Power", "%.2f (P1 X/Y to adjust)", armHoldPower);
			telemetry.addData("Arm Position Hold Max Power", "%.2f (P1 X/Y to adjust)", armHoldMaxPower);
			telemetry.addData("Arm Position Hold Active", currentHeight >= ARM_HOLD_MIN_HEIGHT ? "YES" : "NO");
			telemetry.addData("Arm Position Hold Height", "%.3f (Min: %.2f, Max: %.2f)", currentHeight, ARM_HOLD_MIN_HEIGHT, ARM_HOLD_MAX_HEIGHT);
			telemetry.addData("Arm Motor Connected", armSystem.isArmMotorConnected());
			
			telemetry.addData("=== TOP MOTOR CONTROL ===", "");
			telemetry.addData("Top Motor Power", "%.3f", topSystem.getTopMotorPower());
			telemetry.addData("Top Command (Gamepad2 R2)", "%.3f", gamepad2.right_trigger);
			telemetry.addData("Top M1 Combo Active", gamepad2.right_trigger > 0.1 ? "YES" : "NO");
			telemetry.addData("Top Current Power", "%.3f", topSystem.getCurrentTopPower());
			telemetry.addData("Top Target Power", "%.3f", topSystem.getTargetTopPower());
			telemetry.addData("Top Motor Connected", topSystem.isTopMotorConnected());
			telemetry.addData("Triangle Button (Gamepad2)", gamepad2.triangle ? "PRESSED - Reverse Rotation" : "Released");
			telemetry.addData("Triangle Top Motor Power", "%.3f", gamepad2.triangle ? -1.0 : 0.0);
			telemetry.addData("Gamepad2 R2 Active", gamepad2.right_trigger > 0.05 ? "YES" : "NO");
			telemetry.addData("R2 + Triangle Combo", (gamepad2.right_trigger > 0.05 && gamepad2.triangle) ? "ACTIVE - Reverse" : "Inactive");
			telemetry.addData("Top Rotation Direction", (gamepad2.right_trigger > 0.05 && gamepad2.triangle) ? "REVERSE" : (gamepad2.right_trigger > 0.05 ? "NORMAL" : "STOPPED"));
			
			// IMU telemetry
			telemetry.addData("=== IMU DATA ===", "");
			if (imuMotorSystem.isInitialized()) {
				telemetry.addData("IMU Status", "Connected");
				telemetry.addData("Yaw", "%.2f°", imuMotorSystem.getYaw());
				telemetry.addData("Pitch", "%.2f°", imuMotorSystem.getPitch());
				telemetry.addData("Roll", "%.2f°", imuMotorSystem.getRoll());
				telemetry.addData("Heading", "%.2f°", imuMotorSystem.getHeading());
				telemetry.addData("Angular Velocity Z", "%.2f°/s", imuMotorSystem.getAngularVelocityZ());
				
				// Stored heading display
				if (imuMotorSystem.hasStoredHeading()) {
					telemetry.addData("Stored Heading", "%.2f° (%s)", imuMotorSystem.getStoredHeading(), 
						imuMotorSystem.getHeadingDirection(imuMotorSystem.getStoredHeading()));
				} else {
					telemetry.addData("Stored Heading", "NONE - Press SHARE/BACK to store");
				}
				
				// Current heading direction
				telemetry.addData("Current Direction", "%s (%.2f°)", 
					imuMotorSystem.getHeadingDirection(imuMotorSystem.getYaw()), imuMotorSystem.getYaw());
				
				// Auto-turn status
				telemetry.addData("Auto Turn Active", imuMotorSystem.isAutoTurnActive() ? "YES" : "NO");
				telemetry.addData("Heading Locked", imuMotorSystem.isHeadingLocked() ? "YES" : "NO");
				if (imuMotorSystem.isAutoTurnActive()) {
					double[] autoTurnControlTelemetry = imuMotorSystem.calculateAutoTurnControl();
					telemetry.addData("Auto Turn Left Power", "%.3f", autoTurnControlTelemetry[0]);
					telemetry.addData("Auto Turn Right Power", "%.3f", autoTurnControlTelemetry[1]);
				}
			} else {
				telemetry.addData("IMU Status", "Not Connected");
				if (imuMotorSystem.hasError()) {
					telemetry.addData("IMU Error", imuMotorSystem.getLastError());
				}
			}
			
			telemetry.addData("=== LIFT CONTROL ===", "");
			telemetry.addData("Lift Motor Power", "%.3f", liftSystem.getLiftMotorPower());
			telemetry.addData("Lift Command (Gamepad2 Left Stick Y)", "%.3f", gamepad2.left_stick_y);
			telemetry.addData("Lift Motor Connected", liftSystem.isLiftMotorConnected());
			telemetry.addData("L1 Combo Active", gamepad2.left_bumper ? "YES - Lift+Top+Forward Simultaneous" : "NO - Normal Control");
			telemetry.addData("L1 Forward Speed", "%.2f (P1 L1/R1 to adjust)", triangleForwardSpeed);
			
			telemetry.addData("=== POTENTIOMETER ===", "");
			telemetry.addData("Potentiometer Voltage", "%.3f V", potensionSystem.getPotentiometerVoltage());
			telemetry.addData("Potentiometer Normalized", "%.3f", currentHeight);
			telemetry.addData("Current Height", "%.3f", currentHeight);
			telemetry.addData("Height Limit (Upper)", "%.3f", potensionSystem.getMaxHeight());
			telemetry.addData("Reduction Factor", "%.3f", potensionSystem.getReductionFactor());
			telemetry.addData("Adjusted Lift Power", "%.3f", adjustedLiftPower);
			telemetry.addData("Original Lift Power (Gamepad2 Left Stick Y)", "%.3f", gamepad2.left_stick_y);
			telemetry.addData("Auto Height Control", potensionSystem.isAutoHeightControlEnabled() ? "ON" : "OFF");
			telemetry.addData("Target Height", "%.3f", potensionSystem.getTargetHeight());
			telemetry.addData("Height Error", "%.3f", potensionSystem.getHeightError());
			telemetry.addData("Potentiometer Connected", potensionSystem.isPotentiometerConnected());
			
			telemetry.addData("=== SERVO CONTROL ===", "");
			telemetry.addData("Servo1 Position", "%.3f", servoSystem.getServo1Position());
			telemetry.addData("Servo2 Position", "%.3f", servoSystem.getServo2Position());
			telemetry.addData("Servo1 Connected", servoSystem.isServo1Connected());
			telemetry.addData("Servo2 Connected", servoSystem.isServo2Connected());
			telemetry.addData("Servo Control (Gamepad2)", "D-pad: Left/Right=Servo1, Up/Down=Servo2");
			telemetry.addData("Servo Button Control (Gamepad2)", "A=Servo1(0.250)+Servo2(0.500), B=Servo1(0.700)+Servo2(0.850)");
			telemetry.addData("Servo Triangle Control (Gamepad2)", "Triangle=Servo1(0.700)+Servo2(0.500)");
			
			// Flask status (if available)
			if (flaskSystem != null) {
				telemetry.addData("=== FLASK STATUS ===", "");
				telemetry.addData("Send Count", flaskSystem.getSendCount());
				telemetry.addData("Success Count", flaskSystem.getSuccessCount());
				telemetry.addData("Error Count", flaskSystem.getErrorCount());
				telemetry.addData("Success Rate", "%.1f%%", flaskSystem.getSuccessRate());
				telemetry.addData("Last Error", flaskSystem.getLastError());
				telemetry.addData("Server URL", flaskSystem.getServerURL());
			} else {
				telemetry.addData("=== FLASK STATUS ===", "DISABLED - Flask system not available");
			}
			
			telemetry.addData("=== SYSTEM INFO ===", "");
			telemetry.addData("Update Rate", "20 Hz (50ms)");
			telemetry.addData("Status", "Running");
			
			// Servo1 position change tracking
			double currentServo1Position = servoSystem.getServo1Position();
			if (Math.abs(currentServo1Position - lastServo1Position) > 0.01) {
				telemetry.addData("*** SERVO1 POSITION CHANGE ***", "%.3f -> %.3f", lastServo1Position, currentServo1Position);
				telemetry.addData("*** CHANGE DETECTED ***", "Check Logcat for details");
				lastServo1Position = currentServo1Position;
			} else {
				telemetry.addData("Servo1 Position Stable", "%.3f", currentServo1Position);
			}
			
			// Button states for top motor mapping
			telemetry.addData("Square Button Status", gamepad2.square ? "PRESSED - Top Motor Forward" : "Released");
			telemetry.addData("Circle Button Status", gamepad2.circle ? "PRESSED - Top Motor Reverse" : "Released");
			
			// Current servo control state
			if (gamepad2.square) {
				telemetry.addData("*** SQUARE ACTIVE ***", "Top Motor Forward");
			} else if (gamepad2.circle) {
				telemetry.addData("*** CIRCLE ACTIVE ***", "Top Motor Reverse");
			} else {
				telemetry.addData("*** NO SQUARE/CIRCLE ***", "Other controls active");
			}
			
			// === SERVO1 POSITION TRACKING ===
			telemetry.addData("=== SERVO1 POSITION TRACKING ===", "");
			telemetry.addData("Current Servo1 Position", "%.3f", currentServo1Position);
			telemetry.addData("Last Servo1 Position", "%.3f", lastServo1Position);
			
			// Change detection
			if (Math.abs(currentServo1Position - lastServo1Position) > 0.01) {
				telemetry.addData("*** POSITION CHANGED ***", "YES");
				telemetry.addData("Change Amount", "%.3f", currentServo1Position - lastServo1Position);
			} else {
				telemetry.addData("*** POSITION CHANGED ***", "NO");
			}
			
			// Detailed button state
			telemetry.addData("Gamepad2 Square (Top Forward)", gamepad2.square ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Circle (Top Reverse)", gamepad2.circle ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 A", gamepad2.a ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 B", gamepad2.b ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Left", gamepad2.dpad_left ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Right", gamepad2.dpad_right ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Up", gamepad2.dpad_up ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Down", gamepad2.dpad_down ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 R2 Trigger", "%.3f", gamepad2.right_trigger);
			telemetry.addData("Gamepad2 L1 Bumper", gamepad2.left_bumper ? "PRESSED" : "Released");
			
			// Button debug
			telemetry.addData("=== BUTTON DEBUG ===", "");
			telemetry.addData("Square (Top Forward) Raw Value", gamepad2.square);
			telemetry.addData("Circle (Top Reverse) Raw Value", gamepad2.circle);
			telemetry.addData("Triangle Raw Value", gamepad2.triangle);
			telemetry.addData("Cross (X) Raw Value", gamepad2.x);
			telemetry.addData("A Button Raw Value", gamepad2.a);
			telemetry.addData("B Button Raw Value", gamepad2.b);
			telemetry.addData("Both Square and Circle Pressed", (gamepad2.square && gamepad2.circle) ? "YES" : "NO");
			telemetry.addData("Neither Square nor Circle Pressed", (!gamepad2.square && !gamepad2.circle) ? "YES" : "NO");
			
			// Button mapping check
			telemetry.addData("=== BUTTON MAPPING CHECK ===", "");
			if (gamepad2.square) telemetry.addData("Square (Top Forward) Detected", "YES");
			if (gamepad2.circle) telemetry.addData("Circle (Top Reverse) Detected", "YES");
			if (gamepad2.triangle) telemetry.addData("Triangle Detected", "YES");
			if (gamepad2.x) telemetry.addData("Cross (X) Detected", "YES");
			if (gamepad2.a) telemetry.addData("A Button Detected", "YES");
			if (gamepad2.b) telemetry.addData("B Button Detected", "YES");
			
			// Control priority status
			if (gamepad2.square) {
				telemetry.addData("*** SQUARE PRIORITY ***", "ACTIVE - Top Motor Forward");
				telemetry.addData("Square Override Status", "Top motor forward 100% power");
			} else if (gamepad2.circle) {
				telemetry.addData("*** CIRCLE PRIORITY ***", "ACTIVE - Top Motor Reverse");
				telemetry.addData("Circle Override Status", "Top motor reverse 100% power");
			} else {
				telemetry.addData("*** NORMAL CONTROL ***", "ACTIVE");
				telemetry.addData("D-pad Control", "Enabled");
				telemetry.addData("A/B Button Control", "Enabled");
				telemetry.addData("R2 Servo Control", "Enabled");
				telemetry.addData("L1 Servo Control", "Enabled");
			}
			
			// Expected servo1 position (updated)
			if (gamepad2.back) {
				telemetry.addData("Expected Servo1 Position", "0.250 (Share Button)");
			} else if (gamepad2.start) {
				telemetry.addData("Expected Servo1 Position", "0.700 (Options Button)");
			} else if (gamepad2.a) {
				telemetry.addData("Expected Servo1 Position", "0.250 (A Button)");
			} else if (gamepad2.b) {
				telemetry.addData("Expected Servo1 Position", "0.700 (B Button)");
			} else if (gamepad2.dpad_left) {
				telemetry.addData("Expected Servo1 Position", "Decreasing (D-pad Left)");
			} else if (gamepad2.dpad_right) {
				telemetry.addData("Expected Servo1 Position", "Increasing (D-pad Right)");
			} else if (gamepad2.right_trigger > 0.05) {
				telemetry.addData("Expected Servo1 Position", "0.700 (R2 Trigger - 3s delay)");
			} else if (gamepad2.left_bumper) {
				telemetry.addData("Expected Servo1 Position", "0.700 (L1 Button)");
			} else {
				telemetry.addData("Expected Servo1 Position", "No change expected");
			}
			
			// Position mismatch note (Circle no longer controls servos)
			telemetry.addData("Position Match", "OK - Circle button no longer controls servos");
			
			telemetry.update();
			
			// 50ms loop delay
			sleep(50);
		}
	}
}
