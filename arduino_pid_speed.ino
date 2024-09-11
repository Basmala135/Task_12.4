String command = "";    // Command received from Python
boolean motor_start = false; // Track motor status

const byte pin_a = 2;   // for encoder pulse A
const byte pin_b = 3;   // for encoder pulse B
const byte pin_fwd = 4; // for H-bridge: run motor forward
const byte pin_bwd = 5; // for H-bridge: run motor backward
const byte pin_pwm = 6; // for H-bridge: motor speed

int encoder = 0;
int m_direction = 0;
double set_speed = 0;  // Desired motor speed
double pv_speed = 0;   // Current motor speed (from encoder)

double kp = 1.0, ki = 0.0, kd = 0.0;  // PID constants (default values)
double e_speed = 0, e_speed_pre = 0, e_speed_sum = 0;  // PID errors
double pwm_pulse = 0;  // PWM output (0-255)

int timer1_counter;  // For timer setup

void setup() {
  pinMode(pin_a, INPUT_PULLUP);
  pinMode(pin_b, INPUT_PULLUP);
  pinMode(pin_fwd, OUTPUT);
  pinMode(pin_bwd, OUTPUT);
  pinMode(pin_pwm, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pin_a), detect_a, RISING);  // Encoder pulse detection

  Serial.begin(9600);   // Start serial communication

  // Timer setup for speed calculation
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 59286;  // Timer for 0.1 sec overflow
  TCNT1 = timer1_counter;
  TCCR1B |= (1 << CS12);   // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // Enable timer overflow interrupt
  interrupts();

  analogWrite(pin_pwm, 0);   // Initially stop motor
  digitalWrite(pin_fwd, 0);  // Stop motor forward motion
  digitalWrite(pin_bwd, 0);  // Stop motor backward motion
}

void loop() {
  // Check for serial input
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');  // Read the complete command

    // Process the command received
    if (command == "start") {
      motor_start = true;
      digitalWrite(pin_fwd, HIGH);   // Run motor forward
      digitalWrite(pin_bwd, LOW);
    } 
    else if (command == "stop") {
      motor_start = false;
      digitalWrite(pin_fwd, LOW);    // Stop motor
      digitalWrite(pin_bwd, LOW);
      analogWrite(pin_pwm, 0);
    } 
    else if (command.startsWith("set_speed")) {
      set_speed = command.substring(10).toFloat();  // Extract and set speed value
    }
  }

  // PID control logic
  if (motor_start) {
    e_speed = set_speed - pv_speed;
    pwm_pulse = e_speed * kp + e_speed_sum * ki + (e_speed - e_speed_pre) * kd;
    e_speed_pre = e_speed;  // Update previous error
    e_speed_sum += e_speed; // Accumulate error

    // Clamp the PWM value to 0-255
    pwm_pulse = constrain(pwm_pulse, 0, 255);
    analogWrite(pin_pwm, pwm_pulse);  // Adjust motor speed
  }
}

void detect_a() {
  encoder += 1;  // Increment encoder count on pulse A
}

// Timer interrupt for speed calculation every 0.1 seconds
ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1_counter;  // Reset timer
  pv_speed = (600.0 * (encoder / 200.0)) / 0.1;  // Calculate speed (RPM)
  encoder = 0;

  // Send current speed to the Python GUI
  Serial.print("speed:");
  Serial.println(pv_speed);
}
