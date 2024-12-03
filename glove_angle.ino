#include <Wire.h>
#include <Kalman.h>

#define RESTRICT_PITCH  // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

// PID 제어 변수를 위한 구조체
struct PIDController {
  double Kp;
  double Ki;
  double Kd;
  double previousError;
  double integral;
  double setpoint;

  PIDController(double p, double i, double d) : Kp(p), Ki(i), Kd(d), previousError(0), integral(0), setpoint(0) {}

  double compute(double target, double input, double dt) {
    setpoint = target;
    double error = setpoint - input;
    integral += error * dt;
    double derivative = (error - previousError) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    return output;
  }

  double maxerror(double target, double input) {
    setpoint = target;
    double error = setpoint - input;
    integral += error * 0.01;
    double derivative = error / 0.01;
    double output = Kp * error + Ki * integral + Kd * derivative;
    return output;
  }
};

// 온도 측정과 필터를 처리하는 구조체
struct Temperature {
  float alpha;  // Low Pass Filter 계수 (0 < alpha < 1)
  float temperature;
  float filteredTemperature;
  int CS, SO, sck;

  Temperature(float a, int cs, int so, int sck_pin) : alpha(a), temperature(0), filteredTemperature(0), CS(cs), SO(so), sck(sck_pin) {}

  void readTemp() {
    int error = 0;
    int Value = 0;

    // 온도변환을 초기화
    digitalWrite(CS, LOW);
    delay(2);
    digitalWrite(CS, HIGH);
    delay(2);

    // CS핀을 LOW로 만들어 값을 읽어옴
    digitalWrite(CS, LOW);

    // D15 더미 비트를 위한 클럭 발생
    digitalWrite(sck, HIGH);
    delay(1);
    digitalWrite(sck, LOW);

    // D14 ~ D3의 값을 읽어옴
    for (int i = 11; i >= 0; i--) {
      digitalWrite(sck, HIGH);
      Value += digitalRead(SO) << i;
      digitalWrite(sck, LOW);
    }

    // D2값을 읽어서 에러여부를 확인
    digitalWrite(sck, HIGH);
    error = digitalRead(SO);
    digitalWrite(sck, LOW);

    // D1, D0을 위한 클럭 발생
    for (int i = 1; i >= 0; i--) {
      digitalWrite(sck, HIGH);
      delay(1);
      digitalWrite(sck, LOW);
    }

    // CS핀을 HIGH로 만들어 읽기를 끝냄
    digitalWrite(CS, HIGH);

    if (error != 0) {
      temperature = -CS;
    } else {
      temperature = Value * 0.25;
    }

    // 저역통과 필터를 사용하여 온도를 부드럽게 필터링
    //filteredTemperature = alpha * temperature + (1 - alpha) * filteredTemperature;
    filteredTemperature = temperature;
  }

  float getFilteredTemperature() {
    return filteredTemperature;
  }
};

// MPU와 칼만 필터를 처리하는 구조체
struct MPU {
  Kalman kalmanX, kalmanY;

  double accX, accY, accZ;
  double gyroX, gyroY, gyroZ;

  double kalAngleX, kalAngleY;  // Calculated angle using a Kalman filter

  uint8_t i2cData[14];
  int num_imu__;
  uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
  const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

  uint32_t timer;

  MPU(int num_imu) : kalmanX(), kalmanY(), accX(0), accY(0), accZ(0), gyroX(0), gyroY(0), gyroZ(0), kalAngleX(0), kalAngleY(0), timer(0) , num_imu__(num_imu), IMUAddress(), I2C_TIMEOUT() {}

  void initialize() {
    Wire.begin();
    Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
    i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00;  // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to ±2g

    while (i2cWrite(num_imu__, 0x19, i2cData, 4, false))
      ;  // Write to all four registers at once
    while (i2cWrite(num_imu__, 0x6B, 0x01, true))
      ;  // PLL with X axis gyroscope reference and disable sleep mode
    while (i2cRead(num_imu__, 0x75, i2cData, 1))
      ;

    delay(100);

    update();

    // 칼만 필터 초기화
    kalmanX.setAngle(getKalAngleX());  // 시작 각도
    kalmanY.setAngle(0);
    timer = micros();  // 초기 타이머 설정
  }

  void update() {
    while (i2cRead(num_imu__, 0x3B, i2cData, 14)) ;  // IMU에서 데이터를 읽음
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

    double dt = (double)(micros() - timer) / 1000000;  // 시간 차이 계산
    timer = micros();
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees

    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

    double gyroXrate = gyroX / 131.0;  // 자이로 값을 각속도로 변환
    double gyroYrate = gyroY / 131.0;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      kalAngleX = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  }

  double getKalAngleX() {
    return kalAngleX;
  }


  uint8_t i2cWrite(int num, uint8_t registerAddress, uint8_t data, bool sendStop) {
    return i2cWrite(num, registerAddress, &data, 1, sendStop); // Returns 0 on success
  }

  uint8_t i2cWrite(int num, uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
    if (num == 1) IMUAddress = 0x68;
    else if (num == 2) IMUAddress = 0x69;
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    Wire.write(data, length);
    uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
    if (rcode) {
      Serial.print(F("i2cWrite failed: "));
      Serial.println(rcode);
    }
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }

  uint8_t i2cRead(int num, uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
    uint32_t timeOutTimer;
    if (num == 1) IMUAddress = 0x68;
    else if (num == 2) IMUAddress = 0x69;

    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
    if (rcode) {
      Serial.print(F("i2cRead failed: "));
      Serial.println(rcode);
      return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
    }
    Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
    for (uint8_t i = 0; i < nbytes; i++) {
      if (Wire.available())
        data[i] = Wire.read();
      else {
        timeOutTimer = micros();
        while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
        if (Wire.available())
          data[i] = Wire.read();
        else {
          Serial.println(F("i2cRead timeout"));
          return 5; // This error value is not already taken by endTransmission
        }
      }
    }
    return 0; // Success
  }


};

//---------------------------
// 전역 변수 설정
PIDController pidX(10.81 , 3.874, 0.0);  // PID 값 설정
Temperature temperature(0.1, 10, 12, 13);  // Low Pass Filter 계수 및 핀 설정
MPU mpu1(1);
MPU mpu2(2);// MPU 인스턴스

int CS = 10;
int SO = 12;
int sck = 13;

int ena = 5;
int in1 = 6;
int in2 = 7;
char count = 's';
int pwm = 0;
double pidMax = 0.0;
double target;
double incalX;

unsigned long lastTime = 0;   // 이전 시간 (PID 계산)
unsigned long lastOutputTime = 0; // 이전 출력 적용 시간
unsigned long sampleTime = 200; // PID 계산 샘플링 시간 (밀리초)
//unsigned long outputApplyInterval = 400; // 제어 출력 적용 간격 (밀리초)

void setup() {
  // 핀 설정
  pinMode(CS, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(sck, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena, OUTPUT);
  Serial.begin(115200);

  // MPU 초기화
  mpu1.initialize();
  mpu2.initialize();
  double inkalAngleX_fixed = mpu1.getKalAngleX();  // 기준 값
  double inkalAngleX = mpu2.getKalAngleX();  // X축 값
  incalX = abs(inkalAngleX_fixed - inkalAngleX);
  

  lastTime = millis();  // 현재 시간을 초기화
  lastOutputTime = millis(); // 출력 적용 시간을 초기화
}

void loop() {
  // 현재 시간 확인
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  if(Serial.available()!=0){
    target = Serial.parseInt();
    double inkalAngleX_fixed = mpu1.getKalAngleX();  // 기준 값
    double inkalAngleX = mpu2.getKalAngleX();  // X축 값
    incalX = abs(inkalAngleX_fixed - inkalAngleX);
    pidMax = pidX.maxerror(target, incalX);
  }


  if (deltaTime >= sampleTime) {
    // MPU 갱신
    mpu1.update();
    mpu2.update();
    // 온도 측정 및 필터 처리
    temperature.readTemp();
    // 9. 마지막 시간 업데이트
    lastTime = currentTime;
  
  // PID 계산 (kalAngleX는 고정, kalAngleY만 제어)
  double kalAngleX_fixed = mpu1.getKalAngleX();  // 기준 값
  double kalAngleX = mpu2.getKalAngleX();  // X축 값
  double calX = abs(kalAngleX_fixed - kalAngleX);
  // Print Angle
  Serial.print("initAngle: ");
  Serial.print(incalX);
  Serial.print(" ");
  Serial.print("setAngle: ");
  Serial.print(target);
  Serial.print(" ");
  Serial.print("CurrentAngle: ");
  Serial.print(calX);
  Serial.print(" ");
  // Print Temperature
  Serial.print("Temperature: ");
  Serial.print(temperature.getFilteredTemperature());
  Serial.print(" ");
  double dt = (double)(micros() - mpu1.timer) / 1000000;  // Delta time 계산
  double pidOutput = pidX.compute(target,calX, dt);  // PID 출력 계산

  // PID 출력값을 활용한 제어
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //pwm = constrain(pidOutput, 0, 255);
  //PWM 값 변환
  pwm = (pidOutput / pidMax) * 255;
  Serial.print("MAXERROR: ");
  Serial.print(pidMax);
  Serial.print(" ");
  
  if (pwm < 0) pwm = 0;
  else if (pwm > 255) pwm = 255;
  Serial.print("PWM: ");
  Serial.println(pwm);
  //pwm = map(pidOutput, 0, pidMax,  0, 255);
  analogWrite(ena, pwm);
  }
#if 0
  if (Serial.available() > 0) {
    count = Serial.read();
  }

  if (count == 'u') {
    //delay(500);
    pwm = 255;
    //pwm += 1;
    if (pwm >= 255) {
      pwm = 255;
    }
    //Serial.print("pwm = ");
    //Serial.println(pwm);

  }
  else if (count == 'd') {
    delay(500);
    pwm -= 1;
    if (pwm <= 0) pwm = 0;
    //Serial.print("pwm = ");
    //Serial.println(pwm);

  }

  else if (count == 's') {
    pwm = 0;
    //Serial.print("pwm = ");
    //Serial.println(pwm);
  }
#endif
}
