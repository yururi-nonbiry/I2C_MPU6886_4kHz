// I2C_MPU6886_4kHzのサンプルプログラム
// I2C 400kHzのサンプリング速度
// 1軸読取 MAX 4KHz
// 3軸同時読取 MAX 2875kHz
// 6軸同時読取 MAX 1440kHz

// サンプル数
#define SAMPLE 2048

#include "I2C_MPU6886_4kHz.h"

I2C_MPU6886_4kHz imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire1);

float sample_value[SAMPLE];    //値格納用
int16_t sample_binary[SAMPLE]; //値格納用

void value_read()
{
  // 値格納用(float)
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;

  // 1軸読取
  imu.getAccel_x(&ax);
  imu.getAccel_y(&ay);
  imu.getAccel_z(&az);
  imu.getGyro_x(&gx);
  imu.getGyro_y(&gy);
  imu.getGyro_z(&gz);
  imu.getTemp(&t);

  Serial.printf("%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t);

  // 3軸読取
  imu.getAccel(&ax, &ay, &az);
  imu.getGyro(&gx, &gy, &gz);

  Serial.printf("%f,%f,%f,%f,%f,%f,%f\n", ax, ay, az, gx, gy, gz, t);
}

void binary_read()
{
  // 値格納用(バイナリ)
  int16_t ax_binary;
  int16_t ay_binary;
  int16_t az_binary;
  int16_t gx_binary;
  int16_t gy_binary;
  int16_t gz_binary;
  int16_t t_binary;

  imu.getAccel_x_binary(&ax_binary);
  imu.getAccel_y_binary(&ay_binary);
  imu.getAccel_z_binary(&az_binary);
  imu.getGyro_x_binary(&gx_binary);
  imu.getGyro_y_binary(&gy_binary);
  imu.getGyro_z_binary(&gz_binary);
  imu.getTemp_binary(&t_binary);

  Serial.print(ax_binary);
  Serial.print(F(","));
  Serial.print(ay_binary);
  Serial.print(F(","));
  Serial.print(az_binary);
  Serial.print(F(","));
  Serial.print(gx_binary);
  Serial.print(F(","));
  Serial.print(gy_binary);
  Serial.print(F(","));
  Serial.print(gz_binary);
  Serial.print(F(","));
  Serial.println(t_binary);

  imu.getAccel_binary(&ax_binary, &ay_binary, &az_binary);
  imu.getGyro_binary(&gx_binary, &gy_binary, &gz_binary);

  Serial.print(ax_binary);
  Serial.print(F(","));
  Serial.print(ay_binary);
  Serial.print(F(","));
  Serial.print(az_binary);
  Serial.print(F(","));
  Serial.print(gx_binary);
  Serial.print(F(","));
  Serial.print(gy_binary);
  Serial.print(F(","));
  Serial.print(gz_binary);
  Serial.print(F(","));
  Serial.println(t_binary);
}

void sampling()
{

  unsigned long start_time = micros(); // スタート時間保存

  // 数値用
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float t;

  //バイナリ用
  int16_t ax_binary;
  int16_t ay_binary;
  int16_t az_binary;
  int16_t gx_binary;
  int16_t gy_binary;
  int16_t gz_binary;
  int16_t t_binary;

  for (int i = 0; i < SAMPLE; i++)
  {

    // 1軸数値
    imu.getAccel_x(&ax);
    //imu.getAccel_y(&ay);
    //imu.getAccel_z(&az);
    //imu.getGyro_x(&gx);
    //imu.getGyro_y(&gy);
    //imu.getGyro_z(&gz);
    //imu.getTemp(&t);

    // 3軸数値
    //imu.getAccel(&ax, &ay, &az);
    //imu.getGyro(&gx, &gy, &gz);

    // 1軸バイナリ
    //imu.getAccel_x_binary(&ax_binary);
    //imu.getAccel_y_binary(&ay_binary);
    //imu.getAccel_z_binary(&az_binary);
    //imu.getGyro_x_binary(&gx_binary);
    //imu.getGyro_y_binary(&gy_binary);
    //imu.getGyro_z_binary(&gz_binary);
    //imu.getTemp_binary(&t_binary);

    // 3軸バイナリ
    //imu.getAccel_binary(&ax_binary, &ay_binary, &az_binary);
    //imu.getGyro_binary(&gx_binary, &gy_binary, &gz_binary);

    // sample[]に値を入れたい場合はコメントアウトを外して変数を入れる
    sample_value[i] = ax;
    //sample_binary[i] = t_binary;
  }
  unsigned long end_time = micros(); // 終了時間保存

  // サンプリング周期確認用
  Serial.print(F("Sampling frequency : "));
  Serial.println(1000000.0 / float(end_time - start_time) * SAMPLE);

  //値を確認するときは下のコメントアウトを外す
  for (int i = 0; i < SAMPLE; i++)
  {
    Serial.println(sample_value[i], 4);
    //Serial.println(sample_binary[i]);
  }
}

void setup()
{
  Serial.begin(115200);

  delay(100);

  Wire1.begin(25, 21);    // I2Cのピン設定
  Wire1.setClock(400000); // I2Cの速度を400kHzにする

  // センサのレンジ設定(beginの前に行うこと)
  // ACCEL_CONFIG (加速度)
  // 0:2g 1:4g 2:8g 3:16g [default 2:8g]
  imu.accel_config = 2;
  // GYRO_CONFIG (ジャイロ)
  // 0:250dps 1:500dps 2:1000dps 3:2000dps [default 3:2000dps]
  imu.gyro_config = 3;

  imu.begin(); // MPU6886の初期化
}

void loop()
{
  //確認したい項目のコメントアウトを外してください
  value_read(); // 値読み取り
  //binary_read(); // バイナリ読取
  //sampling(); // サンプリング速度確認

  delay(100);
}
