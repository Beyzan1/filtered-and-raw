#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int16_t ax, ay, az;
float filteredX = 0;
float kalmanGain = 0.02;

void setup() {
  Serial.begin(9600);
  
  Wire.begin();
  mpu.initialize();
  
  // Kalman filtresi için başlangıç değerleri
  filteredX = ax;
}

void loop() {
  mpu.getAcceleration(&ax, &ay, &az);

  // Kalman filtresi
  filteredX = kalmanGain * ax + (1 - kalmanGain) * filteredX;

  Serial.print(ax);
  Serial.print(",");
  Serial.println(filteredX);

  delay(100);
}



*******************************************************************************************************
import serial
import matplotlib.pyplot as plt

ser = serial.Serial('COM4', 9600)  # Port adını ve baud hızını ayarlayın

x_raw = []
x_filtered = []
t = []

plt.ion()  # Canlı grafik için etkileşimli modu aç

fig, ax = plt.subplots()
line_raw, = ax.plot(t, x_raw, 'b', label='Ham Veri')
line_filtered, = ax.plot(t, x_filtered, 'r', label='Filtrelenmiş Veri')
ax.set_xlabel('Zaman')
ax.set_ylabel('X ekseni')
ax.set_title('Ham ve Filtrelenmiş Verilerin Grafik Gösterimi')
ax.legend()
ax.grid(True)

while True:
    try:
        data = ser.readline()
        ax.figure.canvas.draw()  # Grafikleri güncelle
        if data:
            ax.set_xlim(0, len(t) + 1)  # Zamanı güncelle
            t.append(len(t))
            values = data.decode('utf-8').split(",")
            if len(values) == 2:
                x_raw.append(float(values[0]))
                x_filtered.append(float(values[1]))
            line_raw.set_data(t, x_raw)  # Ham veriyi güncelle
            line_filtered.set_data(t, x_filtered)  # Filtrelenmiş veriyi güncelle
            ax.relim()
            ax.autoscale_view(True, True, True)
            ax.figure.canvas.flush_events()
    except KeyboardInterrupt:
        break

plt.ioff()  # İnteraktif modu kapat
plt.show()
ser.close()
