# Sensor Temperatur P10 STM32

Proyek ini adalah firmware STM32 untuk membaca suhu dari sensor MAX31855 (thermocouple) dan menampilkannya pada panel LED P10 menggunakan STM32F1 dan FreeRTOS.

---

## Rincian Pin yang Digunakan

| Nama Pin      | Port/Pin      | Fungsi                        |
|---------------|---------------|-------------------------------|
| USER_BTN      | GPIOA, PIN 0  | Tombol user                   |
| P10_A         | GPIOA, PIN 1  | Panel P10, Address A          |
| P10_B         | GPIOA, PIN 2  | Panel P10, Address B          |
| P10_OE        | GPIOA, PIN 3  | Panel P10, Output Enable      |
| P10_LAT       | GPIOA, PIN 4  | Panel P10, Latch              |
| USER_LED      | GPIOB, PIN 2  | LED indikator user            |
| SPI1_SEL      | GPIOB, PIN 12 | Chip Select MAX31855 (SPI2)   |

**Catatan:**  
- SPI2 digunakan untuk komunikasi dengan sensor MAX31855.
- SPI1 digunakan untuk komunikasi ke panel P10.

---

## Fitur yang Didukung

- Membaca suhu dari sensor MAX31855 via SPI.
- Menampilkan suhu secara real-time pada panel LED P10.
- Indikator suhu dalam bentuk angka dan bar grafis.
- Animasi marquee dan test pattern pada panel P10.
- FreeRTOS multitasking (task sensor & task display).
- Pengaturan kecerahan panel.
- Indikator LED user.

---

## Cara Instalasi di STM32CubeIDE

1. **Clone atau download** repository ini ke komputer Anda.
2. **Buka STM32CubeIDE** dan pilih `File > Open Projects from File System...`.
3. Pilih folder `sensor-temperatur-p10` sebagai project root.
4. **Build project** dengan klik kanan pada project lalu pilih `Build Project`.
5. **Hubungkan board STM32** ke komputer.
6. **Flash firmware** ke board dengan klik tombol `Debug` atau `Run`.
7. Pastikan panel P10 dan sensor MAX31855 sudah terhubung sesuai tabel pin di atas.

---

## TODO List Fitur Mendatang

- [ ] Dukungan multi-panel P10 (lebih dari 1 panel).
- [ ] tambaahkan fitur buzzer untuk alarm suhu dan notifikasi.
- [ ] Penambahan fitur WiFi untuk remote monitoring (jika hardware mendukung).
- [ ] Pengaturan kecerahan dinamis via tombol.
- [ ] Kalibrasi suhu dan offset sensor via USB/serial.
- [ ] Logging data suhu ke USB flashdisk.
- [ ] Web dashboard monitoring via USB CDC atau WiFi (jika hardware mendukung).
- [ ] Alarm suhu tinggi/rendah dengan buzzer.
- [ ] Mode display grafik suhu historis.
- [ ] Pengaturan parameter via aplikasi PC/Android.

---

**Lisensi:**  
Lihat file LICENSE dari STMicroelectronics.  