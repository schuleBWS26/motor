#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <math.h>

#include "AudioFileSourceSD.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"

#define MOTOR_IN1 15
#define MOTOR_IN2 16
#define MOTOR_IN3 12
#define MOTOR_IN4 13

// Wahrscheinliche SPI-Belegung: IO18=SCK, IO19=MOSI, IO20=MISO, IO05=CS.
#define SD_MISO 20
#define SD_MOSI 19
#define SD_SCK 18
#define SD_CS 5
#define SD_SPI_FREQ 400000
#define MP3_FILE "/track.mp3"

#define ULTRASONIC_TRIG 14
#define ULTRASONIC_ECHO 21

#define I2S_BCLK 41
#define I2S_LRC 42
#define I2S_DOUT 40
#define AMP_SD 39

#define I2S_PORT I2S_NUM_0
#define BEEP_SAMPLE_RATE 22050
#define BEEP_BUFFER_SAMPLES 128
#define BEEP_VOLUME 2500
#define STARTUP_BEEP_MS 180
#define STARTUP_BEEP_HZ 880.0
#define SD_OK_BEEP_HZ 1175.0
#define SD_FAIL_BEEP_HZ 330.0
#define ULTRASONIC_INTERVAL_MS 500
#define ULTRASONIC_TIMEOUT_US 30000
#define MOTOR_TEST_MS 700

unsigned long lastUltrasonicMs = 0;
unsigned long lastSdRetryMs = 0;
bool sdReady = false;
bool sdRetryEnabled = true;

AudioGeneratorMP3 *mp3 = nullptr;
AudioFileSourceSD *mp3File = nullptr;
AudioOutputI2S *mp3Out = nullptr;
char selectedMp3Path[64] = MP3_FILE;

void deleteMp3Player() {
    if (mp3 != nullptr) {
        if (mp3->isRunning()) {
            mp3->stop();
        }
        delete mp3;
        mp3 = nullptr;
    }

    delete mp3File;
    mp3File = nullptr;

    if (mp3Out != nullptr) {
        mp3Out->stop();
        delete mp3Out;
        mp3Out = nullptr;
    }
}

void stopMotor() {
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    digitalWrite(MOTOR_IN3, LOW);
    digitalWrite(MOTOR_IN4, LOW);
    Serial.println("Motor aus");
}

void setMotorPins(bool in1, bool in2, bool in3, bool in4) {
    digitalWrite(MOTOR_IN1, in1 ? HIGH : LOW);
    digitalWrite(MOTOR_IN2, in2 ? HIGH : LOW);
    digitalWrite(MOTOR_IN3, in3 ? HIGH : LOW);
    digitalWrite(MOTOR_IN4, in4 ? HIGH : LOW);
}

void turnLeft() {
    setMotorPins(LOW, HIGH, HIGH, LOW);
    Serial.println("Motor Test: links");
}

void turnRight() {
    setMotorPins(HIGH, LOW, LOW, HIGH);
    Serial.println("Motor Test: rechts");
}

void runMotorStartupTest() {
    turnLeft();
    delay(MOTOR_TEST_MS);
    stopMotor();
    delay(300);

    turnRight();
    delay(MOTOR_TEST_MS);
    stopMotor();
}

void playBeep(float frequency, uint16_t durationMs) {
    pinMode(AMP_SD, OUTPUT);
    digitalWrite(AMP_SD, HIGH);

    i2s_config_t i2sConfig = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = BEEP_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BEEP_BUFFER_SAMPLES,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pinConfig = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_uninstall(I2S_PORT);
    if (i2s_driver_install(I2S_PORT, &i2sConfig, 0, nullptr) != ESP_OK ||
        i2s_set_pin(I2S_PORT, &pinConfig) != ESP_OK) {
        Serial.println("Speaker Test: I2S konnte nicht gestartet werden");
        return;
    }

    const uint32_t totalSamples = (BEEP_SAMPLE_RATE * durationMs) / 1000;
    const float phaseStep = 2.0 * PI * frequency / BEEP_SAMPLE_RATE;
    float phase = 0.0;
    uint32_t samplesDone = 0;

    while (samplesDone < totalSamples) {
        int16_t samples[BEEP_BUFFER_SAMPLES * 2];
        uint32_t blockSamples = min((uint32_t)BEEP_BUFFER_SAMPLES, totalSamples - samplesDone);

        for (uint32_t i = 0; i < blockSamples; i++) {
            int16_t sample = (phase < PI) ? BEEP_VOLUME : -BEEP_VOLUME;
            phase += phaseStep;
            if (phase >= 2.0 * PI) {
                phase -= 2.0 * PI;
            }

            samples[i * 2] = sample;
            samples[i * 2 + 1] = sample;
        }

        for (uint32_t i = blockSamples; i < BEEP_BUFFER_SAMPLES; i++) {
            samples[i * 2] = 0;
            samples[i * 2 + 1] = 0;
        }

        size_t bytesWritten = 0;
        i2s_write(I2S_PORT, samples, sizeof(samples), &bytesWritten, portMAX_DELAY);
        samplesDone += blockSamples;
    }

    i2s_zero_dma_buffer(I2S_PORT);
    i2s_driver_uninstall(I2S_PORT);
}

void playStartupBeep() {
    Serial.println("Speaker Test: kurzer Piep");
    playBeep(STARTUP_BEEP_HZ, STARTUP_BEEP_MS);
}

void playSdOkBeep() {
    playBeep(SD_OK_BEEP_HZ, 110);
    delay(120);
    playBeep(SD_OK_BEEP_HZ, 110);
}

void playSdFailBeep() {
    playBeep(SD_FAIL_BEEP_HZ, 120);
    delay(120);
    playBeep(SD_FAIL_BEEP_HZ, 120);
    delay(120);
    playBeep(SD_FAIL_BEEP_HZ, 120);
}

void printMotorPins() {
    Serial.print("Motor: IN1=IO");
    Serial.print(MOTOR_IN1);
    Serial.print(" IN2=IO");
    Serial.print(MOTOR_IN2);
    Serial.print(" IN3=IO");
    Serial.print(MOTOR_IN3);
    Serial.print(" IN4=IO");
    Serial.println(MOTOR_IN4);
}

float readDistanceCm() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    unsigned long durationUs = pulseIn(ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
    if (durationUs == 0) {
        return -1.0;
    }

    return durationUs * 0.0343 / 2.0;
}

void updateUltrasonic() {
    unsigned long nowMs = millis();
    if (nowMs - lastUltrasonicMs < ULTRASONIC_INTERVAL_MS) {
        return;
    }

    lastUltrasonicMs = nowMs;
    float distanceCm = readDistanceCm();
    if (distanceCm < 0) {
        Serial.println("Ultraschall: kein Echo");
    } else {
        Serial.print("Entfernung: ");
        Serial.print(distanceCm, 1);
        Serial.println(" cm");
    }
}

bool isMp3FileName(const char *name) {
    String lowerName(name);
    lowerName.toLowerCase();
    return lowerName.endsWith(".mp3");
}

bool findMp3OnSd(char *path, size_t pathSize) {
    if (SD.exists(MP3_FILE)) {
        strlcpy(path, MP3_FILE, pathSize);
        return true;
    }

    File root = SD.open("/");
    if (!root) {
        Serial.println("SD: Root-Verzeichnis konnte nicht geoeffnet werden");
        return false;
    }

    File entry = root.openNextFile();
    while (entry) {
        if (!entry.isDirectory() && isMp3FileName(entry.name())) {
            if (entry.name()[0] == '/') {
                strlcpy(path, entry.name(), pathSize);
            } else {
                snprintf(path, pathSize, "/%s", entry.name());
            }
            entry.close();
            root.close();
            return true;
        }
        entry.close();
        entry = root.openNextFile();
    }

    root.close();
    return false;
}

void listSdRoot() {
    File root = SD.open("/");
    if (!root) {
        Serial.println("SD: Root-Verzeichnis konnte nicht gelesen werden");
        return;
    }

    Serial.println("SD Dateien im Root:");
    File entry = root.openNextFile();
    while (entry) {
        Serial.print("  ");
        Serial.print(entry.name());
        if (!entry.isDirectory()) {
            Serial.print(" ");
            Serial.print(entry.size());
            Serial.print(" Bytes");
        }
        Serial.println();
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
}

bool setupSdCard() {
    SD.end();
    SPI.end();
    Serial.print("SD SPI: MISO=IO");
    Serial.print(SD_MISO);
    Serial.print(" MOSI=IO");
    Serial.print(SD_MOSI);
    Serial.print(" SCK=IO");
    Serial.print(SD_SCK);
    Serial.print(" CS=IO");
    Serial.println(SD_CS);

    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SPI, SD_SPI_FREQ)) {
        sdReady = false;
        Serial.println("SD: Karte nicht gefunden");
        return false;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        sdReady = false;
        Serial.println("SD: keine Karte erkannt");
        return false;
    }

    Serial.print("SD: Karte erkannt, Groesse ");
    Serial.print(SD.cardSize() / (1024 * 1024));
    Serial.println(" MB");
    listSdRoot();
    sdReady = true;
    return true;
}

bool startMp3FromSd() {
    if (!findMp3OnSd(selectedMp3Path, sizeof(selectedMp3Path))) {
        Serial.println("SD: keine MP3 gefunden, erwartet /track.mp3 oder eine .mp3 im Root");
        return false;
    }

    deleteMp3Player();
    i2s_driver_uninstall(I2S_PORT);

    pinMode(AMP_SD, OUTPUT);
    digitalWrite(AMP_SD, HIGH);

    mp3File = new AudioFileSourceSD(selectedMp3Path);
    if (!mp3File->isOpen()) {
        Serial.print("MP3: Datei konnte nicht geoeffnet werden: ");
        Serial.println(selectedMp3Path);
        deleteMp3Player();
        return false;
    }

    mp3Out = new AudioOutputI2S(0, AudioOutputI2S::EXTERNAL_I2S);
    mp3Out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    mp3Out->SetOutputModeMono(true);

    mp3 = new AudioGeneratorMP3();
    if (!mp3->begin(mp3File, mp3Out)) {
        Serial.println("MP3: Decoder konnte nicht gestartet werden");
        deleteMp3Player();
        return false;
    }

    Serial.print("MP3 spielt von SD: ");
    Serial.println(selectedMp3Path);
    return true;
}

void updateAudio() {
    if (mp3 != nullptr) {
        if (mp3->isRunning()) {
            if (!mp3->loop()) {
                mp3->stop();
            }
            return;
        }

        Serial.println("MP3 fertig, starte erneut");
        startMp3FromSd();
        return;
    }

    unsigned long nowMs = millis();
    if (sdRetryEnabled && nowMs - lastSdRetryMs >= 10000) {
        lastSdRetryMs = nowMs;
        Serial.println("SD: erneuter Versuch");
        if ((sdReady || setupSdCard()) && startMp3FromSd()) {
            return;
        }
        sdRetryEnabled = false;
        Serial.println("SD: noch keine MP3-Wiedergabe, Audio bleibt stumm");
        Serial.println("SD: Retry gestoppt, Reset druecken fuer neuen Versuch");
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    Serial.println("HC-SR04 Test gestartet");
    printMotorPins();
    stopMotor();
    playStartupBeep();
    runMotorStartupTest();

    pinMode(AMP_SD, OUTPUT);
    digitalWrite(AMP_SD, HIGH);
    if (setupSdCard()) {
        playSdOkBeep();
        if (!startMp3FromSd()) {
            Serial.println("Keine MP3 gestartet, Audio bleibt stumm");
        }
    } else {
        playSdFailBeep();
        Serial.println("Keine MP3 gestartet, Audio bleibt stumm");
    }
}

void loop() {
    updateAudio();
    updateUltrasonic();
}
