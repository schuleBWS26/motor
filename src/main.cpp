#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SD_MMC.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <math.h>

#include "AudioFileSourceFS.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"

#define MOTOR_IN1 13
#define MOTOR_IN2 12
#define MOTOR_IN3 16
#define MOTOR_IN4 15

#define SD_MISO 20
#define SD_MOSI 19
#define SD_SCK  18
#define SD_CS   5

#define ULTRASONIC_TRIG 14
#define ULTRASONIC_ECHO 21

#define I2S_BCLK 41
#define I2S_LRC  42
#define I2S_DOUT 40
#define AMP_SD   39

#define MP3_FILE "/track.mp3"
#define I2S_PORT I2S_NUM_0
#define ULTRASONIC_INTERVAL_MS 500
#define ULTRASONIC_TIMEOUT_US  30000

struct SdCardConfig {
    int miso;
    int mosi;
    int sck;
    int csPin;
    uint32_t freq;
};

SdCardConfig sdConfig;

const uint8_t motorPhase[4][4] = {
    {1, 1, 0, 0},
    {1, 1, 1, 1},
    {1, 1, 0, 0},
    {0, 0, 1, 1},
};

struct SdMmcPinMap {
    int clk;
    int cmd;
    int d0;
    const char *name;
};

const SdMmcPinMap sdMmcPinMaps[] = {
    {18, 20, 19, "CLK=IO18 CMD=IO20 D0=IO19"},
    {18, 19, 20, "CLK=IO18 CMD=IO19 D0=IO20"},
    {20, 18, 19, "CLK=IO20 CMD=IO18 D0=IO19"},
    {20, 19, 18, "CLK=IO20 CMD=IO19 D0=IO18"},
    {19, 18, 20, "CLK=IO19 CMD=IO18 D0=IO20"},
    {19, 20, 18, "CLK=IO19 CMD=IO20 D0=IO18"},
};

unsigned long lastUltrasonicMs = 0;
unsigned long lastSdRetryMs    = 0;
bool sdReady        = false;
bool sdRetryEnabled = true;
fs::FS *sdFs        = nullptr;

AudioGeneratorMP3  *mp3     = nullptr;
AudioFileSourceFS  *mp3File = nullptr;
AudioOutputI2S     *mp3Out  = nullptr;
char selectedMp3Path[64]    = MP3_FILE;

int   motorTargetSteps = 0;
bool  motorRunning     = false;
float lastDistanceCm   = -1.0f;

void driveMotor(int steps) {
    if (steps <= 0) return;
    motorRunning = true;
    for (int i = 0; i < steps; i++) {
        digitalWrite(MOTOR_IN1, motorPhase[i % 4][0]);
        digitalWrite(MOTOR_IN2, motorPhase[i % 4][1]);
        digitalWrite(MOTOR_IN3, motorPhase[i % 4][2]);
        digitalWrite(MOTOR_IN4, motorPhase[i % 4][3]);
        delay(1);
    }
    motorRunning = false;
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
    motorRunning = false;
    Serial.println("Motor aus");
}

void printMotorPins() {
    Serial.print("Motor: IN1=IO"); Serial.print(MOTOR_IN1);
    Serial.print(" IN2=IO");       Serial.print(MOTOR_IN2);
    Serial.print(" IN3=IO");       Serial.print(MOTOR_IN3);
    Serial.print(" IN4=IO");       Serial.println(MOTOR_IN4);
}

float readDistanceCm() {
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    unsigned long durationUs = pulseIn(ULTRASONIC_ECHO, LOW, ULTRASONIC_TIMEOUT_US);
    if (durationUs == 0) return -1.0f;

    return durationUs * 0.01715f;
}

void updateUltrasonic() {
    unsigned long nowMs = millis();
    if (nowMs - lastUltrasonicMs < ULTRASONIC_INTERVAL_MS) return;
    lastUltrasonicMs = nowMs;
    lastDistanceCm = readDistanceCm();
    if (lastDistanceCm < 0) {
        Serial.println("Ultraschall: kein Echo");
    } else {
        Serial.print("Entfernung: ");
        Serial.print(lastDistanceCm, 1);
        Serial.println(" cm");
    }
}

bool isMp3FileName(const char *name) {
    String s(name);
    s.toLowerCase();
    return s.endswith(".mp3");
}

void listSdRoot() {
    if (!sdFs) { Serial.println("SD: nicht bereit"); return; }
    File root = sdFs->open("/");
    if (!root) { Serial.println("SD: Root nicht lesbar"); return; }
    Serial.println("SD Dateien im Root:");
    File entry = root.openNextFile();
    while (entry) {
        Serial.print("  "); Serial.print(entry.name());
        if (!entry.isDirectory()) {
            Serial.print("  "); Serial.print(entry.size()); Serial.print(" Bytes");
        }
        Serial.println();
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
}

bool findMp3OnSd(char *path, size_t pathSize) {
    if (!sdFs) { Serial.println("SD: nicht bereit"); return false; }
    if (sdFs->exists(MP3_FILE)) { strlcpy(path, MP3_FILE, pathSize); return true; }

    File root = sdFs->open("/");
    if (!root) { Serial.println("SD: Root nicht geoeffnet"); return false; }

    File entry = root.openNextFile();
    while (entry) {
        if (!entry.isDirectory() && isMp3FileName(entry.name())) {
            if (entry.name()[0] == '/') strlcpy(path, entry.name(), pathSize);
            else snprintf(path, pathSize, "/%s", entry.name());
            entry.close(); root.close();
            return true;
        }
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
    return false;
}

bool finishSdSetup(const char *mode) {
    uint8_t cardType = (sdFs == &SD_MMC) ? SD_MMC.cardType() : SD.cardType();
    if (cardType == CARD_NONE) {
        sdReady = false; sdFs = nullptr;
        Serial.println("SD: keine Karte");
        return false;
    }
    uint64_t cardSize = (sdFs == &SD_MMC) ? SD_MMC.cardSize() : SD.cardSize();
    Serial.print("SD: erkannt via "); Serial.print(mode);
    Serial.print(", "); Serial.print((uint32_t)(cardSize / (1024 * 1024))); Serial.println(" MB");
    listSdRoot();
    sdReady = true;
    return true;
}

bool setupSdMmcCard() {
    SD_MMC.end();
    for (size_t i = 0; i < sizeof(sdMmcPinMaps) / sizeof(sdMmcPinMaps[0]); i++) {
        Serial.print("SD_MMC Versuch: "); Serial.println(sdMmcPinMaps[i].name);
        SD_MMC.end();
        if (!SD_MMC.setPins(sdMmcPinMaps[i].clk, sdMmcPinMaps[i].cmd, sdMmcPinMaps[i].d0)) {
            Serial.println("setPins fehlgeschlagen"); continue;
        }
        if (!SD_MMC.begin("/sdcard", false, false, SD_MMC_FREQ)) continue;
        sdFs = &SD_MMC;
        if (finishSdSetup("SD_MMC 1-bit")) return true;
    }
    return false;
}

bool setupSpiSdCard() {
    SD.end(); SPI.end();
    SPI.begin(sdConfig.sck, sdConfig.miso, sdConfig.mosi, sdConfig.csPin);
    if (!SD.begin(sdConfig.csPin, SPI, sdConfig.freq)) {
        sdReady = false; sdFs = nullptr;
        Serial.println("SD: nicht gefunden");
        return false;
    }
    sdFs = &SD;
    return finishSdSetup("SPI");
}

bool setupSdCard() {
    sdReady = false; sdFs = nullptr;
    if (setupSdMmcCard()) return true;
    Serial.println("SD_MMC fehlgeschlagen, probiere SPI");
    return setupSpiSdCard();
}

void deleteMp3Player() {
    if (mp3) {
        if (mp3->isRunning()) mp3->stop();
        delete mp3; mp3 = nullptr;
    }
    delete mp3File; mp3File = nullptr;
    if (mp3Out) { mp3Out->stop(); delete mp3Out; mp3Out = nullptr; }
}

bool startMp3FromSd() {
    if (!findMp3OnSd(selectedMp3Path, sizeof(selectedMp3Path))) {
        Serial.println("SD: keine MP3 gefunden");
        return false;
    }

    deleteMp3Player();
    i2s_driver_uninstall(I2S_PORT);

    pinMode(AMP_SD, OUTPUT);
    digitalWrite(AMP_SD, LOW);

    if (!sdFs) { Serial.println("MP3: kein SD-FS"); return false; }

    mp3File = new AudioFileSourceFS(*sdFs, selectedMp3Path);
    if (!mp3File->isOpen()) {
        Serial.print("MP3: nicht geoeffnet: "); Serial.println(selectedMp3Path);
        deleteMp3Player(); return false;
    }

    mp3Out = new AudioOutputI2S(0, AudioOutputI2S::INTERNAL_I2S);
    mp3Out->SetPinout(I2S_LRC, I2S_BCLK, I2S_DOUT);
    mp3Out->SetOutputModeMono(true);

    mp3 = new AudioGeneratorMP3();
    if (!mp3->begin(mp3File, mp3Out)) {
        Serial.println("MP3: Decoder fehlgeschlagen");
        deleteMp3Player(); return false;
    }

    Serial.print("MP3 laeuft: "); Serial.println(selectedMp3Path);
    return true;
}

void updateAudio() {
    if (mp3) {
        if (mp3->isRunning()) {
            if (mp3->loop()) mp3->stop();
            return;
        }
        Serial.println("MP3 fertig, starte neu");
        startMp3FromSd();
        return;
    }
    unsigned long nowMs = millis();
    if (sdRetryEnabled && nowMs - lastSdRetryMs >= 10000) {
        lastSdRetryMs = nowMs;
        Serial.println("SD: Retry...");
        if ((sdReady || setupSdCard()) && startMp3FromSd()) return;
        sdRetryEnabled = false;
        Serial.println("SD: kein Erfolg");
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    lastSdRetryMs = millis();

    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    Serial.println("System Start");
    printMotorPins();
    stopMotor();

    pinMode(AMP_SD, OUTPUT);
    digitalWrite(AMP_SD, HIGH);

    if (setupSdCard()) {
        if (!startMp3FromSd()) Serial.println("Kein Audio");
    } else {
        Serial.println("SD fehlgeschlagen");
    }
}

void loop() {
    if (!motorRunning && lastDistanceCm > 10.0f) {
        driveMotor(motorTargetSteps);
    }
    updateAudio();
    updateUltrasonic();
}