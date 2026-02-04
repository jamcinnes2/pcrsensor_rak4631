// MIT License
//
// Copyright (c) 2026 John Andrew McInnes
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Remote LoRa PCR sensor (C) 2024,2025
// by John McInnes
// todo: test/use RxBoosted
// todo: can we measure channel background interference?
// todo: decouple watchdog from sleep interval
// todo: get SX126X packet freq. error, see: https://github.com/jgromes/RadioLib/blob/cf776230a448faf7414f2809e12b95b68ada9014/src/modules/SX126x/SX126x.cpp#L1279
// todo: test ripple on JMXM125 V+
// todo: detect MAC collision
// todo: low voltage cutoff
// todo: blink LED command
// todo: save configuration to NV memory
// todo: use / set RTC in CmdSetInterval
//
// 2024-11-24 JAM v0
// 2024-12-01 JAM optimizing for low power
// 2025-01-18 JAM changing radar wake pin and LoRa radio config
// 2025-04-01 JAM new 2 way comms, HolyHop, CAD, & more! big changes
// 2025-05-24 JAM enable Open SD Logger on I2C bus
//

// NOTE: ***I have modifed the sx126x lib, and adafruit_sleepydog lib***
// NOTE: ***A backup is in JAM's ~/backup folder***
// NOTE: If you do an Arduino debug build, expect that soft-reset arduino-ide flashing will not work,
//       and that you will get Bluefruit init messages printed to Serial.

//#define CFG_DEBUG 1  // what is this? arduino? rtos?
#define DEBUG_PCR  // this build uses a bit more battery w/ the debug messages
#ifdef DEBUG_PCR
#pragma message("DEBUG build on")
#endif

#include <array>
#include <algorithm>
#include <Arduino.h>
//#include <Wire.h>
//#include <SPI.h>
#include <SX126x-RAK4630.h>      // JAM: I made a change to sx126x/radio.cpp:880 to support 62.5kHz LowDatarateOptimization!!
#include <Adafruit_SleepyDog.h>  // JAM: I made a change to support longer sleep time
#include "debugmsg.h"
#include "holyhop.h"
#include "radarsensor.h"
#include "btcomms.h"
#include "bsputil.h"
#include "cbtimer.h"
#include "threadsafequeue.h"
#include "loracomms.h"
#include "sdcard.h"
//#include <assert.h>

//#define RAK12500_GNSS
#ifdef RAK12500_GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS gGNSS;
#endif

/// set firmware version here:
const uint16_t SENSOR_FIRMWARE_VERSION = 0x0084; // dev version
//const uint16_t SENSOR_FIRMWARE_VERSION = 0x01082; // production version

// some constants
#define PIN_VBATT WB_A0
uint32_t vbatt_pin = PIN_VBATT;
#define VBATT_MV_PER_LSB (0.73242188F)  // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBATT_DIVIDER_COMP (1.73f)      // Compensation factor for the VBAT divider, depend on the board
#define REAL_VBATT_MV_PER_LSB (VBATT_DIVIDER_COMP * VBATT_MV_PER_LSB)

// RAK4630 has two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif
#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif
#define LED_PIN LED_BUILTIN

#define _RAKBASE_19007_
// on rak minibase 19003
#ifdef _RAKBASE_19003_
#define RADARWAKE_PIN PIN_SERIAL2_TX  // nRF PO1.6 / AC11 / UART2 TX
// on rak base 19007
#else
#define WIS_3V3S_PIN WB_IO2
#define RADARWAKE_PIN WB_IO1
#endif

#define SENSOR_WAKEMEASURE_TIMEMS (6000)               // max time in ms it might take to: enablesensors,take measurement,be ready to TX
#define MAX_DEEPSLEEP_TIMEMS (6 * 60 * 60 * 1000)      // 6 hours
#define WATCHDOG_TIMEMS (MAX_DEEPSLEEP_TIMEMS + 9000)  // max sleep + 9 seconds deadline to feed wd
#define DEFAULT_MEASUREMENT_TIMEMS (5*60*1000)         // 5 minutes
#define DEFAULT_TELEMETRY_TXSEC (5 * 60)               //(HOLYHOP_DEFAULT_TLM_INTERVAL_SEC) // a default interval until we hear from Base
#define MAX_UNSYNC_TIMEMS (60 * 60 * 1000)             // if in unsynchronized mode this long take a break from active RX & do Radio.sleep
#define UNSYNC_SLEEP_TIMEMS (4 * 60 * 60 * 1000)       // Radio.sleep for this long after MAX_UNSYNC_TIMEMS
static_assert(DEFAULT_MEASUREMENT_TIMEMS <= MAX_DEEPSLEEP_TIMEMS);
static_assert(DEFAULT_TELEMETRY_TXSEC >= DEFAULT_MEASUREMENT_TIMEMS / 1000);

// sensor configuration
struct SensorConfigPCR_t {
    char sensor_name[HOLYHOP_NAME_SIZE] = "noname";    // Ground Truth Alaska id tag
    HHDeviceAddress_t hh_did;                          // our hh address a.k.a. device id
    LoRaConfiguration_t lora_config;
    int32_t measurement_interval_ms = DEFAULT_MEASUREMENT_TIMEMS;
    int32_t telemetry_interval_sec = DEFAULT_TELEMETRY_TXSEC;
    int32_t telemetry_offset_ms = 0;  // offset in ms from wake time for sending tlm. helps avoid TX collisions
    RadarConfiguration_t rad_config;
} SensorConfig;

// sensor measurements
struct RadarSensorMeasurement_t {
    int16_t vbatt_mV; // in millivolts    
    RadarReturn_t rdist;        // "narrow" measurement
    RadarReturn_t wide_rdist;   // "wide-look" distance and strength
    int16_t temp_celsius;
    std::time_t time_stamp;
};

struct RadarSensorMeasurementArray_t{
    RadarSensorMeasurementArray_t():
        MmtCount(0){};

    void Clear(){
        MmtCount = 0;
    }

    bool Store( RadarSensorMeasurement_t &rad_mmt ){
        if ( MmtCount >= MEASUREMENT_ARRAY_SZ )
            return false;
        MmtArray[ MmtCount++ ] = rad_mmt;
        return true;
    }

    RadarSensorMeasurement_t GetLatest(){
        RadarSensorMeasurement_t rad_mmt = {};
        if( MmtCount > 0 )
            rad_mmt = MmtArray[MmtCount-1];
        return rad_mmt;
    }

    static const uint8_t MEASUREMENT_ARRAY_SZ = 50;
private:
    uint8_t MmtCount;
    RadarSensorMeasurement_t MmtArray[MEASUREMENT_ARRAY_SZ];
} gRadarMeasurement;

struct GNSSMeasurement_t{
    int32_t latitude, longitude, altitude;
    uint8_t fix_type;
} gLatestGNSS = {};

// comms variables
static RadioEvents_t RadioEvents = {};
int LastRSSI = 0;
int LastSNR = 0;
volatile uint16_t gLoRaCADCount = 0;
ThreadSafeQueue<HHQueuedPacket_t> gPktSendQueue(10);  // queued pkts for TX

//volatile bool gfWakeIntervalSync = false;             // true once we get a valid interval from Base
//CBTimer gUnsyncTimer;                                 // measure how long we are in Unsynchronized state

// class to manage wake interval sync and timer
class HHSyncState_t{
public:
    HHSyncState_t() :
        fWakeInterval(false){}

    void SetStateUnsync(){
        fWakeInterval = false;
        if ( !UnsyncTimer.isRunning() )
            UnsyncTimer.start();
    }

    void SetStateInSync(){
        fWakeInterval = true;
        UnsyncTimer.stop();
    }

    bool IsSynchronized(){
        return fWakeInterval;
    }

    // @returns true if we should take a nap and give up on sync for a while
    bool IsNapTime(){
        // reset UnsyncTimer if needed
        if ( !fWakeInterval && UnsyncTimer.elapsed_ms() > UNSYNC_SLEEP_TIMEMS ){
            TSLogs << "DBG UnsyncTimer reset\n";
            UnsyncTimer.start();
        }

        bool fnap = !fWakeInterval &&
            UnsyncTimer.elapsed_ms() > MAX_UNSYNC_TIMEMS &&
            UnsyncTimer.elapsed_ms() <= UNSYNC_SLEEP_TIMEMS;
        return fnap;
    }

private:
    std::atomic<bool> fWakeInterval;        // true when we get a valid interval from Base (CMD_SET_INTERVAL)
//    std::atomic<bool> fPendingSync;       // true if we are awaiting a sync result (CMD_SET_INTERVAL)
    CBTimer UnsyncTimer;                    // measure how long we are unsynchronized

} gHHSync;

void happy_blink_worker(void *pvParameters);
// blink the lights in a way that shows success!
void do_happy_blink(){
    xTaskCreate(
        happy_blink_worker,            // Task function
        "Happy Blink Worker",          // Task name (for debugging)
        configMINIMAL_STACK_SIZE * 2,  // Stack size (adjust as needed)
        NULL,                          // Parameter passed to task
        2,                             // Priority (higher number = higher priority)
        NULL                           // Optional task handle
    );
};

void prc_blink_leds( const JsonDocument &data_in, JsonDocument &data_out ){
    do_happy_blink();
}

void prc_configure_radar( const JsonDocument &data_in, JsonDocument &data_out ){    
    auto din = data_in[PRC_CALLDOC]; // get nested input data
    RadarConfiguration_t rcfg;
    rcfg.start_dist_mm = din["start_dist_mm"];
    rcfg.end_dist_mm = din["end_dist_mm"];
    rcfg.max_profile = din["max_profile"];
    rcfg.peak_sorting = din["peak_sorting"];
    rcfg.threshold_method = din["threshold_method"];
    rcfg.threshold_sensitivity = din["threshold_sensitivity"];
    rcfg.fixed_amp_threshold = din["fixed_amp_threshold"];
    rcfg.fixed_str_threshold = din["fixed_str_threshold"];
    rcfg.signal_quality = din["signal_quality"];
    rcfg.max_step_count = din["max_step_count"];
    rcfg.reflector_shape = din["reflector_shape"];
    rcfg.close_range_leakage = din["close_range_leakage"] ? 1 : 0;

    bool fsuccess = true;//configure_radar(rcfg);
    data_out[PRC_RESULTDOC]["success_flag"] = fsuccess;
    // if good radar configuration, save it
    if (fsuccess){
        SensorConfig.rad_config = rcfg;
    }
}

// @brief configure the lora radio params
// @return false on failure
bool ConfigureLoRa(LoRaConfiguration_t &lcfg) {
#ifdef DEBUG_PCR
    TSLogs << "ConfigureLoRa: " << lcfg.frequency_hz << "," << lcfg.tx_power_dbm << "," << lcfg.bandwidth_code
        << "," << lcfg.spreading_factor << "," << lcfg.codingrate_code << "\n";
#endif
    // seed random num gen.
    uint32_t rseed = Radio.Random();  // (must do this here before radio config, it resets radio config)
    TSLogs.Fmt("random seed from Radio: %d\n", rseed);
    randomSeed(rseed);

    // Set Radio channel
    if (!Radio.CheckRfFrequency(lcfg.frequency_hz)) {
        TSLogs << "Bad LoRa frequency: " << lcfg.frequency_hz << "\n";
        return false;
    }
    Radio.SetChannel(lcfg.frequency_hz);

    // Set Radio TX configuration
    //Radio.SetLowDataRate(LORA_LOW_DATA_RATE);
    Radio.SetTxConfig(MODEM_LORA, lcfg.tx_power_dbm, 0, lcfg.bandwidth_code,
                      lcfg.spreading_factor, lcfg.codingrate_code,
                      DEFAULT_LORA_PREAMBLE_LENGTH, DEFAULT_LORA_FIX_LENGTH_PAYLOAD,
                      DEFAULT_LORA_CRC, 0, 0, DEFAULT_LORA_IQ_INVERSION, DEFAULT_LORA_TX_TIMEOUT);

    // Set Radio RX config
    Radio.SetRxConfig(MODEM_LORA, lcfg.bandwidth_code, lcfg.spreading_factor,
                      lcfg.codingrate_code, 0, DEFAULT_LORA_PREAMBLE_LENGTH, DEFAULT_LORA_SYMBOL_TIMEOUT,
                      DEFAULT_LORA_FIX_LENGTH_PAYLOAD, 0, DEFAULT_LORA_CRC, 0, 0, DEFAULT_LORA_IQ_INVERSION,
                      true);
    //Radio.RxBoosted
    Radio.SetPublicNetwork(false);
    return true;
}

// debug this is for debugging
void PrintTaskHandle() {
    TaskHandle_t xCurrentTask = xTaskGetCurrentTaskHandle();
    TSLogs << "Current Thread Handle: " << (long unsigned int)xCurrentTask << "\n";
}

// LoRa RadioEvents
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cad_result);

// HolyHop Events
void OnHHCommandReset(PacketCommandReset_t *);
void OnHHCommandGetVersion(PacketCommandGetVersion_t *);
void OnHHCommandGetName(PacketCommandGetName_t *);
void OnHHCommandSetName(PacketCommandSetName_t *);
void OnHHCommandGetInterval(PacketCommandGetInterval_t *);
void OnHHCommandSetInterval(PacketCommandSetInterval_t *);
void OnHHCommandConfigLoRa(PacketCommandConfigLoRa_t *);
void OnHHCommandConfigRadar(PacketCommandConfigRadar_t *);
void OnHHCommandGetRadarConfig(PacketCommandGetRadarConfig_t *);
void OnHHCommandBeginDFUUpload(PacketCommandBeginDFUUpload_t *);
void OnHHCommandDFUUpload(PacketCommandDFUUpload_t *);
void OnHHCommandVerifyDFUUpload(PacketCommandVerifyDFUUpload_t *);
void OnHHCommandBlinkLED(PacketCommandBlinkLED_t *);
void OnHHCommandGetPreferredUplink(PacketCommandGetPreferredUplink_t *);
void OnHHCommandSetPreferredUplink(PacketCommandSetPreferredUplink_t *);
void OnHHTelemetry(const HHDeviceAddress_t &src_id, uint8_t base_hops);
void OnHHRelayPkt(PacketHeader_t *ppkt_hdr, uint16_t size);

void DoSensorMeasurement();
void SendLatestTelemetry();

//////////////////
// Some FreeRTOS timers & functions
//

// @brief get milliseconds until next timer event/expiry
uint32_t MSUntilNextTimerEv(TimerHandle_t handle){
    if ( !xTimerIsTimerActive(handle) )
        return 0;

    TickType_t expiry_tk = xTimerGetExpiryTime(handle);
    TickType_t now_tk = xTaskGetTickCount();

    // Tick counter may wrap – use modulo arithmetic
    TickType_t ticks_left = (expiry_tk > now_tk) ? (expiry_tk - now_tk) : 0;
    uint32_t ms_left = static_cast<uint32_t>(ticks_left * 1000 / configTICK_RATE_HZ);
    return ms_left;
}

bool gfSystemResetNow = false;  // set true to trigger reset after packet TX

// Track and Calc. time in UNIX Epoch seconds
static const uint64_t TIME_OVERFLOW_MS = (uint64_t)0xFFFFFFFF * 1000 / configTICK_RATE_HZ;
static std::time_t gBootTimeEpochSec = 0;
// @brief system uptime in seconds based on tick count and that accounts for rollover.
uint64_t GetRAK4630UptimeSec() {
    static uint32_t tick_rollover_count = 0;
    static uint32_t last_count_ms = 0;
    uint32_t tick_count = xTaskGetTickCount();
    uint32_t count_ms = tick2ms(tick_count); //uint32_t count_ms = tick_count * 1000 / configTICK_RATE_HZ;
        
    // detect rollover
    if (count_ms < last_count_ms)
        ++tick_rollover_count;
    last_count_ms = count_ms;

    // calc. rollover amount
    uint64_t overflow_ms = tick_rollover_count * TIME_OVERFLOW_MS;
    uint64_t up_sec = (overflow_ms + count_ms) / 1000;

    return up_sec;
}

// @brief get the current time in UNIX epoch seconds, because std::time(0) doesn't work on arduino
std::time_t GetRAK4630Time() {
    uint64_t uptime_sec = GetRAK4630UptimeSec();
    return gBootTimeEpochSec + uptime_sec;
}

// @brief set the current time in UNIX epoch seconds
void SetRAK4630Time(std::time_t nowt) {
    // calculate when we booted based on this time
    gBootTimeEpochSec = nowt - GetRAK4630UptimeSec();
}

// if GetRAK4630Time isn't called often enough we will miss tickcount rollovers:
static_assert(MAX_DEEPSLEEP_TIMEMS < TIME_OVERFLOW_MS);

// @brief Get current date time UTC as string
void GetDateTimeStrN(char *pstr, size_t slen) {
    time_t t = GetRAK4630Time();
    tm my_tm;
    localtime_r(&t, &my_tm);
    size_t sz = strftime(pstr, slen, "%Y-%m-%d %H:%M:%S+0000", &my_tm);
}

// deep sleep timer
SemaphoreHandle_t gSemTaskEvent = NULL;
SoftwareTimer gTelemetryWakeupTimer;
SoftwareTimer gMeasureWakeupTimer;
SoftwareTimer gRXWindowTimer;

// sensor wake up reasons
bool gfWakeForTelemetry = false;
bool gfWakeForMeasure = false;
uint16_t gWakeCount = 0;

// @brief Timer event that wakes up the loop task for sending telemetry
// @param unused
void TelemetryWakeupTEv(TimerHandle_t unused) {
    gfWakeForTelemetry = true;
    // Give the semaphore, so the loop task will wake up
    xSemaphoreGiveFromISR(gSemTaskEvent, pdFALSE);
}

// @brief Timer event that wakes up the loop task for taking measurements
// @param unused
void MeasureWakeupTEv(TimerHandle_t unused) {
    gfWakeForMeasure =  true;
    // Give the semaphore, so the loop task will wake up
    xSemaphoreGiveFromISR(gSemTaskEvent, pdFALSE);
}


// consolidate timer access to these function to avoid bugs mixing ms and seconds
void BeginTaskWakeupTimers() {
    uint32_t interval_ms = SensorConfig.telemetry_interval_sec * 1000;
    gTelemetryWakeupTimer.begin(interval_ms, TelemetryWakeupTEv);
    gTelemetryWakeupTimer.start();

    // start measurement timer as well
    uint32_t measure_ms = SensorConfig.measurement_interval_ms;
    gMeasureWakeupTimer.begin(measure_ms, MeasureWakeupTEv);
    gMeasureWakeupTimer.start();
}

void SetTelemetryWakeupTimerMS(uint32_t interval_ms) {
    gTelemetryWakeupTimer.setPeriod(interval_ms);  // this also restarts timer
}

void ResetTaskWakeupTimers() {
    uint32_t interval_ms = SensorConfig.telemetry_interval_sec * 1000;
    SetTelemetryWakeupTimerMS(interval_ms);

    uint32_t measure_ms = SensorConfig.measurement_interval_ms;
    gMeasureWakeupTimer.setPeriod(measure_ms);
}

// @brief Timer event that ends RX mode and sleeps the radio.
// @param unused
void RXWindowEndsTEv(TimerHandle_t unused) {
    
    // if we are in sync with base, Radio sleep
    if ( gHHSync.IsSynchronized() ) {
        // can we just radio.sleep() from this ISR? is semaphore needed?
        // are we interrupting a pkt RX in the middle of reception?
        TSLogs << "RXWindowEndsTEv: Synchronized with Base, Radio.Sleep\n";
        TSLogs.flush();
        Radio.Sleep();
    }    
    // if we are NOT insync and have not been insync for a long time, Radio sleep
    else if ( gHHSync.IsNapTime() ){
        TSLogs << "RXWindowEndsTEv: Unsynchronized for too long, Radio.Sleep\n";
        TSLogs.flush();
        Radio.Sleep();
    }
    // else restart RX timer
    else {
        TSLogs << "RXWindowEndsTEv: reset timer\n";
        TSLogs.flush();
        gRXWindowTimer.reset();
    }
}

void BeginRXWindowTimer(){
    uint32_t interval_ms = HOLYHOP_RX_WINDOW_SEC * 1000;
    interval_ms += SENSOR_WAKEMEASURE_TIMEMS;                         // we will wake up early to give time to wake & measure.
    gRXWindowTimer.begin(interval_ms, RXWindowEndsTEv, NULL, false);  // non-repeating
    // timer will be start()ed when needed.
}

//////////////////
// get battery voltage
//
float readVBATT() {
    float raw;
    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbatt_pin);
    return raw * REAL_VBATT_MV_PER_LSB;
}

// Convert from raw battery mv to percentage
uint8_t mvToPercent(float mvolts) {
    if (mvolts < 3300)
        return 0;
    if (mvolts < 3600) {
        mvolts -= 3300;
        return mvolts / 30;
    }

    mvolts -= 3600;
    return 10 + (mvolts * 0.15f);  // thats mvolts /6.66666666
}

///////////////////
// setup/shutdown sensors & peripherals
//
void EnableSensors() {
    // wisblock switched 3V3 sensor supply on
    digitalWrite(WIS_3V3S_PIN, HIGH);
    delay(150); // give things time to power on

    // I2C bus on. this will set the needed pinModes too.
    Wire.begin();

    // we used to wake radar sensor. now we completely turn if off/on. so no need to 'wake' it here
    //digitalWrite(RADARWAKE_PIN, HIGH);    

    // setup the Radar sensor with normal/narrow configuration
    TSLogs << "Setup radar\n";
    setup_radar();
    if (!configure_radar(SensorConfig.rad_config)) {
        TSLogs << "configure_radar failed!\n";
    }

    // setup sdcard access
    if(!sdcard_begin()){
        TSLogs << "sdcard_begin failed! Please make sure the card is inserted & reboot.\n";
    }

#ifdef RAK12500_GNSS
    // setup GNSS
    if ( !gGNSS.begin()){   // Connect to the u-blox module using I2C
        TSLogs << "u-blox GNSS not detected at default I2C address!\n";
    }
    else{
        gGNSS.setI2COutput(COM_TYPE_UBX);   //Set the I2C port to output UBX only (turn off NMEA noise)
        gGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR    
    }
#endif
    TSLogs << "EnabledSensors\n";
}

void DisableSensors() {
    sdcard_end();

    // disable I2C bus & pullups for power saving (or the QwiicOpenLog will draw power from I2C pins)
    Wire.end();
    pinMode(PIN_WIRE_SDA, INPUT);
    pinMode(PIN_WIRE_SCL, INPUT);

    // wisblock switched 3V3 sensor supply off
    digitalWrite(WIS_3V3S_PIN, LOW);
    delay(200); // whats this for? waiting for 'things to settle'?    
    TSLogs << "DisabledSensors\n";
}

/*
// for debugging & testing dfu process
#include "carray_0111.h"
void TestDFUNow(){
    uint8_t *pdfu_src = ___pcrsensor_rak4631_slim_build_rakwireless_nrf52_WisCoreRAK4631Board_pcrsensor_rak4631_slim_0111_bin;
    uint32_t dfu_len = ___pcrsensor_rak4631_slim_build_rakwireless_nrf52_WisCoreRAK4631Board_pcrsensor_rak4631_slim_0111_bin_len;    
    uint16_t dfu_crc = crc16( pdfu_src, dfu_len );

    // test/debug DFU
    TSLogs.Fmt( "Testing DFU in 5 seconds. DFU len: %d, CRC calculated: %04X\n", dfu_len, dfu_crc );
    delay(5000);

    TSLogs << "begin write\n";
    if ( !begin_bank1_write( dfu_len, dfu_crc ) ){
        TSLogs << "bank1_begin_write failed!\n";
        return;
    }
    delay(1000);

    if ( !bank1_write( pdfu_src, dfu_len ) ){
        TSLogs << "bank1_write failed!\n";
        return;
    }
    delay(1000);
    
    TSLogs << "Verifying DFU\n";
    if ( !bank1_dfu_verify(pdfu_src) ){
        TSLogs << "bank1_dfu_verify failed!\n";
        return;
    }
    delay(1000);

    TSLogs << "Activating DFU\n";
    if ( !bank1_dfu_activate() ){
        TSLogs << "bank1_dfu_activate failed\n";
        return;
    }
    TSLogs << "SUCCESS! ready to reset.\n";
    // else{
    //     Serial.println("Resetting now..");
    //     Serial.flush();
    //     delay(500);  // give our serial msg time to print
    //     NVIC_SystemReset();
    //     return;
    // }
    delay(5000);
}
*/

//////////////////////
// Arduino sketch setup
//
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // led1 on during setup
    pinMode(LED_BUILTIN2, OUTPUT);
    pinMode(WIS_3V3S_PIN, OUTPUT);
    pinMode(RADARWAKE_PIN, OUTPUT);
    // set wisblock switched 3V3 sensor supply off - power cycling sensors
    digitalWrite(WIS_3V3S_PIN, LOW);
    gWakeCount = 0;
    gHHSync.SetStateUnsync();
    gfWakeForTelemetry = false;
    gfWakeForMeasure = false;
    gfSystemResetNow = false;
    int wdt_time_ms = Watchdog.enable(WATCHDOG_TIMEMS);

    // Initialize Serial for debug output. It takes time on some platforms like NRF.
    time_t timeout = millis();
    Serial.begin(115200);
    while (!Serial) {
        if ((millis() - timeout) < 5000)
            delay(100);
        else
            break;
    }
    // print boot up messages
    char ver_str[20];
    // snprintf(ver_str, sizeof(ver_str), "%d.%02d",
    //          get_version_major(SENSOR_FIRMWARE_VERSION),
    //          get_version_minor(SENSOR_FIRMWARE_VERSION));
    snprintf(ver_str, sizeof(ver_str), "0x%04X", SENSOR_FIRMWARE_VERSION);
    TSLogs << "---------------------------\n";
    TSLogs.Fmt("LoRa PCR Sensor starting v %s\n", ver_str);
    TSLogs << "---------------------------\n";
    print_hwinfo();
    print_fwinfo();
    TSLogs << "wdt_time_ms: " << wdt_time_ms << "\n";
    PrintTaskHandle();  // debug
    TSLogs.Fmt("HOLYHOP version 0x%04X\n", HOLYHOP_VERSION);
    TSLogs.flush();
    delay(100);  // let serial flush. some crashes are so fast we dont get the above printed out.

    // wisblock switched 3V3 sensor supply on
    delay(100);  // make sure we waited at least this long
    digitalWrite(WIS_3V3S_PIN, HIGH);

    // init Wire lib
    Wire.begin();
    //Wire.setClock(100000); //Wire.setClock(400000); // limit I2C bus speed if slow sensors
    delay(10);

    // Setup to read battery voltage
    analogReference(AR_INTERNAL_3_0);  // Set the analog reference to 3.0V (default = 3.6V)
    analogReadResolution(12);          // 12-bit resolution
    delay(1);                          // Let the ADC settle
    readVBATT();                       // Get a single ADC sample and throw it away

    // setup bluetooth
    bool fbt_dfu = true;
    bt_setup(fbt_dfu);

    // use bluetooth MAC to create unique hh device id for ourselves
    ble_gap_addr_t bt_addr = Bluefruit.getAddr();
    SensorConfig.hh_did.SetID(KnownOUI_t::OUI_RAK_NRF52, bt_addr.addr[2], bt_addr.addr[1], bt_addr.addr[0]);

    TSLogs << "bluefruit BT MAC: ";
    for (int i = sizeof(bt_addr.addr) - 1; i >= 0; --i) {
        uint8_t the_byte = bt_addr.addr[i];
        TSLogs.Fmt("%02X ", the_byte);
    }
    TSLogs << "\n";
    TSLogs << "HOLYHOP DID: " << SensorConfig.hh_did.c_str() << "\n";

    // // test/debug DFU
    // TestDFUNow();

    // setup holyhop
    HHEvents_t hh_callbacks = {};
    hh_callbacks.OnCommandReset = OnHHCommandReset;
    hh_callbacks.OnCommandGetVersion = OnHHCommandGetVersion;
    hh_callbacks.OnCommandGetName = OnHHCommandGetName;
    hh_callbacks.OnCommandSetName = OnHHCommandSetName;
    hh_callbacks.OnCommandGetInterval = OnHHCommandGetInterval;
    hh_callbacks.OnCommandSetInterval = OnHHCommandSetInterval;
    hh_callbacks.OnCommandConfigLoRa = OnHHCommandConfigLoRa;
    hh_callbacks.OnCommandConfigRadar = OnHHCommandConfigRadar;
    hh_callbacks.OnCommandGetRadarConfig = OnHHCommandGetRadarConfig;
    hh_callbacks.OnCommandBeginDFUUpload = OnHHCommandBeginDFUUpload;
    hh_callbacks.OnCommandDFUUpload = OnHHCommandDFUUpload;
    hh_callbacks.OnCommandVerifyDFUUpload = OnHHCommandVerifyDFUUpload;
    hh_callbacks.OnCommandBlinkLED = OnHHCommandBlinkLED;
    hh_callbacks.OnCommandGetPreferredUplink = OnHHCommandGetPreferredUplink;
    hh_callbacks.OnCommandSetPreferredUplink = OnHHCommandSetPreferredUplink;
    hh_callbacks.OnTelemetry = OnHHTelemetry;
    hh_callbacks.OnRelayPkt = OnHHRelayPkt;
    InitHH(false, SensorConfig.hh_did, hh_callbacks);

    // setup radar sensor
    EnableSensors();  // sensors on

    // init LoRa chip
    TSLogs << "Initialize LoRa chip\n";
    lora_rak4630_init();
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;
    TSLogs << "Initialize LoRa radio\n";
    Radio.Init(&RadioEvents);
    TSLogs << "Configure LoRa radio\n";
    ConfigureLoRa(SensorConfig.lora_config);

    // Create the Wake event semaphore and start wakeup timer.
    // We use the default wake interval until we hear from Base.
    // Radio does not sleep until we hear from Base and synchronize.
    gSemTaskEvent = xSemaphoreCreateBinary();
    xSemaphoreGive(gSemTaskEvent);      // must give to initialize semaphore
    xSemaphoreTake(gSemTaskEvent, 10);  // take the semaphore so the main loop() will go to sleep until an event happens
    BeginTaskWakeupTimers();

    // Create and start RX window timer
    BeginRXWindowTimer();
    gRXWindowTimer.start();
/*
// DEBUG - loop taking measurements at 1 Hz & write to sd card
while( 1 ){
    Watchdog.reset();  // must feed watchdog, no WDT disabler on nRF52 or we would use: Watchdog.disable();
    DoSensorMeasurement();
    sdcard_flush();
    delay(1000);
}
*/
    // Take a measurement and send a packet. This sets radio mode as well.
    Watchdog.reset();  // must feed watchdog, no WDT disabler on nRF52 or we would use: Watchdog.disable();
    TSLogs.flush();    // flush any important debug messages
    DoSensorMeasurement();

    // turn sensors off, send tlm, led off
    DisableSensors();
    SendLatestTelemetry();
    digitalWrite(LED_PIN, LOW);
}

// @brief take a measurement, assumes sensors are powered on & ready
void DoSensorMeasurement() {
    TSLogs << "DoSensorMeasurement\n"; TSLogs.flush();
    RadarSensorMeasurement_t rad_mmt;
    rad_mmt.vbatt_mV = readVBATT();
    
    // radar should already be configured for narrow measurement in EnableSensors(). do it.
    do_radar(rad_mmt.rdist);
    
    // do wide-look measurement
    if (!configure_radar_wide() ){
        TSLogs << "configure_radar_wide failed!\n";
    }
    do_radar(rad_mmt.wide_rdist);

    // read temp. and store measurement
    rad_mmt.temp_celsius = read_radar_temp();
    rad_mmt.time_stamp = GetRAK4630Time();
    gRadarMeasurement.Store( rad_mmt );

    int32_t base_hops = GetHHBaseHops();  // if we have no uplink this will be 0
    uint32_t uplink_dw = GetHHUplinkAddr().ToDW();

#ifdef RAK12500_GNSS
    // get GNSS    
    TSLogs << "GNSS fix ok: " << gGNSS.getGnssFixOk() << "\n";
    TSLogs << "GNSS fix type: " << gGNSS.getFixType() << "\n";
    TSLogs << "GNSS latitude: " << gGNSS.getLatitude() << "\n";
    TSLogs << "GNSS longitude: " << gGNSS.getLongitude() << "\n";
    TSLogs << "GNSS altitude: " << gGNSS.getAltitude() << "\n";
    TSLogs << "GNSS ground speed: " << gGNSS.getGroundSpeed() << "\n";
    TSLogs << "GNSS heading: " << gGNSS.getHeading() << "\n";
    TSLogs << "GNSS SIV: " << gGNSS.getSIV() << "\n";
    if ( gGNSS.getGnssFixOk() ){
        gLatestGNSS.latitude = gGNSS.getLatitude();
        gLatestGNSS.longitude = gGNSS.getLongitude();
        gLatestGNSS.altitude = gGNSS.getAltitude();
        gLatestGNSS.fix_type = gGNSS.getFixType();
    }
    else{
        gLatestGNSS = {};
    }
#endif

    // log it
    char date_str[50] = "fail";
    GetDateTimeStrN(date_str, sizeof(date_str));

    char tlm_str[256];
#ifdef RAK12500_GNSS
    // Radar telemetry with GNSS
    TSLogs << "Measurement type FULL RADAR WITH GNSS\n";
    snprintf(tlm_str, sizeof(tlm_str),
        "%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%08X,%d,%d,%d,%d",
        date_str,
        SensorConfig.sensor_name,
        gWakeCount, rad_mmt.vbatt_mV, LastRSSI, LastSNR,
        rad_mmt.rdist.d0, rad_mmt.rdist.d1, rad_mmt.rdist.d2,
        rad_mmt.rdist.d3, rad_mmt.rdist.d4, rad_mmt.rdist.d5, 
        rad_mmt.rdist.d6, rad_mmt.rdist.d7, rad_mmt.rdist.d8, rad_mmt.rdist.d9,
        rad_mmt.rdist.s0, rad_mmt.rdist.s1, rad_mmt.rdist.s2, 
        rad_mmt.rdist.s3, rad_mmt.rdist.s4, rad_mmt.rdist.s5, 
        rad_mmt.rdist.s6, rad_mmt.rdist.s7, rad_mmt.rdist.s8, rad_mmt.rdist.s9, 
        rad_mmt.temp_celsius,
        base_hops, uplink_dw,
	    gLatestGNSS.latitude, gLatestGNSS.longitude, gLatestGNSS.altitude, gLatestGNSS.fix_type );
#else
    // plain Radar telemetry
    TSLogs << "Measurement type RADAR\n";
    snprintf(tlm_str, sizeof(tlm_str),
        "%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%08X",
        date_str,
        SensorConfig.sensor_name,
        gWakeCount, rad_mmt.vbatt_mV, LastRSSI, LastSNR,
        rad_mmt.rdist.d0, rad_mmt.rdist.d1, rad_mmt.rdist.d2, rad_mmt.rdist.s0,
        rad_mmt.wide_rdist.d0, rad_mmt.wide_rdist.d1, rad_mmt.wide_rdist.d2, rad_mmt.wide_rdist.s0,
        rad_mmt.temp_celsius,
        base_hops, uplink_dw );
#endif

    TSLogs.Time() << "telemetry: " << tlm_str << "\n";
    TSLogs << "Logging to sdcard.\n"; TSLogs.flush();
    sdcard_log_it(tlm_str);
}

/// @brief sensors on, take measurement, send it, sensors off
void TakeOneMeasurement() {
    EnableSensors();
    DoSensorMeasurement();
    DisableSensors();
}

// @brief do the RX window: relay for other nodes, send our telemetry
void DoTelemetry(){
    // Start RX window and enable RX while we wait our turn to TX telemetry.
    // Because we might need to relay telemetry for another node.
    gRXWindowTimer.reset();
    CBTimer rxw_elapsed_ms;
    rxw_elapsed_ms.start();
    StartRXMode();  //todo: this clobbers any packet that hasn't finished TX. its ok.
    TSLogs << "******Sensor RX Window Open******\n";
    PrintTaskHandle();  // debug

    // delay until time to send Telemetry. (radio thread still running so yes we can block here)
    int32_t tlm_delay = SensorConfig.telemetry_offset_ms - rxw_elapsed_ms.elapsed_ms();
    TSLogs << "DEBUG tlm_delay: " << tlm_delay << "ms\n";
    if (tlm_delay >= 0) {
        delay(tlm_delay);
    } else {
        TSLogs << "DEBUG We are late on TX telemetry! tlm_delay: " << tlm_delay << "ms\n";
    }

    // see if we have uplink
    bool fuplink = UpdateHHNetworkState();
    // if no uplink and we ARE synchronized, then
    if ( !fuplink && gHHSync.IsSynchronized() ) {
        // we might be out of contact or out of sync. go to unsync mode.
        // Downlinks will see that we are out of contact from our base_hops tlm being == 0.
        gHHSync.SetStateUnsync();
        TSLogs << "No Uplink, going to unsynchronized mode.\n";
    }        

    // stop any Radio RX and send our telemetry
    Radio.Standby();
    SendLatestTelemetry();
}

/////////////////////////
/// @brief Arduino loop task. Called by the FreeRTOS task handler
//
void loop() {
    TSLogs << "*** loop() top\n\n";
    // Sleep until we are woken up by an event
    if (xSemaphoreTake(gSemTaskEvent, portMAX_DELAY) == pdTRUE){
        Watchdog.reset();  // feed the watch dog
        // Switch on grn LED to show we are awake
        digitalWrite(LED_BUILTIN, HIGH);
        // Switch on blue LED to show if we ARE in sync and have uplink
        if ( gHHSync.IsSynchronized() && IsHHUplinkValid() )
            digitalWrite(LED_BUILTIN2, HIGH);

        ++gWakeCount;
        // uint32_t current_tick = xTaskGetTickCount();
        // uint32_t ct_ms = current_tick * 1000 / configTICK_RATE_HZ;
        // Serial.printf("DEBUG current tick %d, in ms %d\n", current_tick, ct_ms );

        // do flagged work
        //
        if ( gfWakeForTelemetry ){
            TSLogs << "***WAKE for Telemetry\n";
            // Enforce timer period: CmdSetInterval can temporarliy change the wake timer interval
            // ..to keep us in sync. So fix it if necessary. Do it before doing other things.
            TickType_t tmr_ticks = xTimerGetPeriod(gTelemetryWakeupTimer.getHandle());
            uint32_t tmr_ms = tmr_ticks * 1000 / configTICK_RATE_HZ;
            uint32_t tlm_ms = SensorConfig.telemetry_interval_sec * 1000;
            if (tmr_ms != tlm_ms) {
                TSLogs << "Enforcing tlm timer period. current " << tmr_ticks << "ticks "
                        << tmr_ms << "ms, cfg is " << tlm_ms << "ms\n";
                ResetTaskWakeupTimers();  // restart wake timers NOW
            }

            // now send telemetry
            DoTelemetry();
            gfWakeForTelemetry = false;
        }
        if ( gfWakeForMeasure ){
            TSLogs << "***WAKE for Measurement\n";
            // todo: dont interrupt ongoing CAD or TX
            // See if we have time to take a measurement. Telemetry is timing sensitive and we
            // don't want to delay it while taking measurements.
            uint32_t telemetry_next_ms = MSUntilNextTimerEv( gTelemetryWakeupTimer.getHandle() );
            TSLogs << "***NEXT telemetry in " << telemetry_next_ms << "\n";
            if ( telemetry_next_ms > SENSOR_WAKEMEASURE_TIMEMS ){
                // sensors on, take a measurement, sensors off
                TakeOneMeasurement();
                gfWakeForMeasure = false;
            }
            // else: Measurement will still be flagged and will happen after next Telemetry
        }
        
        // loop thread back to sleep & leds off
        xSemaphoreTake(gSemTaskEvent, 10); // why is this here? The SemaphoreTake is at top of loop.
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(LED_BUILTIN2, LOW);
    }
    Watchdog.reset();  // feed the watch dog
}

// @brief Put radio in RX mode and start/restart RX window timer
// ..we only listen for incoming packets for a while.
void StartRXMode() {
    TSLogs << "StartRXMode\n";
    Radio.Rx(0);
    // If we set preamble longer than 16 we could make use of power saving RX.
    // (Less than 16 with minsymbols=8 means radio can't sleep at all during RX)
    //StartRXDutyCycleAuto(Radio, SensorConfig.lora_config);

    // check system reset flag
    if (gfSystemResetNow) {
        DoSystemReset();  // no return from this!
    }
}

// @brief Keep RXing or not
void UpdateRXMode() {
    TSLogs << "UpdateRXMode... ";
    // make sure we aren't already in a callback for CAD or TX
    if (Radio.GetStatus() != RadioState_t::RF_IDLE && Radio.GetStatus() != RadioState_t::RF_RX_RUNNING) {
        TSLogs << "not in RX or IDLE mode.\n";
        return;
    }

    // A timer event puts the radio to sleep so dont worry about doing that.
    // So a rxerror or rxtimeout must have happened. Stay awake listening
    TSLogs << "stay awake.\n";
    Radio.Rx(0);
}

/// @brief do channel activity detection. if channel clear we will TX
void StartLoRaCAD() {
    TSLogs << "StartLoRaCAD\n";
    // make sure we aren't already in a callback for CAD or TX
    if (Radio.GetStatus() == RadioState_t::RF_CAD || Radio.GetStatus() == RadioState_t::RF_TX_RUNNING) {
        TSLogs << "Active CAD or TX already\n";
        return;
    }
    uint8_t cad_symbol_num = LORA_CAD_08_SYMBOL;
    uint8_t cad_det_peak = SensorConfig.lora_config.spreading_factor + 13;
    uint8_t cad_det_min = 10;
    uint8_t cad_exit_mode = LORA_CAD_ONLY;
    uint32_t cad_timeout = 0;  //DEFAULT_LORA_CAD_TIMEOUT; // <--THIS IS ONLY USED WITH LORA_CAD_RX MODE.

    // if we are not synchronized, random wait BEFORE CAD. helps avoid collision deadlock w/ other nodes.
    if ( !gHHSync.IsSynchronized() ){
        int random_ms = random(DEFAULT_LORA_CAD_RANDOM_WAIT + LORA_TURNAROUND_DELAYMS);
        delay(random_ms);
    }
    else {
        // block for some ms. Give us & other nodes time to switch from TX to RX so they will RX our pkt.
        // The rpi w/ sx1262 needs quite a bit of time for some reason.
        delay(LORA_TURNAROUND_DELAYMS);
    }

    // Start CAD
    Radio.Standby();
    Radio.SetCadParams(cad_symbol_num, cad_det_peak, cad_det_min, cad_exit_mode, cad_timeout);
    Radio.StartCad();
}

/// @brief if there is a pkt queued for send, send it!
void SendLoRaPkt() {
    TSLogs.Fmt("SendLoRaPkt Q size: %d\n", gPktSendQueue.size());
    HHQueuedPacket_t qpkt;
    if (!gPktSendQueue.pop(qpkt, pdMS_TO_TICKS(2)))  // pop() with timeout
        return;

    // copy packet to our send buffer
    uint8_t send_buf[LORA_MAX_PKT_SIZE];
    std::copy(qpkt.pkt_array.begin(), qpkt.pkt_array.end(), send_buf);

    uint32_t onair_ms = Radio.TimeOnAir(MODEM_LORA, qpkt.pkt_size);
    RadioState_t radio_status = Radio.GetStatus();
    TSLogs.Fmt("TX LoRa packet %d bytes %dms on air, radio status %d\n",
                  qpkt.pkt_size, onair_ms, radio_status);

    // LED2 on and send it
    digitalWrite(LED_BUILTIN2, HIGH);
    Radio.Send(send_buf, qpkt.pkt_size);
    // Radio goes to TX mode
}

/// @brief Radio channel activity detection done event
void OnCadDone(bool cad_result) {
    Watchdog.reset();  // feed the watch dog
    TSLogs << "OnCadDone\n";
    PrintTaskHandle();  // debug

    // if channel busy
    if (cad_result){
        // wait a random amount of time
        int random_ms = random(DEFAULT_LORA_CAD_RANDOM_WAIT) + 250;
        delay(random_ms);
        // try a few more times
        if (gLoRaCADCount++ < DEFAULT_LORA_CAD_TRY_COUNT) {
            TSLogs.Fmt("LoRa CAD count %d waited %dms\n", gLoRaCADCount, random_ms);
            StartLoRaCAD();
            return;
        }
        TSLogs << "LoRa CAD count maxed out!\n";
    }
    // channel is clear, or we ran out of retries. send the packet.
    gLoRaCADCount = 0;
    SendLoRaPkt();
    // radio goes from CAD to TX mode
}

void DoSystemReset() {
    // reboot now
    TSLogs << "Resetting now...\n";
    TSLogs.flush();
    delay(200);  // give our serial msg time to print
    NVIC_SystemReset();
}

// @brief Radio Tx Done event
void OnTxDone(void) {
    Watchdog.reset();  // feed the watch dog
    // LED2 off
    digitalWrite(LED_BUILTIN2, LOW);
    TSLogs << "OnTxDone\n";
    PrintTaskHandle();  // debug

    // After transmitting a packet, Send more or put Radio to RX mode
    if (!gPktSendQueue.isEmpty()) {
        StartLoRaCAD();
    } else {
        StartRXMode();
    }
}

// @brief Radio Tx Timeout event
void OnTxTimeout(void) {
    Watchdog.reset();  // feed the watch dog
    // LED2 off
    digitalWrite(LED_BUILTIN2, LOW);
    TSLogs << "OnTxTimeout\n";
    PrintTaskHandle();  // debug

    // We failed to transmit a packet, Send more or put Radio to RX mode
    if (!gPktSendQueue.isEmpty()) {
        StartLoRaCAD();
    } else {
        StartRXMode();
    }
}

// @brief received good LoRa packet
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    Watchdog.reset();  // feed the watch dog
    LastRSSI = rssi;
    LastSNR = snr;

    TSLogs.Fmt("RX LoRa packet %d bytes RSSI %d dBm SNR %d dB\n", size, rssi, snr);
    PrintTaskHandle();  // debug
#ifdef DEBUG_PKTS
    TSLogs << "RX raw: ";
    for (uint16_t i = 0; i < size; i++) {
        TSLogs.Fmt("%02X", payload[i]);
        if (i < size) TSLogs << ",";
    }
    TSLogs << "\n";
#endif

    // check for valid HH header
    bool fpkt_for_us = false;
    if (size < sizeof(PacketHeader_t)) {
        TSLogs << "RX invalid HHPacket header! ignored.\n";
    } else {
        PacketHeader_t *ppkt_hdr = reinterpret_cast<PacketHeader_t *>(payload);
        HHDeviceAddress_t src_addr(ppkt_hdr->Src);
        HHDeviceAddress_t dest_addr(ppkt_hdr->Dest);
        HHDeviceAddress_t relayto_addr(ppkt_hdr->RelayTo);
        HHDeviceAddress_t relayby_addr(ppkt_hdr->RelayBy);

        // debug whitelisting addresses
        uint32_t *white_list = nullptr;
        uint32_t wl_len = 0;
        /*
        uint32_t wlist_0A8[] = {
            0x02000001,
            0x0A5C4D24,
            0x0A8654E2,
            0x0AD9F2F0,
            0x0A1928C9,
            0x0A6633F7,
        };
        uint32_t wlist_0A6[] = {
            0x02000001,
            //0x0A5C4D24,
            //0x0A8654E2,
            //0x0AD9F2F0
            //0x0A1928C9,
            //0x0A6633F7,
        };
        uint32_t wlist_0A5[] = {
            //0x02000001,
            0x0A5C4D24,
            0x0A8654E2,
            //0x0AD9F2F0
            0x0A1928C9,
            //0x0A6633F7,
        };
        uint32_t wlist_0A1[] = {
            //0x02000001,
            0x0A5C4D24,
            //0x0A8654E2,
            0x0AD9F2F0,
            0x0A1928C9,
            //0x0A6633F7,
        };
        uint32_t wlist_0AD[] = {
            //0x02000001,
            //0x0A5C4D24,
            //0x0A8654E2,
            0x0AD9F2F0,
            0x0A1928C9,
            //0x0A6633F7,
        };

        if (SensorConfig.hh_did == 0x0A8654E2) {
            white_list = wlist_0A8;
            wl_len = sizeof(wlist_0A8) / sizeof(uint32_t);
        } else if (SensorConfig.hh_did == 0x0A6633F7) {
            white_list = wlist_0A6;
            wl_len = sizeof(wlist_0A6) / sizeof(uint32_t);
        } else if (SensorConfig.hh_did == 0x0A5C4D24) {
            white_list = wlist_0A5;
            wl_len = sizeof(wlist_0A5) / sizeof(uint32_t);
        } else if (SensorConfig.hh_did == 0x0A1928C9) {
            white_list = wlist_0A1;
            wl_len = sizeof(wlist_0A1) / sizeof(uint32_t);
        } else if (SensorConfig.hh_did == 0x0AD9F2F0) {
            white_list = wlist_0AD;
            wl_len = sizeof(wlist_0AD) / sizeof(uint32_t);
        } else {
            TSLogs << "DBG no whitelist.\n";
        }
        */
        bool faccept = false;
        if (white_list) {
            for (int i = 0; i < wl_len; i++) {
                uint32_t dwid = white_list[i];
                TSLogs.Fmt("dwid %08X relayby %s\n", dwid, relayby_addr.c_str());
                if (dwid == relayby_addr.ToDW()) {
                    faccept = true;
                    break;
                }
            }
        } else {
            faccept = true;
        }

        if (faccept) {  // debug
            TSLogs.Fmt("** Packet Src %s, Dest %s, RelayBy %s, RelayTo %s\n",
                       src_addr.c_str(), dest_addr.c_str(), relayby_addr.c_str(), relayto_addr.c_str());
            TSLogs.Fmt("** Packet Type: (%d) %s, %d hops\n",
                       ppkt_hdr->PktType, GetHHPktTypeName(ppkt_hdr->PktType), ppkt_hdr->HopCount);

            if (!ProcessHHPkt(ppkt_hdr, size, rssi, snr)) {
                TSLogs << "ProcessHHPkt failed.\n";
            } else if (relayto_addr == SensorConfig.hh_did) {
                // good pkt that is to US, restart RX timer & keep receiving
                TSLogs << "OnRxDone good process pkt: reset RX timer\n";
                TSLogs.flush();
                gRXWindowTimer.reset();
            }
        }  //debug whitelist
        else {
            TSLogs << "@@@@@@@@ DROPPED PKT @@@@@@@@@ from " << relayby_addr << "\n";
        }
    }

    // After receiving a packet and not sending or CAD, begin receive window again.
    // Check first though because in some cases we are sending from within this recv handler!
    TSLogs << "OnRxDone() radio status: " << Radio.GetStatus() << "\n";
    UpdateRXMode();
}

// @brief radio rx timeout event
void OnRxTimeout() {
    Watchdog.reset();  // feed the watch dog
    TSLogs << "OnRxTimeout\n";
    PrintTaskHandle();  // debug
    UpdateRXMode();
}

// @brief radio rx error event
void OnRxError() {
    Watchdog.reset();  // feed the watch dog
    TSLogs << "OnRxError\n";
    PrintTaskHandle();  // debug
    UpdateRXMode();
}

// @brief send latest telemetry
void SendLatestTelemetry() {
    // todo: average measurements?
    RadarSensorMeasurement_t rad_mmt = gRadarMeasurement.GetLatest();
    gRadarMeasurement.Clear();
    uint8_t base_hops = GetHHBaseHops();  // if we have no uplink this will be 0

#ifdef RAK12500_GNSS    
    PacketRadarGNSSTelemetry_t pkt(base_hops);
    pkt.Payload.wake_count = gWakeCount;
    pkt.Payload.batt_voltage = rad_mmt.vbatt_mV;
    pkt.Payload.last_rssi = LastRSSI;
    pkt.Payload.last_snr = LastSNR;
    pkt.Payload.radar_dist0 = rad_mmt.rdist.d0;
    pkt.Payload.radar_dist1 = rad_mmt.rdist.d1;
    pkt.Payload.radar_dist2 = rad_mmt.rdist.d2;
    pkt.Payload.radar_dist3 = rad_mmt.rdist.d3;
    pkt.Payload.radar_dist4 = rad_mmt.rdist.d4;
    pkt.Payload.radar_dist5 = rad_mmt.rdist.d5;
    pkt.Payload.radar_dist6 = rad_mmt.rdist.d6;
    pkt.Payload.radar_dist7 = rad_mmt.rdist.d7;
    pkt.Payload.radar_dist8 = rad_mmt.rdist.d8;
    pkt.Payload.radar_dist9 = rad_mmt.rdist.d9;
    pkt.Payload.radar_strength0 = rad_mmt.rdist.s0;
    pkt.Payload.radar_strength1 = rad_mmt.rdist.s1;
    pkt.Payload.radar_strength2 = rad_mmt.rdist.s2;
    pkt.Payload.radar_strength3 = rad_mmt.rdist.s3;
    pkt.Payload.radar_strength4 = rad_mmt.rdist.s4;
    pkt.Payload.radar_strength5 = rad_mmt.rdist.s5;
    pkt.Payload.radar_strength6 = rad_mmt.rdist.s6;
    pkt.Payload.radar_strength7 = rad_mmt.rdist.s7;
    pkt.Payload.radar_strength8 = rad_mmt.rdist.s8;
    pkt.Payload.radar_strength9 = rad_mmt.rdist.s9;
    pkt.Payload.temp_c = rad_mmt.temp_celsius;
    pkt.Payload.uplink = GetHHUplinkAddr().ToDW();
    // GNSS
    pkt.Payload.latitude = gLatestGNSS.latitude;
    pkt.Payload.longitude = gLatestGNSS.longitude;
    pkt.Payload.altitude = gLatestGNSS.altitude;
    pkt.Payload.fix_type = gLatestGNSS.fix_type;
#else
    PacketRadarTelemetry_t pkt(base_hops);
    pkt.Payload.wake_count = gWakeCount;
    pkt.Payload.batt_voltage = rad_mmt.vbatt_mV;
    pkt.Payload.last_rssi = LastRSSI;
    pkt.Payload.last_snr = LastSNR;
    pkt.Payload.radar_dist0 = rad_mmt.rdist.d0;
    pkt.Payload.radar_dist1 = rad_mmt.rdist.d1;
    pkt.Payload.radar_dist2 = rad_mmt.rdist.d2;
    pkt.Payload.radar_strength0 = rad_mmt.rdist.s0;
    pkt.Payload.radarwide_dist0 = rad_mmt.wide_rdist.d0;
    pkt.Payload.radarwide_dist1 = rad_mmt.wide_rdist.d1;
    pkt.Payload.radarwide_dist2 = rad_mmt.wide_rdist.d2;
    pkt.Payload.radarwide_strength0 = rad_mmt.wide_rdist.s0;
    pkt.Payload.temp_c = rad_mmt.temp_celsius;
    pkt.Payload.uplink = GetHHUplinkAddr().ToDW();
#endif
    PacketDeviceID_t dst_id = PacketDeviceID_t::GenericBaseID();
    QueueLoRaPkt(pkt, dst_id);
}

// @brief Queue a generic HolyHop packet for transmission
template<class TPacket>
void QueueLoRaPkt(TPacket &pkt, PacketDeviceID_t &dst_id) {
    static_assert(sizeof(pkt) <= LORA_MAX_PKT_SIZE);
    uint16_t pkt_size = sizeof(pkt);
    // set packet destination & source device ids
    // we are the source and origin of this packet.
    pkt.Header.Dest = dst_id;
    pkt.Header.Src = SensorConfig.hh_did.ToDID();
    pkt.Header.RelayBy = SensorConfig.hh_did.ToDID();
    pkt.Header.HopCount = 1;  // this is it's first hop

    // check our nodeinfo data to see if we need relay to this Destination.
    HHDeviceAddress_t relay_to;
    if (GetHHNextHopFor(dst_id, relay_to))
        pkt.Header.RelayTo = relay_to.ToDID();
    else
        pkt.Header.RelayTo = dst_id;  // we don't know how to reach. try direct.

    HHDeviceAddress_t dsta_id(pkt.Header.Dest);
    HHDeviceAddress_t srca_id(pkt.Header.Src);
    TSLogs.Fmt("Queue LoRa packet to %s from us %s size %d bytes\n",
               dsta_id.c_str(), srca_id.c_str(), pkt_size);
    TSLogs.Fmt("Packet Type: (%d) %s, %d hops\n",
               pkt.Header.PktType, GetHHPktTypeName(pkt.Header.PktType), pkt.Header.HopCount);

    if (!relay_to.IsNull())
        TSLogs.Fmt("Relay To %s\n", relay_to.c_str());

    // enqueue packet
    HHQueuedPacket_t qpkt = {
        pkt.Header.PktType,
        pkt_size
    };
    uint8_t *psrc = reinterpret_cast<uint8_t *>(&pkt);
    std::copy(psrc, psrc + pkt_size, qpkt.pkt_array.begin());
    gPktSendQueue.push(qpkt);

    // do LoRa ChannelActivityDetection to see if channel is clear right now
    StartLoRaCAD();
    // Radio goes to CAD mode. TX will happen after.
}

// @brief queue a HolyHop command reply packet for transmission
template<class TPacket>
void QueueLoRaCmdReply(TPacket &pkt, PacketDeviceID_t &dst_id) {
// this is built into Packet constructor now:   pkt.ref_id = cmd_ref_id;  // enforce setting the command ref_id
    QueueLoRaPkt(pkt, dst_id);
}

// @brief a relay has been requested for this packet. we are not the source or destination.
void OnHHRelayPkt(PacketHeader_t *ppkt_hdr, uint16_t pkt_size) {
    // check our nodeinfo data to see if we need relay to this Destination or if it is direct.
    HHDeviceAddress_t dest_addr(ppkt_hdr->Dest);
    HHDeviceAddress_t relay_to;
    if (GetHHNextHopFor(ppkt_hdr->Dest, relay_to)) {
        // make sure it isn't bouncing back
        if (relay_to.IsMatch(ppkt_hdr->RelayBy)) {
            TSLogs << "Packet dropped because it tried to loop.\n";
            return;
        }
        ppkt_hdr->RelayTo = relay_to.ToDID();
    } else {
        // we don't know how to reach that node. drop.
        TSLogs << "Packet dropped because we dont have next hop.\n";
        return;
    }
    // increase the packet hop count & set relay_by to us.
    ++ppkt_hdr->HopCount;
    if (ppkt_hdr->HopCount > HOLYHOP_MAX_HOP_COUNT) {
        TSLogs.Fmt("Packet dropped. Too many hops: %d.\n", ppkt_hdr->HopCount);
        return;
    }

    ppkt_hdr->RelayBy = SensorConfig.hh_did.ToDID();
    TSLogs.Fmt("Queue relaypkt to %s for destination %s\n", relay_to.c_str(), dest_addr.c_str());

    // enqueue packet
    HHQueuedPacket_t qpkt = {
        ppkt_hdr->PktType,
        pkt_size
    };
    uint8_t *psrc = reinterpret_cast<uint8_t *>(ppkt_hdr);
    std::copy(psrc, psrc + pkt_size, qpkt.pkt_array.begin());
    gPktSendQueue.push(qpkt);

    // do LoRa ChannelActivityDetection to see if channel is clear right now
    StartLoRaCAD();
    // Radio goes to CAD mode. TX will happen after.
}

void OnHHCommandReset(PacketCommandReset_t *ppkt) {
    TSLogs << "OnHHCommandReset\n";
    // reply success. will trigger reset after packet is sent.
    gfSystemResetNow = true;
    PacketReplySuccess_t reply_pkt(ppkt->RefID);
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
    return;
}

void OnHHCommandGetVersion(PacketCommandGetVersion_t *ppkt) {
    TSLogs.Fmt("OnHHCommandGetVersion, Remote protocol version: 0x%04X, Our version: 0x%04X\n",
                  ppkt->Payload.hh_version, HOLYHOP_VERSION);

    PacketReplyVersion_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.fw_version = SENSOR_FIRMWARE_VERSION;
    reply_pkt.Payload.bl_version = get_actual_bootload_ver();
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

void OnHHCommandGetName(PacketCommandGetName_t *ppkt) {
    TSLogs << "OnHHCommandGetName\n";
    PacketReplyName_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.name.Set(SensorConfig.sensor_name, strlen(SensorConfig.sensor_name));
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

void OnHHCommandSetName(PacketCommandSetName_t *ppkt) {
    TSLogs << "OnHHCommandSetName\n";
    strncpy(SensorConfig.sensor_name, ppkt->Payload.name.Get(), sizeof(SensorConfig.sensor_name));
    TSLogs << "Set new sensor_name: " << SensorConfig.sensor_name << "\n";
    PacketReplyName_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.name.Set(SensorConfig.sensor_name, strlen(SensorConfig.sensor_name));
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

void OnHHCommandGetInterval(PacketCommandGetInterval_t *ppkt) {
    TSLogs << "OnHHCommandGetInterval\n";
    TSLogs.Fmt("Our measurement_interval_ms is %d, wake_interval_sec is %d\n",
                  SensorConfig.measurement_interval_ms, SensorConfig.telemetry_interval_sec);

    PacketReplyInterval_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.measurement_ms = SensorConfig.measurement_interval_ms;
    reply_pkt.Payload.tlm_int_sec = SensorConfig.telemetry_interval_sec;
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

// HAPPY BLINK! (tm) worker thread
void happy_blink_worker(void *pvParameters) {
    int delay_ms = 300;
    int delay_ticks = pdMS_TO_TICKS(delay_ms);
    // Task's work loop goes here
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN2, LOW);
    vTaskDelay(delay_ticks);  // Delay to yield CPU
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN2, HIGH);
    vTaskDelay(delay_ticks);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN2, LOW);
    vTaskDelay(delay_ticks);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN2, HIGH);
    vTaskDelay(delay_ticks);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN2, LOW);
    vTaskDelay(delay_ticks);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN2, HIGH);
    vTaskDelay(delay_ticks);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN2, LOW);
    vTaskDelay(delay_ticks);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN2, HIGH);
    vTaskDelay(delay_ticks);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN2, HIGH);
    vTaskDelay(delay_ticks);
    // leave both leds off
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN2, LOW);
    // Tasks must not return - delete if breaking from loop
    vTaskDelete(NULL);
}

// new interval too low or high?
bool IsMeasureIntervalOK(int measurement_ms) {
    if (measurement_ms < 1000 || measurement_ms > MAX_DEEPSLEEP_TIMEMS) {
        return false;
    }
    return true;
}

bool IsTlmIntervalOK(int int_sec, int next_ms) {
    if (int_sec < 30 || int_sec > (MAX_DEEPSLEEP_TIMEMS / 1000)) {
        return false;
    }
    if (next_ms < 0 || next_ms > MAX_DEEPSLEEP_TIMEMS) {
        return false;
    }
    return true;
}

bool IsTlmOffsetOK(uint32_t tlm_offset_ms, int int_sec) {
    if (tlm_offset_ms >= int_sec * 1000 * 4) {
        return false;  // seems like that is way too long
    }
    return true;
}

// @brief timing & synchronization settings from Base
void OnHHCommandSetInterval(PacketCommandSetInterval_t *ppkt) {
    TSLogs << "OnHHCommandSetInterval\n";
    // set system RTC date & time
    std::time_t time_change = std::difftime(ppkt->Payload.etime_sec, GetRAK4630Time());
    UpdateHHDatetimes(time_change);
    //setenv("TZ", "AKST9AKDT,M3.2.0,M11.1.0", 1 );
    SetRAK4630Time(ppkt->Payload.etime_sec);
    TSLogs.Time() << "<---System time set.\n";

    // We are going to calculate when we should wake up. We need to be awake when the
    // RX window opens so we can relay traffic (if needed) and be ready to send our telemetry.
    // Also we want to have enought time beforehand to take at least one sensor measurement first.
    // So we will wake up before the RX window.
    //
    // compensate for pkt travel time. assume it wasn't CAD delayed or retried.
    int hop_count = ppkt->Header.HopCount;
    int pkt_toa_ms = Radio.TimeOnAir(MODEM_LORA, sizeof(PacketRadarTelemetry_t));
    int relay_ms = pkt_toa_ms + LORA_TURNAROUND_DELAYMS + LORA_CAD_DELAYMS;
    int travel_offset_ms = hop_count * relay_ms;

    // compensate for wakeup and 1 measurement time
    int wakeup_measure_ms = SENSOR_WAKEMEASURE_TIMEMS;  // radar sensor

    // Using these numbers calc time to next interval start (for us anyway) in milliseconds.
    // NOTE this does not include the wait for tlm_offset_ms. That is in the loop() function.
    int32_t next_wake_ms = ppkt->Payload.next_tlm_ms - travel_offset_ms - wakeup_measure_ms;
    // wrap around to next interval if needed. we can't fire a timer event in the past.
    if (next_wake_ms < 0) {
        next_wake_ms = ppkt->Payload.tlm_int_sec * 1000 + next_wake_ms;
    }

    // compensate tlm_offset_ms for waking early to do a measurement
    int32_t tlm_offset_ms = ppkt->Payload.tlm_offset_ms + wakeup_measure_ms;

    TSLogs.Fmt("pkt next_tlm_ms: %dms\n", ppkt->Payload.next_tlm_ms);
    TSLogs.Fmt("Estimated pkt travel time: %dms, est. wake and measure time: %dms\n",
               travel_offset_ms, wakeup_measure_ms);
    TSLogs.Fmt("Compensated next_wake_ms: %dms\n", next_wake_ms);
    TSLogs.Fmt("Compensated tlm_offset_ms: %dms\n", tlm_offset_ms);

    // new interval too low or high?
    if (!IsMeasureIntervalOK(ppkt->Payload.measurement_ms) ||
        !IsTlmIntervalOK(ppkt->Payload.tlm_int_sec, next_wake_ms) ||
        !IsTlmOffsetOK(tlm_offset_ms, ppkt->Payload.tlm_int_sec)) {
        TSLogs << "Failure. Intervals are out of bounds.\n";
        TSLogs.Fmt("measurement_ms %d, telemetry_interval_sec %d, telemetry_next_ms %d, telemetry_offset_ms %d\n",
                   ppkt->Payload.measurement_ms, ppkt->Payload.tlm_int_sec,
                   next_wake_ms, tlm_offset_ms);

        PacketReplyFailure_t reply_pkt(ppkt->RefID);
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
        return;
    }

    // A sync command from base! The packet's RelayBy will be our uplink
    if ( SetHHUplink(ppkt->Header.RelayBy) ){
        gHHSync.SetStateInSync();
    }

    // set new interval
    SensorConfig.measurement_interval_ms = ppkt->Payload.measurement_ms;
    SensorConfig.telemetry_interval_sec = ppkt->Payload.tlm_int_sec;
    SensorConfig.telemetry_offset_ms = tlm_offset_ms;

    TSLogs.Time() << "**********  **********  **********\n";
    TSLogs.Fmt("Synchronizing to base: next tlm/wake/interval in %dms, tlm interval is %ds\n",
               next_wake_ms, SensorConfig.telemetry_interval_sec);

    // Wake on time for next Base interval. After wake up, loop will set the correct repeating interval ms.
    Watchdog.reset();  // feed watchdog since we will be waking up at a new unknown interval.
    SetTelemetryWakeupTimerMS(next_wake_ms);
    // restart measurement timer
    gMeasureWakeupTimer.setPeriod(SensorConfig.measurement_interval_ms);

    TSLogs.Fmt("New measurement_interval_ms %d, telemetry_interval_sec %d, tlm_offset_ms %d\n",
                  SensorConfig.measurement_interval_ms, SensorConfig.telemetry_interval_sec,
                  SensorConfig.telemetry_offset_ms);

    // reply success!
    PacketReplyInterval_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.measurement_ms = SensorConfig.measurement_interval_ms;
    reply_pkt.Payload.tlm_int_sec = SensorConfig.telemetry_interval_sec;
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);

    // do a HAPPY BLINK! we are in contact with base.
    do_happy_blink();    
}

void OnHHCommandGetPreferredUplink(PacketCommandGetPreferredUplink_t *ppkt) {
    TSLogs << "OnHHCommandGetPreferredUplink\n";
    // reply
    PacketReplyPreferredUplink_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.uplink = GetHHPreferredUplinkAddr().ToDW();
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

void OnHHCommandSetPreferredUplink(PacketCommandSetPreferredUplink_t *ppkt) {
    TSLogs << "OnHHCommandSetPreferredUplink\n";    
    // reply
    PacketReplyPreferredUplink_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.uplink = GetHHPreferredUplinkAddr().ToDW();
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

void OnHHCommandConfigLoRa(PacketCommandConfigLoRa_t *ppkt) {
    // received a new radio configuration
    TSLogs << "OnHHCommandConfigLoRa\n";
    LoRaConfiguration_t new_lorac;
    new_lorac.frequency_hz = ppkt->Payload.frequency_hz;
    new_lorac.tx_power_dbm = ppkt->Payload.tx_power_dbm;
    new_lorac.bandwidth_code = ppkt->Payload.bandwidth_code;
    new_lorac.spreading_factor = ppkt->Payload.spreading_factor;
    new_lorac.codingrate_code = ppkt->Payload.codingrate_code;
    // try it and send reply
    if (ConfigureLoRa(new_lorac)) {
        TSLogs << "ConfigureLoRa good.\n";
        SensorConfig.lora_config = new_lorac;
        // this reply will be under new LoRa config!!
        PacketReplySuccess_t reply_pkt(ppkt->RefID);
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
        // todo: start new lora config timer and revert if no acknowledgement
    }
    // else put current config back in place
    else {
        TSLogs << "ConfigureLoRa bad.\n";
        ConfigureLoRa(SensorConfig.lora_config);
        PacketReplyFailure_t reply_pkt(ppkt->RefID);
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
    }
}

void OnHHCommandConfigRadar(PacketCommandConfigRadar_t *ppkt) {
    // received a new radar configuration. try it
    TSLogs << "OnHHCommandConfigRadar\n";
    RadarConfiguration_t new_radc;
    new_radc.start_dist_mm = ppkt->Payload.start_dist_mm;
    new_radc.end_dist_mm = ppkt->Payload.end_dist_mm;
    new_radc.max_profile = ppkt->Payload.max_profile;
    new_radc.peak_sorting = ppkt->Payload.peak_sorting;
    new_radc.threshold_method = ppkt->Payload.threshold_method;
    new_radc.threshold_sensitivity = ppkt->Payload.threshold_sensitivity;
    new_radc.fixed_amp_threshold = ppkt->Payload.fixed_amp_threshold;
    new_radc.fixed_str_threshold = ppkt->Payload.fixed_str_threshold;
    new_radc.signal_quality = ppkt->Payload.signal_quality;
    new_radc.max_step_count = ppkt->Payload.max_step_count;
    new_radc.reflector_shape = ppkt->Payload.reflector_shape;
    new_radc.close_range_leakage = ppkt->Payload.close_range_leakage;

    // RADAR ON to try configuration
    EnableSensors();

    // if success keep config and send reply
    if (configure_radar(new_radc)) {
        TSLogs << "configure_radar good.\n";
        SensorConfig.rad_config = new_radc;

        PacketReplyRadarConfig_t reply_pkt(ppkt->RefID);
        reply_pkt.Payload.start_dist_mm = SensorConfig.rad_config.start_dist_mm;
        reply_pkt.Payload.end_dist_mm = SensorConfig.rad_config.end_dist_mm;
        reply_pkt.Payload.max_profile = SensorConfig.rad_config.max_profile;
        reply_pkt.Payload.peak_sorting = SensorConfig.rad_config.peak_sorting;
        reply_pkt.Payload.threshold_method = SensorConfig.rad_config.threshold_method;
        reply_pkt.Payload.threshold_sensitivity = SensorConfig.rad_config.threshold_sensitivity;
        reply_pkt.Payload.fixed_amp_threshold = SensorConfig.rad_config.fixed_amp_threshold;
        reply_pkt.Payload.fixed_str_threshold = SensorConfig.rad_config.fixed_str_threshold;
        reply_pkt.Payload.signal_quality = SensorConfig.rad_config.signal_quality;
        reply_pkt.Payload.max_step_count = SensorConfig.rad_config.max_step_count;
        reply_pkt.Payload.reflector_shape = SensorConfig.rad_config.reflector_shape;
        reply_pkt.Payload.close_range_leakage = SensorConfig.rad_config.close_range_leakage;
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
    }
    // else put current config back in place
    else {
        TSLogs << "configure_radar bad.\n";
        configure_radar(SensorConfig.rad_config);
        PacketReplyFailure_t reply_pkt(ppkt->RefID);
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
    }

    // RADAR OFF
    DisableSensors();
}

void OnHHCommandGetRadarConfig(PacketCommandGetRadarConfig_t *ppkt) {
    TSLogs << "OnHHCommandGetRadarConfig\n";

    // send reply
    PacketReplyRadarConfig_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.start_dist_mm = SensorConfig.rad_config.start_dist_mm;
    reply_pkt.Payload.end_dist_mm = SensorConfig.rad_config.end_dist_mm;
    reply_pkt.Payload.max_profile = SensorConfig.rad_config.max_profile;
    reply_pkt.Payload.peak_sorting = SensorConfig.rad_config.peak_sorting;
    reply_pkt.Payload.threshold_method = SensorConfig.rad_config.threshold_method;
    reply_pkt.Payload.threshold_sensitivity = SensorConfig.rad_config.threshold_sensitivity;
    reply_pkt.Payload.fixed_amp_threshold = SensorConfig.rad_config.fixed_amp_threshold;
    reply_pkt.Payload.fixed_str_threshold = SensorConfig.rad_config.fixed_str_threshold;
    reply_pkt.Payload.signal_quality = SensorConfig.rad_config.signal_quality;
    reply_pkt.Payload.max_step_count = SensorConfig.rad_config.max_step_count;
    reply_pkt.Payload.reflector_shape = SensorConfig.rad_config.reflector_shape;
    reply_pkt.Payload.close_range_leakage = SensorConfig.rad_config.close_range_leakage;

    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

// prepare to receive a DFU image
void OnHHCommandBeginDFUUpload(PacketCommandBeginDFUUpload_t *ppkt) {
    TSLogs << "OnHHCommandBeginDFUUpload\n";
    TSLogs.Time() << "DFU begin CRC " << ppkt->Payload.dfu_crc
        << " size " << ppkt->Payload.dfu_size << "\n";
    
    if (!begin_bank1_write(ppkt->Payload.dfu_size, ppkt->Payload.dfu_crc)) {
        TSLogs.Time() << "bank1_begin_write failed!\n";
        PacketReplyDFUUpload_t reply_pkt(ppkt->RefID);
        reply_pkt.Payload.fsuccess = 0;  // failure
        reply_pkt.Payload.current_offset = 0;
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
        return;
    }

    // success
    PacketReplyDFUUpload_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.fsuccess = 1;  // success
    reply_pkt.Payload.current_offset = 0;
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

// receive the next chunk of the DFU image
void OnHHCommandDFUUpload(PacketCommandDFUUpload_t *ppkt) {
    TSLogs << "OnHHCommandDFUUpload\n";
    TSLogs.Time() << "DFU chunk " << ppkt->Payload.dfu_offset << " size " << ppkt->Payload.num_bytes << "\n";
    TSLogs << "bank1_cur_offset " << bank1_cur_offset() << "\n";

    // verify offset
    if (ppkt->Payload.dfu_offset != bank1_cur_offset()) {
        TSLogs.Time() << "bad dfu_offset in packet!\n";
        PacketReplyDFUUpload_t reply_pkt(ppkt->RefID);
        reply_pkt.Payload.fsuccess = 0;  // failure
        reply_pkt.Payload.current_offset = bank1_cur_offset();
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
        return;
    }
    // write dfu image bytes to bank1
    if (!bank1_write(ppkt->Payload.barray, ppkt->Payload.num_bytes)) {
        TSLogs.Time() << "bank1_write failed!\n";
        TSLogs << "bank1_cur_offset " << bank1_cur_offset() << "\n";
        PacketReplyDFUUpload_t reply_pkt(ppkt->RefID);
        reply_pkt.Payload.fsuccess = 0;  // failure
        reply_pkt.Payload.current_offset = bank1_cur_offset();
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
        return;
    }
    // success
    TSLogs << "bank1_cur_offset " << bank1_cur_offset() << "\n";
    PacketReplyDFUUpload_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.fsuccess = 1;  // success
    reply_pkt.Payload.current_offset = bank1_cur_offset();
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

// verify the DFU image checksum
void OnHHCommandVerifyDFUUpload(PacketCommandVerifyDFUUpload_t *ppkt) {
    TSLogs << "OnHHCommandVerifyDFUUpload\n";

    // checksum ok?
    if (!bank1_dfu_verify()) {
        TSLogs.Time() << "DFU image verification failed!\n";
        PacketReplyVerifyDFUUpload_t reply_pkt(ppkt->RefID);
        reply_pkt.Payload.fsuccess = 0;  // failure
        QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
        return;
    }
    // success
    bank1_dfu_activate();    
    TSLogs.Time() << "$$$$ DFU image verified and activated.\n";
    PacketReplyVerifyDFUUpload_t reply_pkt(ppkt->RefID);
    reply_pkt.Payload.fsuccess = 1;  // success
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

// Blink the LEDs in a highly visable way
void OnHHCommandBlinkLED(PacketCommandBlinkLED_t *ppkt) {
    TSLogs << "OnHHCommandBlinkLEDS\n";
    // do a HAPPY BLINK! we are in contact with base.
    do_happy_blink();
    // success
    PacketReplySuccess_t reply_pkt(ppkt->RefID);
    QueueLoRaCmdReply(reply_pkt, ppkt->Header.Src);
}

// @brief Attempt to sync now
void AttemptSyncToInterval( const HHDeviceAddress_t &src_id, uint8_t base_hops ){
    bool fsrc_insync = base_hops > 0;
    if ( !gHHSync.IsSynchronized() && fsrc_insync ){
        TSLogs.Time() << "@@ Temporarily synchronizing to tlm from " << src_id << "\n";

        // A Synchronized node just set its telemetry. That means we are somewhere within the
        // RX Window right now. Give time to let other nodes finish their telemetry, then wake and 
        // send ours. The RX window should still be open and other nodes will be able to relay us.
        //
        // compensate for wakeup and measure time
        int wakeup_measure_ms = SENSOR_WAKEMEASURE_TIMEMS;  // radar sensor
        int next_int_ms = HOLYHOP_TX_EXTRA_WAIT_SEC * 1000 - wakeup_measure_ms;
        next_int_ms = std::min(next_int_ms, 1);
        SetTelemetryWakeupTimerMS(next_int_ms); //todo: get from pkt
        // restart measurement timer
        gMeasureWakeupTimer.setPeriod(SensorConfig.measurement_interval_ms);
        //we used to just do this: SendLatestTelemetry();
    }
}

// @brief directly received another sensor's telemetry/awake pkt.
void OnHHTelemetry(const HHDeviceAddress_t &src_id, uint8_t base_hops ) {
    TSLogs << "OnHHTelemetry\n";
    AttemptSyncToInterval( src_id, base_hops );
}
