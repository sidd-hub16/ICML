/*
========================================================
 SMART IMU ROAD ACCIDENT DETECTION SYSTEM
 ESP32 WEB DASHBOARD + ANDROID SMS ALERT
========================================================

HARDWARE
--------------------------------------------------------
Push Button : GPIO 4
LED         : GPIO 2
Buzzer      : GPIO 5
LCD SDA     : GPIO 21
LCD SCL     : GPIO 22

NO MPU6050
NO GPS
NO GSM

Sensor values are live MPU6050 readings.

SMS FLOW
--------------------------------------------------------
ESP32
   |
   | WiFi HTTP POST
   v
Android Phone :5000
   |
   +----> SMS to Amma
   |
   +----> SMS to Appa

BUTTON OPERATION
--------------------------------------------------------
1st press:
    Accident detected
    10 second cancellation window

2nd press within 10 seconds:
    Accident cancelled

No second press:
    Accident confirmed
    Android phone receives alert
    SMS sent to Amma and Appa

After confirmation:
    Hold button for 2 seconds to reset
========================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// ======================================================
// WIFI
// ======================================================

const char* WIFI_SSID = "Shreeram/Siddharth";
const char* WIFI_PASSWORD = "11223344";


// ======================================================
// ANDROID PHONE
// ======================================================

// IMPORTANT:
// Change this if your Android app shows a different IP.

const char* PHONE_IP = "192.168.0.115";
const int PHONE_PORT = 5000;


// ======================================================
// PINS
// ======================================================

#define BUTTON_PIN 4
#define LED_PIN 2
#define BUZZER_PIN 5

#define SDA_PIN 21
#define SCL_PIN 22


// ======================================================
// LCD
// ======================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);


// ======================================================
// ESP32 WEB SERVER
// ======================================================

WebServer server(80);


// ======================================================
// SYSTEM STATES
// ======================================================

enum SystemState
{
  NORMAL,
  ACCIDENT_DETECTED,
  ACCIDENT_CONFIRMED
};

SystemState systemState = NORMAL;


// ======================================================
// BUTTON
// ======================================================

bool lastButtonState = HIGH;
bool currentButtonState = HIGH;

unsigned long buttonPressStart = 0;
unsigned long lastDebounceTime = 0;

const unsigned long debounceDelay = 50;
const unsigned long longPressTime = 2000;


// ======================================================
// ACCIDENT
// ======================================================

unsigned long accidentStartTime = 0;

const unsigned long cancellationTime = 10000;

bool smsSent = false;


// ======================================================
// REAL MPU6050 SENSOR
// ======================================================

#define MPU_SDA 25
#define MPU_SCL 26
#define MPU_DEFAULT_ADDR 0x68

TwoWire MPU_I2C = TwoWire(1);
byte mpuAddress = MPU_DEFAULT_ADDR;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 1.0;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

float acceleration = 1.00;       // Total acceleration in g
float gyro = 0.00;               // Gyroscope magnitude in deg/s
float motionIntensity = 0.00;    // Change in acceleration in g
float tiltAngle = 0.00;          // Tilt relative to calibrated normal

float impactG = 1.00;
float vibrationG = 0.00;
float rollAngle = 0.0;
float pitchAngle = 0.0;
float relativeTilt = 0.0;
float sepi = 20.0;

float previousAccelX = 0.0;
float previousAccelY = 0.0;
float previousAccelZ = 1.0;

float normalAx = 0.0;
float normalAy = 0.0;
float normalAz = 1.0;

float gyroBiasX = 0.0;
float gyroBiasY = 0.0;
float gyroBiasZ = 0.0;

bool mpuConnected = false;
bool carFlipped = false;
String carStatus = "Normal";
String detectionReason = "Monitoring";

// The sensor is ignored briefly after calibration so startup transients
// cannot be interpreted as a crash.
unsigned long sensorReadyTime = 0;
const unsigned long SENSOR_STARTUP_GRACE = 5000;

// Accident thresholds.
// The first two are intentionally based on the real reference code:
// strong impact and sudden acceleration change.
// Gyro is used as a secondary crash cue, while flip is handled separately.
const float ACCIDENT_IMPACT_THRESHOLD = 2.50;   // g
const float ACCIDENT_VIBRATION_THRESHOLD = 1.20; // g change
const float ACCIDENT_GYRO_THRESHOLD = 300.0;    // deg/s

// A vehicle is considered flipped only when its gravity direction has
// rotated far away from the calibrated normal direction.
const float FLIP_ANGLE_THRESHOLD = 120.0;        // degrees

// Extreme tilt is displayed but is NOT by itself a crash trigger.
// This avoids the false startup/normal-position trigger from the old code.

int sensorAccidentSamples = 0;
const int SENSOR_CONFIRM_SAMPLES = 3;

// MPU sampling
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 50; // 20 Hz

// ======================================================
// GPS SIMULATION
// ======================================================

float latitude = 12.981500;
float longitude = 80.218000;

String locationName = "Velachery, Chennai";

int satellites = 12;


// ======================================================
// EVENT TIME
// ======================================================

String eventTime = "--";


// ======================================================
// DASHBOARD HTML
// ======================================================

const char MAIN_PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width, initial-scale=1.0">

<title>Smart IMU Accident Detection System</title>

<style>

* {
    box-sizing: border-box;
}

body {
    margin: 0;
    font-family: Arial, Helvetica, sans-serif;
    background: #f4f6f9;
    color: #202832;
}

.app {
    display: flex;
    min-height: 100vh;
}


/* =====================================================
   SIDEBAR
===================================================== */

.sidebar {
    width: 250px;
    background: #0b1d30;
    color: white;
    position: fixed;
    left: 0;
    top: 0;
    bottom: 0;
    padding: 20px 14px;
}

.logo {
    display: flex;
    align-items: center;
    gap: 12px;
    padding: 5px 8px 25px;
}

.logoIcon {
    width: 50px;
    height: 50px;
    border-radius: 10px;
    background: #e63b45;
    display: flex;
    align-items: center;
    justify-content: center;
    color: white;
    font-size: 20px;
    font-weight: bold;
}

.logoText {
    font-size: 15px;
    font-weight: bold;
    line-height: 1.4;
}

.navItem {
    display: flex;
    align-items: center;
    gap: 14px;
    padding: 14px 16px;
    margin-bottom: 6px;
    border-radius: 10px;
    color: #d8e2ed;
}

.navItem.active {
    background: #2364b6;
    color: white;
}

.sosButton {
    position: absolute;
    left: 15px;
    right: 15px;
    bottom: 25px;
    width: calc(100% - 30px);
    padding: 16px;
    background: #ed4148;
    color: white;
    border: none;
    border-radius: 8px;
    font-size: 16px;
    font-weight: bold;
    cursor: pointer;
}


/* =====================================================
   MAIN
===================================================== */

.main {
    margin-left: 250px;
    width: calc(100% - 250px);
    padding: 28px;
}


/* =====================================================
   HEADER
===================================================== */

.topHeader {
    display: flex;
    justify-content: space-between;
    align-items: flex-start;
    margin-bottom: 25px;
}

.pageTitle h1 {
    margin: 0;
    font-size: 34px;
}

.pageTitle p {
    margin-top: 8px;
    color: #68727d;
}

.systemLive {
    text-align: right;
}

.liveDot {
    display: inline-block;
    width: 12px;
    height: 12px;
    background: #2bbd68;
    border-radius: 50%;
    margin-right: 7px;
}

.dateTime {
    margin-top: 12px;
    color: #66717c;
}


/* =====================================================
   STATUS CARDS
===================================================== */

.statusGrid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 20px;
    margin-bottom: 25px;
}

.statusCard {
    background: white;
    border-radius: 12px;
    padding: 22px;
    min-height: 165px;
    box-shadow: 0 3px 12px rgba(0,0,0,0.06);
}

.statusCard h3 {
    margin: 0;
    font-size: 14px;
    color: #3e4650;
}

.statusValue {
    margin-top: 17px;
    font-size: 24px;
    font-weight: bold;
}

.statusDescription {
    margin-top: 18px;
    font-size: 14px;
    color: #555;
}

.accidentText {
    color: #29a95d;
}

.activeText {
    color: #29a95d;
}

.gpsText {
    color: #3579c5;
}

.alertText {
    color: #777;
}


/* =====================================================
   MAIN GRID
===================================================== */

.mainGrid {
    display: grid;
    grid-template-columns: 1fr 1.5fr;
    gap: 20px;
    margin-bottom: 25px;
}

.card {
    background: white;
    border-radius: 12px;
    box-shadow: 0 3px 12px rgba(0,0,0,0.06);
    overflow: hidden;
}

.cardTitle {
    padding: 20px 24px 10px;
    font-size: 16px;
    font-weight: bold;
}


/* =====================================================
   SENSOR
===================================================== */

.sensorList {
    padding: 5px 24px 15px;
}

.sensor {
    display: flex;
    align-items: center;
    padding: 17px 0;
    border-bottom: 1px solid #e6e8eb;
}

.sensor:last-child {
    border-bottom: none;
}

.sensorIcon {
    width: 46px;
    height: 46px;
    border-radius: 50%;
    display: flex;
    align-items: center;
    justify-content: center;
    margin-right: 14px;
    font-size: 20px;
    font-weight: bold;
}

.sensorGreen {
    background: #dcf5e7;
    color: #39a96a;
}

.sensorBlue {
    background: #dcecff;
    color: #3778bf;
}

.sensorPurple {
    background: #eee1ff;
    color: #7650aa;
}

.sensorOrange {
    background: #fff0d8;
    color: #e99a32;
}

.sensorInfo {
    flex: 1;
}

.sensorName {
    font-weight: bold;
    font-size: 16px;
}

.sensorSub {
    color: #777;
    font-size: 13px;
    margin-top: 4px;
}

.sensorValue {
    text-align: right;
}

.sensorNumber {
    font-size: 22px;
    font-weight: bold;
}

.sensorState {
    font-size: 13px;
    color: #555;
    margin-top: 4px;
}


/* =====================================================
   MAP
===================================================== */

.mapCard {
    min-height: 430px;
}

.mapContainer {
    position: relative;
    height: 365px;
    overflow: hidden;
    background: #e8e5db;
}

.road {
    position: absolute;
    background: white;
    height: 9px;
}

.road1 {
    width: 110%;
    top: 46%;
    left: -5%;
    transform: rotate(-7deg);
}

.road2 {
    width: 100%;
    top: 64%;
    left: 0;
    transform: rotate(9deg);
}

.road3 {
    width: 80%;
    top: 20%;
    left: 10%;
    transform: rotate(27deg);
}

.mapLabel {
    position: absolute;
    color: #555;
    font-size: 13px;
}

.label1 {
    left: 10%;
    top: 18%;
}

.label2 {
    left: 68%;
    top: 17%;
}

.label3 {
    left: 42%;
    top: 67%;
}

.label4 {
    left: 70%;
    top: 74%;
}

.mapMarker {
    position: absolute;
    left: 50%;
    top: 50%;
    transform: translate(-50%, -50%);
}

.markerPin {
    width: 48px;
    height: 48px;
    background: #e33b43;
    border-radius: 50% 50% 50% 0;
    transform: rotate(-45deg);
}

.markerDot {
    position: absolute;
    width: 16px;
    height: 16px;
    background: white;
    border-radius: 50%;
    left: 16px;
    top: 16px;
}

.locationPopup {
    position: absolute;
    background: white;
    width: 215px;
    padding: 14px;
    border-radius: 9px;
    box-shadow: 0 3px 12px rgba(0,0,0,0.2);
    left: -83px;
    top: -145px;
    font-size: 13px;
}

.locationPopup strong {
    font-size: 15px;
}

.locationPopup p {
    margin: 7px 0 0;
    color: #555;
}

.mapBottom {
    position: absolute;
    bottom: 12px;
    left: 15px;
    right: 15px;
    background: rgba(255,255,255,0.94);
    padding: 10px 14px;
    border-radius: 7px;
    font-size: 13px;
}


/* =====================================================
   BOTTOM CARDS
===================================================== */

.bottomGrid {
    display: grid;
    grid-template-columns: 1.25fr 1fr 1fr;
    gap: 20px;
}

.bottomCard {
    background: white;
    border-radius: 12px;
    padding: 22px;
    min-height: 160px;
    box-shadow: 0 3px 12px rgba(0,0,0,0.06);
}

.recentAlert {
    background: #fff5f3;
}

.bottomTitle {
    font-size: 16px;
    font-weight: bold;
    margin-bottom: 20px;
}

.alertLine {
    display: flex;
    gap: 10px;
    margin-bottom: 14px;
    font-size: 14px;
}

.alertSymbol {
    color: #df3f47;
    font-weight: bold;
}

.viewButton {
    float: right;
    border: 1px solid #d55757;
    color: #bd4141;
    background: white;
    padding: 8px 14px;
    border-radius: 6px;
}

.contactRow {
    display: flex;
    align-items: center;
    gap: 12px;
    margin-bottom: 10px;
}

.contactCircle {
    width: 43px;
    height: 43px;
    border-radius: 50%;
    background: #dcecff;
    color: #2673c8;
    display: flex;
    align-items: center;
    justify-content: center;
    font-weight: bold;
}

.contactStatus {
    color: #2ca45a;
    margin-top: 15px;
    font-weight: bold;
}

.deviceRow {
    margin: 10px 0;
    font-size: 14px;
}


/* =====================================================
   MOBILE
===================================================== */

@media(max-width:1000px) {

    .statusGrid {
        grid-template-columns: repeat(2,1fr);
    }

    .mainGrid {
        grid-template-columns: 1fr;
    }

    .bottomGrid {
        grid-template-columns: 1fr;
    }
}

@media(max-width:700px) {

    .sidebar {
        width: 70px;
    }

    .logoText {
        display: none;
    }

    .main {
        margin-left: 70px;
        width: calc(100% - 70px);
        padding: 18px;
    }

    .statusGrid {
        grid-template-columns: 1fr;
    }

    .topHeader {
        flex-direction: column;
    }

    .systemLive {
        text-align: left;
        margin-top: 10px;
    }
}

</style>

</head>


<body>


<div class="app">


<!-- ===================================================
     SIDEBAR
=================================================== -->

<aside class="sidebar">

<div class="logo">

<div class="logoIcon">
IMU
</div>

<div class="logoText">
Smart IMU<br>
Accident Detection<br>
System
</div>

</div>


<div class="nav">

<div class="navItem active">
<span>HOME</span>
<span>Dashboard</span>
</div>

<div class="navItem">
<span>DATA</span>
<span>Live Data</span>
</div>

<div class="navItem">
<span>GPS</span>
<span>Location</span>
</div>

<div class="navItem">
<span>ALRT</span>
<span>Alerts</span>
</div>

<div class="navItem">
<span>HIST</span>
<span>History</span>
</div>

<div class="navItem">
<span>SET</span>
<span>Settings</span>
</div>

<div class="navItem">
<span>INFO</span>
<span>About</span>
</div>

</div>


<button
class="sosButton"
onclick="sosTest()">

SOS TEST

</button>

</aside>


<!-- ===================================================
     MAIN
=================================================== -->

<main class="main">


<div class="topHeader">

<div class="pageTitle">

<h1>
Dashboard
</h1>

<p>
Real-time monitoring of vehicle and accident status
</p>

</div>


<div id="carStatusBox"
style="background:white;border-radius:10px;padding:12px 18px;box-shadow:0 3px 12px rgba(0,0,0,0.06);min-width:150px;text-align:center;">

<div style="font-size:12px;font-weight:bold;color:#68727d;">
CAR STATUS
</div>

<div id="carStatus"
style="font-size:20px;font-weight:bold;color:#29a95d;margin-top:5px;">
NORMAL
</div>

<div id="carStatusReason"
style="font-size:11px;color:#777;margin-top:4px;">
Monitoring
</div>

</div>


<div class="systemLive">

<div>

<span class="liveDot"></span>

System Active

</div>

<div
class="dateTime"
id="dateTime">

Loading...

</div>

</div>

</div>


<!-- ===================================================
     STATUS
=================================================== -->

<div class="statusGrid">


<div class="statusCard">

<h3>
ACCIDENT STATUS
</h3>

<div
id="accidentStatus"
class="statusValue accidentText">

NORMAL

</div>

<div
id="accidentTime"
class="statusDescription">

System monitoring

</div>

</div>


<div class="statusCard">

<h3>
SYSTEM STATUS
</h3>

<div class="statusValue activeText">

ACTIVE

</div>

<div class="statusDescription">

All systems operational

</div>

</div>


<div class="statusCard">

<h3>
GPS STATUS
</h3>

<div class="statusValue gpsText">

CONNECTED

</div>

<div class="statusDescription">

Satellites:
<span id="satellites">
12
</span>

</div>

</div>


<div class="statusCard">

<h3>
ALERT STATUS
</h3>

<div
id="alertStatus"
class="statusValue alertText">

NOT SENT

</div>

<div
id="alertDescription"
class="statusDescription">

SMS Alert

</div>

</div>

</div>


<!-- ===================================================
     SENSOR + LOCATION
=================================================== -->

<div class="mainGrid">


<div class="card">

<div class="cardTitle">
SENSOR READINGS
</div>


<div class="sensorList">


<div class="sensor">

<div class="sensorIcon sensorGreen">
A
</div>

<div class="sensorInfo">

<div class="sensorName">
Acceleration (g)
</div>

<div class="sensorSub">
Live MPU6050 value
</div>

</div>

<div class="sensorValue">

<div
id="acceleration"
class="sensorNumber">

1.00

</div>

<div
id="accelerationState"
class="sensorState">

Normal

</div>

</div>

</div>


<div class="sensor">

<div class="sensorIcon sensorBlue">
G
</div>

<div class="sensorInfo">

<div class="sensorName">
Gyroscope (deg/s)
</div>

<div class="sensorSub">
Live MPU6050 value
</div>

</div>

<div class="sensorValue">

<div
id="gyro"
class="sensorNumber">

10.00

</div>

<div
id="gyroState"
class="sensorState">

Normal Rotation

</div>

</div>

</div>


<div class="sensor">

<div class="sensorIcon sensorPurple">
M
</div>

<div class="sensorInfo">

<div class="sensorName">
Motion Intensity
</div>

<div class="sensorSub">
Live MPU6050 value
</div>

</div>

<div class="sensorValue">

<div
id="motion"
class="sensorNumber">

1.20

</div>

<div
id="motionState"
class="sensorState">

Normal

</div>

</div>

</div>


<div class="sensor">

<div class="sensorIcon sensorOrange">
T
</div>

<div class="sensorInfo">

<div class="sensorName">
Vehicle Tilt (deg)
</div>

<div class="sensorSub">
Live MPU6050 value
</div>

</div>

<div class="sensorValue">

<div
id="tilt"
class="sensorNumber">

2.50

</div>

<div
id="tiltState"
class="sensorState">

Stable

</div>

</div>

</div>


</div>

<div style="margin:0 24px 18px;padding:14px;background:#f7f9fc;border:1px solid #e6e8eb;border-radius:9px;">

<div style="font-size:14px;font-weight:bold;margin-bottom:10px;">
LIVE MPU6050 DETAILS
</div>

<div style="display:grid;grid-template-columns:repeat(2,1fr);gap:7px 18px;font-size:12px;color:#555;">

<div>Accel X: <strong id="accelX">0.00</strong> g</div>
<div>Accel Y: <strong id="accelY">0.00</strong> g</div>
<div>Accel Z: <strong id="accelZ">1.00</strong> g</div>

<div>Gyro X: <strong id="gyroX">0.00</strong> °/s</div>
<div>Gyro Y: <strong id="gyroY">0.00</strong> °/s</div>
<div>Gyro Z: <strong id="gyroZ">0.00</strong> °/s</div>

<div>Impact: <strong id="impact">1.00</strong> g</div>
<div>Vibration: <strong id="vibration">0.00</strong> g</div>

<div>Roll: <strong id="roll">0.00</strong> °</div>
<div>Pitch: <strong id="pitch">0.00</strong> °</div>

<div>Relative Tilt: <strong id="relativeTilt">0.00</strong> °</div>
<div>SEPI: <strong id="sepi">0.0</strong></div>

</div>
</div>


</div>


<!-- ===================================================
     LOCATION
=================================================== -->

<div class="card mapCard">

<div class="cardTitle">
LOCATION
</div>


<div class="mapContainer">


<div class="road road1"></div>
<div class="road road2"></div>
<div class="road road3"></div>


<div class="mapLabel label1">
Velachery
</div>

<div class="mapLabel label2">
Chennai
</div>

<div class="mapLabel label3">
Guindy
</div>

<div class="mapLabel label4">
Taramani
</div>


<div class="mapMarker">


<div class="locationPopup">

<strong>
Accident Location
</strong>

<p>

<span id="mapLat">
12.981500
</span>

N,

<span id="mapLon">
80.218000
</span>

E

</p>


<p id="mapTime">
Monitoring
</p>

</div>


<div class="markerPin"></div>

<div class="markerDot"></div>


</div>


<div class="mapBottom">

Location:

<span id="locationName">
Velachery, Chennai
</span>

&nbsp; | &nbsp;

GPS Connected

</div>


</div>

</div>

</div>


<!-- ===================================================
     BOTTOM
=================================================== -->

<div class="bottomGrid">


<div class="bottomCard recentAlert">

<div class="bottomTitle">
RECENT ALERT
</div>


<div class="alertLine">

<span class="alertSymbol">
ALERT
</span>

<span id="recentAlertText">
No accident detected
</span>

</div>


<div class="alertLine">

<strong>
Location:
</strong>

<span>

<span id="recentLat">
12.981500
</span>

N,

<span id="recentLon">
80.218000
</span>

E

</span>

</div>


<button
class="viewButton"
onclick="showDetails()">

VIEW DETAILS

</button>


</div>


<!-- CONTACTS -->

<div class="bottomCard">

<div class="bottomTitle">
EMERGENCY CONTACT
</div>


<div class="contactRow">

<div class="contactCircle">
A
</div>

<div>

<strong>
Amma
</strong>

<br>

<span id="ammaStatus">
Waiting
</span>

</div>

</div>


<div class="contactRow">

<div class="contactCircle">
A
</div>

<div>

<strong>
Appa
</strong>

<br>

<span id="appaStatus">
Waiting
</span>

</div>

</div>


<div
id="contactStatus"
class="contactStatus">

Alert not sent

</div>

</div>


<!-- DEVICE -->

<div class="bottomCard">

<div class="bottomTitle">
DEVICE INFO
</div>


<div class="deviceRow">

<strong>
Device ID:
</strong>

IMU-001

</div>


<div class="deviceRow">

<strong>
Firmware:
</strong>

v1.0.0

</div>


<div class="deviceRow">

<strong>
Controller:
</strong>

ESP32

</div>


<div class="deviceRow">

<strong>
Connection:
</strong>

WiFi

</div>

</div>


</div>


</main>

</div>


<script>


// =====================================================
// DATE TIME
// =====================================================

function updateDateTime()
{

    var now = new Date();

    var date =
        now.toLocaleDateString(
            'en-GB',
            {
                day: '2-digit',
                month: 'short',
                year: 'numeric'
            }
        );

    var time =
        now.toLocaleTimeString();

    document.getElementById(
        'dateTime'
    ).innerHTML =
        date + ' | ' + time;
}

setInterval(
    updateDateTime,
    1000
);

updateDateTime();


// =====================================================
// UPDATE DASHBOARD
// =====================================================

function updateDashboard()
{

    fetch('/api/status')

    .then(
        function(response)
        {
            return response.json();
        }
    )

    .then(
        function(data)
        {

            // ACCIDENT STATUS

            var accidentStatus =
                document.getElementById(
                    'accidentStatus'
                );


            if(data.confirmed)
            {

                accidentStatus.innerHTML =
                    'ACCIDENT<br>CONFIRMED';

                accidentStatus.style.color =
                    '#dc3d42';

                document.getElementById(
                    'accidentTime'
                ).innerHTML =
                    'Confirmed at ' +
                    data.eventTime;

            }

            else if(data.accident)
            {

                accidentStatus.innerHTML =
                    'ACCIDENT<br>DETECTED';

                accidentStatus.style.color =
                    '#dc3d42';

                document.getElementById(
                    'accidentTime'
                ).innerHTML =
                    'Cancellation window active';

            }

            else
            {

                accidentStatus.innerHTML =
                    'NORMAL';

                accidentStatus.style.color =
                    '#29a95d';

                document.getElementById(
                    'accidentTime'
                ).innerHTML =
                    'System monitoring';
            }


            // SENSOR VALUES

            document.getElementById(
                'acceleration'
            ).innerHTML =
                Number(
                    data.acceleration
                ).toFixed(2);


            document.getElementById(
                'gyro'
            ).innerHTML =
                Number(
                    data.gyro
                ).toFixed(2);


            document.getElementById(
                'motion'
            ).innerHTML =
                Number(
                    data.motion
                ).toFixed(2);


            document.getElementById(
                'tilt'
            ).innerHTML =
                Number(
                    data.tilt
                ).toFixed(2);


            // DETAILED MPU6050 VALUES

            document.getElementById('accelX').innerHTML =
                Number(data.accelX).toFixed(2);

            document.getElementById('accelY').innerHTML =
                Number(data.accelY).toFixed(2);

            document.getElementById('accelZ').innerHTML =
                Number(data.accelZ).toFixed(2);

            document.getElementById('gyroX').innerHTML =
                Number(data.gyroX).toFixed(1);

            document.getElementById('gyroY').innerHTML =
                Number(data.gyroY).toFixed(1);

            document.getElementById('gyroZ').innerHTML =
                Number(data.gyroZ).toFixed(1);

            document.getElementById('impact').innerHTML =
                Number(data.impact).toFixed(2);

            document.getElementById('vibration').innerHTML =
                Number(data.vibration).toFixed(2);

            document.getElementById('roll').innerHTML =
                Number(data.roll).toFixed(1);

            document.getElementById('pitch').innerHTML =
                Number(data.pitch).toFixed(1);

            document.getElementById('relativeTilt').innerHTML =
                Number(data.tilt).toFixed(1);

            document.getElementById('sepi').innerHTML =
                Number(data.sepi).toFixed(1);


            // CAR STATUS BOX

            var carStatusBox =
                document.getElementById('carStatus');

            carStatusBox.innerHTML =
                data.carStatus.toUpperCase();

            if(data.carStatus === 'Flipped')
            {
                carStatusBox.style.color = '#dc3d42';
            }
            else if(data.carStatus === 'Crash')
            {
                carStatusBox.style.color = '#dc3d42';
            }
            else
            {
                carStatusBox.style.color = '#29a95d';
            }

            document.getElementById('carStatusReason').innerHTML =
                data.reason;


            // SENSOR STATES

            document.getElementById(
                'accelerationState'
            ).innerHTML =
                Number(data.impact) >= 2.50
                ? 'High Impact'
                : 'Normal';

            document.getElementById(
                'gyroState'
            ).innerHTML =
                Number(data.gyro) >= 300
                ? 'High Rotation'
                : 'Normal Rotation';

            document.getElementById(
                'motionState'
            ).innerHTML =
                Number(data.vibration) >= 1.20
                ? 'Sudden Motion'
                : 'Normal';

            document.getElementById(
                'tiltState'
            ).innerHTML =
                data.flipped
                ? 'Flipped'
                : (Number(data.tilt) >= 45
                    ? 'Tilted'
                    : 'Stable');


            // GPS

            document.getElementById(
                'mapLat'
            ).innerHTML =
                Number(
                    data.latitude
                ).toFixed(6);


            document.getElementById(
                'mapLon'
            ).innerHTML =
                Number(
                    data.longitude
                ).toFixed(6);


            document.getElementById(
                'recentLat'
            ).innerHTML =
                Number(
                    data.latitude
                ).toFixed(6);


            document.getElementById(
                'recentLon'
            ).innerHTML =
                Number(
                    data.longitude
                ).toFixed(6);


            document.getElementById(
                'locationName'
            ).innerHTML =
                data.location;


            // ALERT STATUS

            var alertStatus =
                document.getElementById(
                    'alertStatus'
                );


            if(data.alertSent)
            {

                alertStatus.innerHTML =
                    'SENT';

                alertStatus.style.color =
                    '#e8921d';


                document.getElementById(
                    'alertDescription'
                ).innerHTML =
                    'SMS sent to Amma + Appa';


                document.getElementById(
                    'ammaStatus'
                ).innerHTML =
                    'SMS Sent';


                document.getElementById(
                    'appaStatus'
                ).innerHTML =
                    'SMS Sent';


                document.getElementById(
                    'contactStatus'
                ).innerHTML =
                    'Alert sent successfully';

            }

            else if(data.accident)
            {

                alertStatus.innerHTML =
                    'PENDING';

                alertStatus.style.color =
                    '#e8921d';


                document.getElementById(
                    'alertDescription'
                ).innerHTML =
                    'Waiting for confirmation';


                document.getElementById(
                    'ammaStatus'
                ).innerHTML =
                    'Pending';


                document.getElementById(
                    'appaStatus'
                ).innerHTML =
                    'Pending';


                document.getElementById(
                    'contactStatus'
                ).innerHTML =
                    'Alert pending';

            }

            else
            {

                alertStatus.innerHTML =
                    'NOT SENT';

                alertStatus.style.color =
                    '#777';


                document.getElementById(
                    'alertDescription'
                ).innerHTML =
                    'SMS Alert';


                document.getElementById(
                    'ammaStatus'
                ).innerHTML =
                    'Waiting';


                document.getElementById(
                    'appaStatus'
                ).innerHTML =
                    'Waiting';


                document.getElementById(
                    'contactStatus'
                ).innerHTML =
                    'Alert not sent';
            }


            // RECENT ALERT

            if(data.confirmed)
            {

                document.getElementById(
                    'recentAlertText'
                ).innerHTML =
                    'Accident confirmed at ' +
                    data.eventTime;


                document.getElementById(
                    'mapTime'
                ).innerHTML =
                    data.eventTime;

            }

            else if(data.accident)
            {

                document.getElementById(
                    'recentAlertText'
                ).innerHTML =
                    'Accident detected - pending';


                document.getElementById(
                    'mapTime'
                ).innerHTML =
                    'Accident detected';

            }

            else
            {

                document.getElementById(
                    'recentAlertText'
                ).innerHTML =
                    'No accident detected';


                document.getElementById(
                    'mapTime'
                ).innerHTML =
                    'Monitoring';
            }

        }
    )

    .catch(
        function(error)
        {
            console.log(
                'Dashboard error:',
                error
            );
        }
    );
}


// =====================================================
// SOS TEST
// =====================================================

function sosTest()
{

    fetch(
        '/api/accident',
        {
            method: 'POST'
        }
    )

    .then(
        function(response)
        {
            return response.text();
        }
    )

    .then(
        function(data)
        {
            console.log(data);
        }
    );
}


// =====================================================
// VIEW DETAILS
// =====================================================

function showDetails()
{

    alert(
        'Smart IMU Accident Detection System\n\n' +
        'Location: Velachery, Chennai\n' +
        'GPS: 12.981500, 80.218000\n\n' +
        'Live MPU6050 values are shown on the dashboard.'
    );
}


// =====================================================
// AUTO UPDATE
// =====================================================

setInterval(
    updateDashboard,
    500
);

updateDashboard();

</script>


</body>

</html>

)rawliteral";


// ======================================================
// SETUP
// ======================================================

void setup()
{

  Serial.begin(115200);

  delay(1000);


  Serial.println();
  Serial.println("========================================");
  Serial.println(" SMART IMU ACCIDENT DETECTION SYSTEM");
  Serial.println("========================================");


  // ====================================================
  // PINS
  // ====================================================

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );

  pinMode(
    LED_PIN,
    OUTPUT
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );


  digitalWrite(
    LED_PIN,
    LOW
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  // ====================================================
  // LCD
  // ====================================================

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  lcd.init();

  lcd.backlight();

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Smart IMU");

  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  delay(2000);


  // ====================================================
  // MPU6050
  // ====================================================

  if (!initMPU6050())
  {
    Serial.println();
    Serial.println("ERROR: MPU6050 NOT FOUND!");
    Serial.println("Check:");
    Serial.println("VCC -> 3.3V");
    Serial.println("GND -> GND");
    Serial.println("SDA -> GPIO 25");
    Serial.println("SCL -> GPIO 26");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 ERROR");
    lcd.setCursor(0, 1);
    lcd.print("Check Wiring");

    while (true)
    {
      delay(1000);
    }
  }

  calibrateMPU6050();

  sensorReadyTime = millis() + SENSOR_STARTUP_GRACE;
  sensorAccidentSamples = 0;
  carStatus = "Normal";
  detectionReason = "Monitoring";


  // ====================================================
  // WIFI
  // ====================================================

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");


  Serial.println();
  Serial.println("Connecting to WiFi...");


  WiFi.mode(
    WIFI_STA
  );


  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );


  int attempts = 0;


  while(
    WiFi.status() != WL_CONNECTED &&
    attempts < 30
  )
  {

    delay(500);

    Serial.print(".");

    attempts++;
  }


  Serial.println();


  // ====================================================
  // WIFI CONNECTED
  // ====================================================

  if(
    WiFi.status() == WL_CONNECTED
  )
  {

    Serial.println(
      "WiFi Connected!"
    );


    Serial.print(
      "ESP32 IP Address: "
    );


    Serial.println(
      WiFi.localIP()
    );


    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");

    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());

    delay(2500);

  }

  else
  {

    Serial.println(
      "WiFi Connection Failed"
    );


    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed");

    lcd.setCursor(0, 1);
    lcd.print("Check Network");

    delay(2000);
  }


  // ====================================================
  // ESP32 DASHBOARD SERVER
  // ====================================================

  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );


  server.on(
    "/api/status",
    HTTP_GET,
    handleStatus
  );


  server.on(
    "/api/accident",
    HTTP_POST,
    handleAccident
  );


  server.begin();


  Serial.println();
  Serial.println(
    "ESP32 Web Server Started!"
  );


  Serial.print(
    "Dashboard: http://"
  );

  Serial.println(
    WiFi.localIP()
  );


  // ====================================================
  // ANDROID CONNECTION TEST
  // ====================================================

  Serial.println();
  Serial.println("----------------------------------------");
  Serial.println("TESTING ANDROID CONNECTION");
  Serial.println("----------------------------------------");


  Serial.print(
    "Android IP: "
  );

  Serial.println(
    PHONE_IP
  );


  Serial.print(
    "Android Port: "
  );

  Serial.println(
    PHONE_PORT
  );


  if(
    WiFi.status() == WL_CONNECTED
  )
  {

    WiFiClient testClient;


    Serial.println(
      "Connecting to Android..."
    );


    if(
      testClient.connect(
        PHONE_IP,
        PHONE_PORT
      )
    )
    {

      Serial.println(
        "SUCCESS: Android port 5000 is reachable!"
      );


      testClient.stop();

    }

    else
    {

      Serial.println(
        "FAILED: Android port 5000 is NOT reachable!"
      );


      Serial.println(
        "Make sure Android app is OPEN."
      );

      Serial.println(
        "Make sure phone IP is correct."
      );

    }
  }


  Serial.println("----------------------------------------");


  // ====================================================
  // SYSTEM READY
  // ====================================================

  Serial.println(
    "SYSTEM READY"
  );


  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SYSTEM READY");

  lcd.setCursor(0, 1);
  lcd.print("Press Button");


  randomSeed(
    micros()
  );
}


// ======================================================
// LOOP
// ======================================================

void loop()
{

  server.handleClient();

  readButton();

  updateSystem();

  updateSensorValues();

  delay(10);
}


// ======================================================
// BUTTON
// ======================================================

void readButton()
{

  bool reading =
    digitalRead(
      BUTTON_PIN
    );


  if(
    reading != lastButtonState
  )
  {

    lastDebounceTime =
      millis();
  }


  if(
    millis() - lastDebounceTime >
    debounceDelay
  )
  {

    if(
      reading != currentButtonState
    )
    {

      currentButtonState =
        reading;


      // ==================================================
      // BUTTON PRESSED
      // ==================================================

      if(
        currentButtonState == LOW
      )
      {

        buttonPressStart =
          millis();


        Serial.println(
          "BUTTON PRESSED"
        );
      }


      // ==================================================
      // BUTTON RELEASED
      // ==================================================

      else
      {

        unsigned long duration =
          millis() -
          buttonPressStart;


        Serial.print(
          "Button duration: "
        );

        Serial.print(
          duration
        );

        Serial.println(
          " ms"
        );


        // ==================================================
        // NORMAL
        // ==================================================

        if(
          systemState == NORMAL
        )
        {

          if(
            duration >= 100
          )
          {

            triggerAccident();
          }
        }


        // ==================================================
        // ACCIDENT DETECTED
        // ==================================================

        else if(
          systemState ==
          ACCIDENT_DETECTED
        )
        {

          if(
            duration >= 100
          )
          {

            cancelAccident();
          }
        }


        // ==================================================
        // ACCIDENT CONFIRMED
        // ==================================================

        else if(
          systemState ==
          ACCIDENT_CONFIRMED
        )
        {

          if(
            duration >=
            longPressTime
          )
          {

            resetSystem();
          }
        }
      }
    }
  }


  lastButtonState =
    reading;
}


// ======================================================
// TRIGGER ACCIDENT
// ======================================================

void triggerAccident()
{

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "ACCIDENT DETECTED"
  );

  Serial.println(
    "10 SECOND CANCELLATION WINDOW"
  );

  Serial.println(
    "================================"
  );


  systemState =
    ACCIDENT_DETECTED;


  accidentStartTime =
    millis();


  smsSent =
    false;

  sensorAccidentSamples = 0;
  carStatus = "Crash";
  if (detectionReason == "Monitoring")
    detectionReason = "Manual button trigger";


  digitalWrite(
    LED_PIN,
    HIGH
  );


  digitalWrite(
    BUZZER_PIN,
    HIGH
  );


  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("ACCIDENT!");

  lcd.setCursor(0, 1);
  lcd.print("Cancel: 10 sec");
}


// ======================================================
// UPDATE SYSTEM
// ======================================================

void updateSystem()
{

  if(
    systemState ==
    ACCIDENT_DETECTED
  )
  {

    unsigned long elapsed =
      millis() -
      accidentStartTime;


    if(
      elapsed <
      cancellationTime
    )
    {

      unsigned long remaining =
        (
          cancellationTime -
          elapsed +
          999
        ) / 1000;


      static int lastRemaining =
        -1;


      if(
        (int)remaining !=
        lastRemaining
      )
      {

        lastRemaining =
          remaining;


        Serial.print(
          "Cancellation countdown: "
        );

        Serial.print(
          remaining
        );

        Serial.println(
          " seconds"
        );


        lcd.setCursor(
          0,
          1
        );

        lcd.print(
          "Cancel: "
        );

        lcd.print(
          remaining
        );

        lcd.print(
          " sec "
        );
      }

    }

    else
    {

      confirmAccident();
    }
  }


  if(
    systemState ==
    ACCIDENT_CONFIRMED
  )
  {

    digitalWrite(
      LED_PIN,
      HIGH
    );
  }
}


// ======================================================
// CANCEL ACCIDENT
// ======================================================

void cancelAccident()
{

  Serial.println();
  Serial.println(
    "ACCIDENT CANCELLED"
  );


  systemState =
    NORMAL;


  smsSent =
    false;


  // Do not reset sensor readings to fake values.
  // updateSensorValues() continues reading the real MPU6050.

  sensorAccidentSamples = 0;


  digitalWrite(
    LED_PIN,
    LOW
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("ACCIDENT");

  lcd.setCursor(0, 1);
  lcd.print("CANCELLED");


  delay(1500);


  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SYSTEM READY");

  lcd.setCursor(0, 1);
  lcd.print("Press Button");
}


// ======================================================
// CONFIRM ACCIDENT
// ======================================================

void confirmAccident()
{

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "ACCIDENT CONFIRMED"
  );

  Serial.println(
    "CANCELLATION TIME EXPIRED"
  );

  Serial.println(
    "================================"
  );


  systemState =
    ACCIDENT_CONFIRMED;


  digitalWrite(
    LED_PIN,
    HIGH
  );


  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  // ====================================================
  // SEND REAL ALERT TO ANDROID
  // ====================================================

  bool success =
    sendAccidentToPhone();


  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("ACCIDENT CONF.");


  if(success)
  {

    lcd.setCursor(0, 1);
    lcd.print("SMS SENT");

  }

  else
  {

    lcd.setCursor(0, 1);
    lcd.print("PHONE ERROR");
  }
}


// ======================================================
// SEND ACCIDENT TO ANDROID PHONE
// ======================================================

bool sendAccidentToPhone()
{

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "SENDING ACCIDENT TO ANDROID"
  );

  Serial.println(
    "========================================"
  );


  // ----------------------------------------------------
  // CHECK WIFI
  // ----------------------------------------------------

  if(
    WiFi.status() != WL_CONNECTED
  )
  {

    Serial.println(
      "ERROR: WiFi is not connected!"
    );

    smsSent =
      false;

    return false;
  }


  // ----------------------------------------------------
  // SHOW CONNECTION INFORMATION
  // ----------------------------------------------------

  Serial.print(
    "ESP32 IP: "
  );

  Serial.println(
    WiFi.localIP()
  );


  Serial.print(
    "Android IP: "
  );

  Serial.println(
    PHONE_IP
  );


  Serial.print(
    "Android Port: "
  );

  Serial.println(
    PHONE_PORT
  );


  // ----------------------------------------------------
  // CREATE HTTP CLIENT
  // ----------------------------------------------------

  WiFiClient client;

  HTTPClient http;


  String url =
    "http://" +
    String(PHONE_IP) +
    ":" +
    String(PHONE_PORT) +
    "/accident";


  Serial.print(
    "URL: "
  );

  Serial.println(
    url
  );


  // ----------------------------------------------------
  // CONNECT
  // ----------------------------------------------------

  if(
    !http.begin(
      client,
      url
    )
  )
  {

    Serial.println(
      "ERROR: HTTP client could not start!"
    );

    smsSent =
      false;

    return false;
  }


  http.setTimeout(
    5000
  );


  http.addHeader(
    "Content-Type",
    "text/plain"
  );


  // ----------------------------------------------------
  // CREATE MESSAGE
  // ----------------------------------------------------

  String message =

    "ACCIDENT DETECTED!\n"
    "Location: " +
    locationName +
    "\nGPS: " +
    String(latitude, 6) +
    ", " +
    String(longitude, 6) +
    "\nTime: " +
    eventTime +
    "\nAcceleration: " +
    String(acceleration, 2) +
    " g\nGyro: " +
    String(gyro, 2) +
    " deg/s\nMotion: " +
    String(motionIntensity, 2) +
    "\nTilt: " +
    String(tiltAngle, 2);


  Serial.println();
  Serial.println(
    "Message:"
  );

  Serial.println(
    message
  );


  // ----------------------------------------------------
  // SEND POST
  // ----------------------------------------------------

  Serial.println();
  Serial.println(
    "Connecting to Android server..."
  );


  int httpCode =
    http.POST(
      message
    );


  // ----------------------------------------------------
  // CHECK RESULT
  // ----------------------------------------------------

  Serial.print(
    "HTTP Response: "
  );

  Serial.println(
    httpCode
  );


  if(
    httpCode > 0
  )
  {

    String response =
      http.getString();


    Serial.print(
      "Phone response: "
    );

    Serial.println(
      response
    );


    if(
      httpCode >= 200 &&
      httpCode < 300
    )
    {

      Serial.println();
      Serial.println(
        "================================"
      );

      Serial.println(
        "ACCIDENT SENT TO PHONE!"
      );

      Serial.println(
        "Android should now send SMS"
      );

      Serial.println(
        "to Amma and Appa."
      );

      Serial.println(
        "================================"
      );


      smsSent =
        true;


      http.end();


      return true;
    }

    else
    {

      Serial.println(
        "ERROR: Android returned HTTP error."
      );

      smsSent =
        false;
    }

  }

  else
  {

    Serial.print(
      "ERROR: Could not connect to phone. Code: "
    );

    Serial.println(
      httpCode
    );


    Serial.println();
    Serial.println(
      "CHECK:"
    );

    Serial.println(
      "1. Android app is OPEN"
    );

    Serial.println(
      "2. Phone IP is 192.168.0.115"
    );

    Serial.println(
      "3. Phone and ESP32 use same WiFi"
    );

    Serial.println(
      "4. Android server says Waiting for ESP32"
    );

    Serial.println(
      "5. Server port is 5000"
    );


    smsSent =
      false;
  }


  http.end();


  return false;
}


// ======================================================
// RESET SYSTEM
// ======================================================

void resetSystem()
{

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "SYSTEM RESET"
  );

  Serial.println(
    "================================"
  );


  systemState =
    NORMAL;


  smsSent =
    false;


  // Do not reset sensor readings to fake values.
  // updateSensorValues() continues reading the real MPU6050.

  sensorAccidentSamples = 0;


  digitalWrite(
    LED_PIN,
    LOW
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SYSTEM RESET");

  lcd.setCursor(0, 1);
  lcd.print("READY");


  delay(1500);


  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("SYSTEM READY");

  lcd.setCursor(0, 1);
  lcd.print("Press Button");
}


// ======================================================
// REAL MPU6050 SENSOR VALUES + ACCIDENT DETECTION
// ======================================================

void updateSensorValues()
{
  if (!mpuConnected)
    return;

  if (millis() - lastSensorRead < sensorInterval)
    return;

  lastSensorRead = millis();

  float ax, ay, az;
  float gx, gy, gz;

  if (!readMPU6050(ax, ay, az, gx, gy, gz))
  {
    Serial.println("MPU6050 read error!");
    return;
  }

  accelX = ax;
  accelY = ay;
  accelZ = az;

  gyroX = gx - gyroBiasX;
  gyroY = gy - gyroBiasY;
  gyroZ = gz - gyroBiasZ;

  // ----------------------------------------------------
  // TOTAL ACCELERATION / IMPACT
  // ----------------------------------------------------

  impactG = sqrt(
    ax * ax +
    ay * ay +
    az * az
  );

  acceleration = impactG;

  // ----------------------------------------------------
  // SUDDEN ACCELERATION CHANGE / VIBRATION
  // ----------------------------------------------------

  vibrationG = sqrt(
    (ax - previousAccelX) * (ax - previousAccelX) +
    (ay - previousAccelY) * (ay - previousAccelY) +
    (az - previousAccelZ) * (az - previousAccelZ)
  );

  motionIntensity = vibrationG;

  previousAccelX = ax;
  previousAccelY = ay;
  previousAccelZ = az;

  // ----------------------------------------------------
  // GYROSCOPE MAGNITUDE
  // ----------------------------------------------------

  gyro = sqrt(
    gyroX * gyroX +
    gyroY * gyroY +
    gyroZ * gyroZ
  );

  // ----------------------------------------------------
  // RAW ROLL / PITCH
  // ----------------------------------------------------

  rollAngle = atan2(
    ay,
    az
  ) * 180.0 / PI;

  pitchAngle = atan2(
    -ax,
    sqrt(
      ay * ay +
      az * az
    )
  ) * 180.0 / PI;

  // ----------------------------------------------------
  // RELATIVE TILT FROM THE CORRECTLY POSITIONED
  // CALIBRATED NORMAL ORIENTATION
  // ----------------------------------------------------

  float normalMagnitude = sqrt(
    normalAx * normalAx +
    normalAy * normalAy +
    normalAz * normalAz
  );

  float currentMagnitude = sqrt(
    ax * ax +
    ay * ay +
    az * az
  );

  float dotProduct = 1.0;

  if (normalMagnitude > 0.01 && currentMagnitude > 0.01)
  {
    dotProduct =
      (normalAx * ax +
       normalAy * ay +
       normalAz * az) /
      (normalMagnitude * currentMagnitude);

    dotProduct = constrain(dotProduct, -1.0, 1.0);
  }

  // Angle away from the calibrated normal position.
  relativeTilt = acos(dotProduct) * 180.0 / PI;
  tiltAngle = relativeTilt;

  // ----------------------------------------------------
  // FLIP
  // ----------------------------------------------------

  carFlipped =
    (relativeTilt >= FLIP_ANGLE_THRESHOLD);

  // ----------------------------------------------------
  // SEPI
  // A simple normalized impact severity indicator for
  // the dashboard; 1 g = baseline, 5 g = 100.
  // ----------------------------------------------------

  sepi = ((impactG - 1.0) / 4.0) * 100.0;

  if (sepi < 0.0)
    sepi = 0.0;

  if (sepi > 100.0)
    sepi = 100.0;

  // ----------------------------------------------------
  // CAR STATUS
  // ----------------------------------------------------

  if (carFlipped)
  {
    carStatus = "Flipped";
  }
  else if (systemState != NORMAL)
  {
    carStatus = "Crash";
  }
  else
  {
    carStatus = "Normal";
  }

  // ----------------------------------------------------
  // AUTOMATIC ACCIDENT DETECTION
  //
  // Startup grace period prevents the sensor settling
  // immediately after upload from causing a false crash.
  //
  // Flip requires its own clear condition.
  // Impact/vibration/gyro must persist for 3 samples.
  // ----------------------------------------------------

  bool impactEvent =
    (impactG >= ACCIDENT_IMPACT_THRESHOLD);

  bool vibrationEvent =
    (vibrationG >= ACCIDENT_VIBRATION_THRESHOLD) &&
    (impactG >= 1.50);

  bool rotationEvent =
    (gyro >= ACCIDENT_GYRO_THRESHOLD);

  bool sensorCrashEvent =
    impactEvent ||
    vibrationEvent ||
    rotationEvent;

  if (millis() >= sensorReadyTime &&
      systemState == NORMAL)
  {
    if (carFlipped)
    {
      sensorAccidentSamples++;

      detectionReason = "Vehicle flipped";

      if (sensorAccidentSamples >= SENSOR_CONFIRM_SAMPLES)
      {
        Serial.println();
        Serial.println("========================================");
        Serial.println("MPU6050: VEHICLE FLIPPED");
        Serial.println("========================================");

        triggerAccident();
        carStatus = "Flipped";
        detectionReason = "Vehicle flipped";
        sensorAccidentSamples = 0;
      }
    }
    else if (sensorCrashEvent)
    {
      sensorAccidentSamples++;

      if (impactEvent)
        detectionReason = "High impact";
      else if (vibrationEvent)
        detectionReason = "Sudden vibration";
      else
        detectionReason = "High rotation";

      if (sensorAccidentSamples >= SENSOR_CONFIRM_SAMPLES)
      {
        Serial.println();
        Serial.println("========================================");
        Serial.println("REAL MPU6050 CRASH DETECTED");
        Serial.println("========================================");

        Serial.print("Impact: ");
        Serial.print(impactG, 2);
        Serial.println(" g");

        Serial.print("Vibration: ");
        Serial.print(vibrationG, 2);
        Serial.println(" g");

        Serial.print("Gyro: ");
        Serial.print(gyro, 2);
        Serial.println(" deg/s");

        Serial.print("Relative tilt: ");
        Serial.print(relativeTilt, 2);
        Serial.println(" deg");

        triggerAccident();
        carStatus = "Crash";
        sensorAccidentSamples = 0;
      }
    }
    else
    {
      sensorAccidentSamples = 0;
      detectionReason = "Monitoring";
    }
  }
  else if (systemState == NORMAL)
  {
    sensorAccidentSamples = 0;
    detectionReason = "Startup stabilization";
  }
  else
  {
    sensorAccidentSamples = 0;
  }

  // Keep status correct after a button-triggered accident.
  if (systemState != NORMAL && !carFlipped)
    carStatus = "Crash";

  // ----------------------------------------------------
  // SERIAL LIVE DATA
  // ----------------------------------------------------

  static unsigned long lastSensorPrint = 0;

  if (millis() - lastSensorPrint >= 500)
  {
    lastSensorPrint = millis();

    Serial.print("ACC X: ");
    Serial.print(accelX, 2);
    Serial.print(" g | Y: ");
    Serial.print(accelY, 2);
    Serial.print(" g | Z: ");
    Serial.print(accelZ, 2);
    Serial.print(" g | ");

    Serial.print("GYRO X: ");
    Serial.print(gyroX, 1);
    Serial.print(" | Y: ");
    Serial.print(gyroY, 1);
    Serial.print(" | Z: ");
    Serial.print(gyroZ, 1);
    Serial.print(" deg/s | ");

    Serial.print("IMPACT: ");
    Serial.print(impactG, 2);
    Serial.print(" g | VIBRATION: ");
    Serial.print(vibrationG, 2);
    Serial.print(" g | ROLL: ");
    Serial.print(rollAngle, 1);
    Serial.print(" | PITCH: ");
    Serial.print(pitchAngle, 1);
    Serial.print(" | TILT: ");
    Serial.print(relativeTilt, 1);
    Serial.print(" | SEPI: ");
    Serial.print(sepi, 1);
    Serial.print(" | STATUS: ");
    Serial.println(carStatus);
  }
}


// ======================================================
// INITIALIZE MPU6050
// ======================================================

bool initMPU6050()
{
  MPU_I2C.begin(MPU_SDA, MPU_SCL, 400000);
  delay(100);

  Serial.println();
  Serial.println("Scanning MPU6050 I2C bus...");
  Serial.println("SDA = GPIO 25");
  Serial.println("SCL = GPIO 26");

  byte possibleAddresses[] = {0x68, 0x69};
  bool found = false;

  for (byte i = 0; i < 2; i++)
  {
    byte address = possibleAddresses[i];
    MPU_I2C.beginTransmission(address);
    byte error = MPU_I2C.endTransmission();

    if (error == 0)
    {
      mpuAddress = address;
      found = true;
      Serial.print("I2C device found at 0x");
      if (address < 0x10) Serial.print("0");
      Serial.println(address, HEX);
      break;
    }
  }

  if (!found)
  {
    Serial.println("No I2C device found at 0x68 or 0x69.");
    return false;
  }

  MPU_I2C.beginTransmission(mpuAddress);
  MPU_I2C.write(0x75);
  if (MPU_I2C.endTransmission(false) != 0) return false;

  if (MPU_I2C.requestFrom(mpuAddress, (uint8_t)1, true) != 1) return false;

  byte whoAmI = MPU_I2C.read();

  // MPU6050 normally reports 0x68.
  // Some boards sold as MPU6050 use the MPU6500-compatible silicon,
  // which reports 0x70. Both use the same basic accel/gyro registers
  // used by this project.
  if (whoAmI != 0x68 && whoAmI != 0x69 && whoAmI != 0x70)
  {
    Serial.print("Unexpected WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);
    return false;
  }

  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);

  if (whoAmI == 0x70)
  {
    Serial.println("MPU6500-compatible sensor detected.");
    Serial.println("Using MPU6050-compatible accel/gyro registers.");
  }

  // Wake sensor.
  MPU_I2C.beginTransmission(mpuAddress);
  MPU_I2C.write(0x6B);
  MPU_I2C.write(0x00);
  if (MPU_I2C.endTransmission() != 0) return false;

  // Digital low-pass filter.
  MPU_I2C.beginTransmission(mpuAddress);
  MPU_I2C.write(0x1A);
  MPU_I2C.write(0x03);
  if (MPU_I2C.endTransmission() != 0) return false;

  // Accelerometer +/-2g.
  MPU_I2C.beginTransmission(mpuAddress);
  MPU_I2C.write(0x1C);
  MPU_I2C.write(0x00);
  if (MPU_I2C.endTransmission() != 0) return false;

  // Gyroscope +/-250 deg/s.
  MPU_I2C.beginTransmission(mpuAddress);
  MPU_I2C.write(0x1B);
  MPU_I2C.write(0x00);
  if (MPU_I2C.endTransmission() != 0) return false;

  delay(100);
  mpuConnected = true;

  Serial.println("MPU SENSOR CONNECTED");
  Serial.print("Address = 0x");
  Serial.println(mpuAddress, HEX);
  Serial.println("Accelerometer = +/-2g");
  Serial.println("Gyroscope = +/-250 deg/s");
  Serial.println("Live sensor reading enabled.");

  return true;
}


// ======================================================
// CALIBRATE MPU6050 NORMAL ORIENTATION
// ======================================================

void calibrateMPU6050()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("MPU6050 CALIBRATION");
  Serial.println("Place the sensor in the FINAL NORMAL");
  Serial.println("vehicle position and KEEP IT STILL.");
  Serial.println("========================================");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU CALIBRATE");
  lcd.setCursor(0, 1);
  lcd.print("KEEP STILL");

  // Give the sensor time to settle.
  delay(3000);

  float sumX = 0.0;
  float sumY = 0.0;
  float sumZ = 0.0;

  float sumGx = 0.0;
  float sumGy = 0.0;
  float sumGz = 0.0;

  int validSamples = 0;

  // Average many samples while the sensor is in its true
  // normal mounted position.
  for (int i = 0; i < 200; i++)
  {
    float ax, ay, az;
    float gx, gy, gz;

    if (readMPU6050(ax, ay, az, gx, gy, gz))
    {
      sumX += ax;
      sumY += ay;
      sumZ += az;

      sumGx += gx;
      sumGy += gy;
      sumGz += gz;

      validSamples++;
    }

    delay(10);
  }

  if (validSamples < 100)
  {
    Serial.println("Calibration failed: too few valid samples.");
    return;
  }

  normalAx = sumX / validSamples;
  normalAy = sumY / validSamples;
  normalAz = sumZ / validSamples;

  gyroBiasX = sumGx / validSamples;
  gyroBiasY = sumGy / validSamples;
  gyroBiasZ = sumGz / validSamples;

  previousAccelX = normalAx;
  previousAccelY = normalAy;
  previousAccelZ = normalAz;

  acceleration = sqrt(
    normalAx * normalAx +
    normalAy * normalAy +
    normalAz * normalAz
  );

  impactG = acceleration;
  vibrationG = 0.0;
  motionIntensity = 0.0;
  gyro = 0.0;
  rollAngle = atan2(normalAy, normalAz) * 180.0 / PI;
  pitchAngle = atan2(
    -normalAx,
    sqrt(normalAy * normalAy + normalAz * normalAz)
  ) * 180.0 / PI;

  relativeTilt = 0.0;
  tiltAngle = 0.0;
  carFlipped = false;
  carStatus = "Normal";

  Serial.println();
  Serial.println("Calibration complete.");

  Serial.print("Normal X = ");
  Serial.println(normalAx, 3);

  Serial.print("Normal Y = ");
  Serial.println(normalAy, 3);

  Serial.print("Normal Z = ");
  Serial.println(normalAz, 3);

  Serial.print("Gyro bias X = ");
  Serial.println(gyroBiasX, 3);

  Serial.print("Gyro bias Y = ");
  Serial.println(gyroBiasY, 3);

  Serial.print("Gyro bias Z = ");
  Serial.println(gyroBiasZ, 3);

  Serial.println("Normal position stored successfully.");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU READY");
  lcd.setCursor(0, 1);
  lcd.print("Normal Stored");

  delay(1200);
}


// ======================================================
// READ RAW MPU6050 DATA
// ======================================================

bool readMPU6050(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
)
{
  MPU_I2C.beginTransmission(mpuAddress);
  MPU_I2C.write(0x3B);

  if (MPU_I2C.endTransmission(false) != 0)
  {
    return false;
  }

  int received =
    MPU_I2C.requestFrom(
      mpuAddress,
      14,
      true
    );

  if (received != 14)
  {
    return false;
  }

  int16_t rawAx =
    (int16_t)(
      ((uint16_t)MPU_I2C.read() << 8) |
      MPU_I2C.read()
    );

  int16_t rawAy =
    (int16_t)(
      ((uint16_t)MPU_I2C.read() << 8) |
      MPU_I2C.read()
    );

  int16_t rawAz =
    (int16_t)(
      ((uint16_t)MPU_I2C.read() << 8) |
      MPU_I2C.read()
    );

  // Temperature bytes - not required.
  MPU_I2C.read();
  MPU_I2C.read();

  int16_t rawGx =
    (int16_t)(
      ((uint16_t)MPU_I2C.read() << 8) |
      MPU_I2C.read()
    );

  int16_t rawGy =
    (int16_t)(
      ((uint16_t)MPU_I2C.read() << 8) |
      MPU_I2C.read()
    );

  int16_t rawGz =
    (int16_t)(
      ((uint16_t)MPU_I2C.read() << 8) |
      MPU_I2C.read()
    );

  // +/-2g -> 16384 LSB/g
  ax = rawAx / 16384.0;
  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;

  // +/-250 deg/s -> 131 LSB/(deg/s)
  gx = rawGx / 131.0;
  gy = rawGy / 131.0;
  gz = rawGz / 131.0;

  return true;
}


// ======================================================
// EVENT TIME
// ======================================================

String getTimeString()
{

  unsigned long totalSeconds =
    millis() / 1000;


  unsigned long hours =
    totalSeconds / 3600;


  unsigned long minutes =
    (
      totalSeconds % 3600
    ) / 60;


  unsigned long seconds =
    totalSeconds % 60;


  char buffer[20];


  sprintf(
    buffer,
    "%02lu:%02lu:%02lu",
    hours,
    minutes,
    seconds
  );


  return String(
    buffer
  );
}


// ======================================================
// ROOT PAGE
// ======================================================

void handleRoot()
{

  server.send_P(
    200,
    "text/html",
    MAIN_PAGE
  );
}


// ======================================================
// STATUS API
// ======================================================

void handleStatus()
{
  String json = "{";

  json += "\"accident\":" +
          String(systemState != NORMAL ? "true" : "false");

  json += ",\"confirmed\":" +
          String(systemState == ACCIDENT_CONFIRMED ? "true" : "false");

  json += ",\"alertSent\":" +
          String(smsSent ? "true" : "false");

  // Live accelerometer axes
  json += ",\"accelX\":" + String(accelX, 3);
  json += ",\"accelY\":" + String(accelY, 3);
  json += ",\"accelZ\":" + String(accelZ, 3);

  // Live gyroscope axes
  json += ",\"gyroX\":" + String(gyroX, 2);
  json += ",\"gyroY\":" + String(gyroY, 2);
  json += ",\"gyroZ\":" + String(gyroZ, 2);

  // Derived values
  json += ",\"acceleration\":" + String(acceleration, 2);
  json += ",\"gyro\":" + String(gyro, 2);
  json += ",\"motion\":" + String(motionIntensity, 2);
  json += ",\"impact\":" + String(impactG, 2);
  json += ",\"vibration\":" + String(vibrationG, 2);
  json += ",\"roll\":" + String(rollAngle, 2);
  json += ",\"pitch\":" + String(pitchAngle, 2);
  json += ",\"tilt\":" + String(tiltAngle, 2);
  json += ",\"sepi\":" + String(sepi, 1);

  json += ",\"flipped\":" +
          String(carFlipped ? "true" : "false");

  json += ",\"carStatus\":\"" + carStatus + "\"";
  json += ",\"reason\":\"" + detectionReason + "\"";

  // GPS
  json += ",\"latitude\":" + String(latitude, 6);
  json += ",\"longitude\":" + String(longitude, 6);
  json += ",\"location\":\"" + locationName + "\"";
  json += ",\"satellites\":" + String(satellites);

  json += ",\"eventTime\":\"" + eventTime + "\"";

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}


// ======================================================
// SOS TEST FROM WEB DASHBOARD
// ======================================================

void handleAccident()
{

  if(
    systemState ==
    NORMAL
  )
  {

    triggerAccident();


    server.send(
      200,
      "text/plain",
      "ACCIDENT DETECTED"
    );
  }

  else
  {

    server.send(
      200,
      "text/plain",
      "SYSTEM ALREADY ACTIVE"
    );
  }
}