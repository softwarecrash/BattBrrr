#pragma once

// Central settings list for the BattBrrr Controller Preferences-based Settings.
// TYPE,   GROUP,      NAME,              API_NAME,           DEFAULT,        MIN,    MAX
// GROUP = JSON group + NVS namespace
// NAME  = JSON field name + NVS key
// API_NAME = name of getter/setter functions in get./set.
//
// Supported TYPE values: BOOL, INT32, UINT16, UINT32, FLOAT, STRING

#define SETTINGS_ITEMS(X) \
  /* ---- Network section ---- */ \
  X(STRING, "network",   "deviceName",         deviceName,       "BattBrrr",      0,    0) \
  X(STRING, "network",   "wifiSsid0",          wifiSsid0,        "",              0,    0) \
  X(STRING, "network",   "wifiBssid0",         wifiBssid0,       "",              0,    0) \
  X(BOOL,   "network",   "wifiBssidLock",      wifiBssidLock,    false,           0,    0) \
  X(STRING, "network",   "wifiPass0",          wifiPass0,        "",              0,    0) \
  X(STRING, "network",   "wifiSsid1",          wifiSsid1,        "",              0,    0) \
  X(STRING, "network",   "wifiPass1",          wifiPass1,        "",              0,    0) \
  X(STRING, "network",   "staticIP",           staticIP,         "",              0,    0) \
  X(STRING, "network",   "staticGW",           staticGW,         "",              0,    0) \
  X(STRING, "network",   "staticSN",           staticSN,         "",              0,    0) \
  X(STRING, "network",   "staticDNS",          staticDNS,        "",              0,    0) \
  X(STRING, "network",   "webUIuser",          webUIuser,        "",              0,    0) \
  X(STRING, "network",   "webUIPass",          webUIPass,        "",              0,    0) \
  \
  /* ---- Control section ---- */ \
  X(BOOL,   "control",   "enabled",            enabled,          false,           0,    0) \
  X(INT32,  "control",   "mode",               mode,             0,               0,    4) \
  X(BOOL,   "control",   "frostEnable",        frostEnable,      true,            0,    0) \
  X(FLOAT,  "control",   "targetIdleC",        targetIdleC,      5.0,           -40,  80) \
  X(FLOAT,  "control",   "targetChargeC",      targetChargeC,    15.0,          -40,  80) \
  X(FLOAT,  "control",   "targetDischargeC",   targetDischargeC, 15.0,          -40,  80) \
  X(FLOAT,  "control",   "targetFrostC",       targetFrostC,     2.0,           -40,  80) \
  X(INT32,  "control",   "algorithm",          algorithm,        0,              0,    1) \
  X(FLOAT,  "control",   "pidKp",              pidKp,            10.0,            0,  1000) \
  X(FLOAT,  "control",   "pidKi",              pidKi,            0.05,            0,   100) \
  X(FLOAT,  "control",   "pidKd",              pidKd,            0.0,             0,   100) \
  X(FLOAT,  "control",   "pidIntegralLimit",   pidIntegralLimit, 30.0,            0,  1000) \
  X(FLOAT,  "control",   "pidDerivFilter",     pidDerivFilter,   0.1,             0,     1) \
  X(FLOAT,  "control",   "hystOnDelta",        hystOnDelta,      1.0,           0.1,    20) \
  X(FLOAT,  "control",   "hystOffDelta",       hystOffDelta,     0.5,           0.1,    20) \
  X(FLOAT,  "control",   "manualOutputPct",    manualOutputPct,  50.0,            0,   100) \
  X(FLOAT,  "control",   "maxOutputPct",       maxOutputPct,     100.0,           0,   100) \
  X(UINT32, "control",   "minOnMs",            minOnMs,          2000,            0, 600000) \
  X(UINT32, "control",   "minOffMs",           minOffMs,         2000,            0, 600000) \
  X(UINT32, "control",   "sensorPollMs",       sensorPollMs,     2000,          250, 60000) \
  X(UINT16, "control",   "sensorFailCount",    sensorFailCount,  3,               1,    20) \
  X(UINT16, "control",   "sensorRescanMin",    sensorRescanMin,  10,              0,  1440) \
  \
  /* ---- Safety section ---- */ \
  X(FLOAT,  "safety",    "maxTempC",           maxTempC,         50.0,          -20,   120) \
  X(FLOAT,  "safety",    "maxDeltaC",          maxDeltaC,         5.0,            0,    50) \
  X(FLOAT,  "safety",    "stuckOnPct",         stuckOnPct,       70.0,            0,   100) \
  X(UINT32, "safety",    "stuckOnS",           stuckOnS,         300,            10, 36000) \
  X(FLOAT,  "safety",    "minRiseC",           minRiseC,          1.0,          0.1,    20) \
  X(UINT32, "safety",    "riseWindowS",        riseWindowS,      300,            10, 36000) \
  X(BOOL,   "safety",    "runawayEnable",      runawayEnable,    true,            0,     0) \
  X(FLOAT,  "safety",    "runawayRateCPerMin", runawayRateCPerMin, 5.0,         0.1,   100) \
  X(UINT32, "safety",    "runawayWindowS",     runawayWindowS,   120,            10, 36000) \
  X(FLOAT,  "safety",    "runawayMarginC",     runawayMarginC,    5.0,         0.1,    50) \
  X(BOOL,   "safety",    "runawayLatch",       runawayLatch,     true,            0,     0) \
  \
  /* ---- GPIO section ---- */ \
  X(INT32,  "gpio",      "oneWirePin",         oneWirePin,       -1,             -1,    48) \
  X(INT32,  "gpio",      "heaterOutPin",       heaterOutPin,     -1,             -1,    48) \
  X(BOOL,   "gpio",      "heaterOutInvert",    heaterOutInvert,  false,           0,     0) \
  X(INT32,  "gpio",      "heaterOutType",      heaterOutType,     1,              0,     1) \
  X(UINT32, "gpio",      "pwmFreq",            pwmFreq,          1000,           10,  40000) \
  X(UINT16, "gpio",      "pwmResolution",      pwmResolution,    10,              8,    14) \
  X(UINT32, "gpio",      "windowMs",           windowMs,         2000,          200, 600000) \
  X(INT32,  "gpio",      "enableInPin",        enableInPin,      -1,             -1,    48) \
  X(INT32,  "gpio",      "enableInPull",       enableInPull,      0,              0,     2) \
  X(INT32,  "gpio",      "enableInActive",     enableInActive,    0,              0,     1) \
  X(UINT16, "gpio",      "enableInDebounce",   enableInDebounce, 50,              0,  1000) \
  X(INT32,  "gpio",      "modeInPin",          modeInPin,        -1,             -1,    48) \
  X(INT32,  "gpio",      "modeInPull",         modeInPull,        0,              0,     2) \
  X(INT32,  "gpio",      "modeInActive",       modeInActive,      0,              0,     1) \
  X(UINT16, "gpio",      "modeInDebounce",     modeInDebounce,   50,              0,  1000) \
  X(INT32,  "gpio",      "manualInPin",        manualInPin,      -1,             -1,    48) \
  X(INT32,  "gpio",      "manualInPull",       manualInPull,      0,              0,     2) \
  X(INT32,  "gpio",      "manualInActive",     manualInActive,    0,              0,     1) \
  X(UINT16, "gpio",      "manualInDebounce",   manualInDebounce, 50,              0,  1000) \
  \
  /* ---- MQTT section ---- */ \
  X(BOOL,   "mqtt",      "mqttEnable",         mqttEnable,       false,           0,     0) \
  X(STRING, "mqtt",      "mqttHost",           mqttHost,         "",              0,     0) \
  X(UINT16, "mqtt",      "mqttPort",           mqttPort,         1883,            1, 65535) \
  X(STRING, "mqtt",      "mqttUser",           mqttUser,         "",              0,     0) \
  X(STRING, "mqtt",      "mqttPass",           mqttPass,         "",              0,     0) \
  X(STRING, "mqtt",      "mqttClientId",       mqttClientId,     "",              0,     0) \
  X(STRING, "mqtt",      "mqttBaseTopic",      mqttBaseTopic,    "battbrrr",      0,     0) \
  X(UINT16, "mqtt",      "mqttKeepaliveS",     mqttKeepaliveS,   30,              5,   600) \
  X(UINT16, "mqtt",      "mqttPublishS",       mqttPublishS,     5,               1,  3600) \
  X(BOOL,   "mqtt",      "mqttRetain",         mqttRetain,       false,           0,     0) \
  \
  /* ---- BMS section ---- */ \
  X(STRING, "bms",       "bmsStateTopic",      bmsStateTopic,    "",              0,     0) \
  X(STRING, "bms",       "bmsTempTopic",       bmsTempTopic,     "",              0,     0) \
  X(STRING, "bms",       "bmsStatePath",       bmsStatePath,     "",              0,     0) \
  X(STRING, "bms",       "bmsTempPath",        bmsTempPath,      "",              0,     0) \
  X(UINT16, "bms",       "bmsTimeoutS",        bmsTimeoutS,      60,              1,  3600) \
  X(BOOL,   "bms",       "bmsFallback",        bmsFallback,      false,           0,     0) \
  \
  /* ---- Failsafe section ---- */ \
  X(INT32,  "failsafe",  "mqttLossMode",       mqttLossMode,     1,               0,     3) \
  X(UINT16, "failsafe",  "mqttTimeoutS",       mqttTimeoutS,     60,              1,  3600) \
  \
  /* ---- Sensors section ---- */ \
  X(STRING, "sensors",   "sensorsJson",        sensorsJson,      "[]",            0,     0) \
  /* End of settings items */
