# Bluetooth-Health-Temperature-Measurement
A Bluetooth Low Energy (BLE) implementation of a health temperature measurement system using the Blue Gecko BGM121 module

Features:\
-- Temperature measurement performed using Si7021 onboard temperature and relative humidity sensor\
-- Custom I2C driver for taking temperature measurement and load power management (LPM) of Si7021\
-- Automated temperature indications at 4 s intervals using the onboard low-energy LETIMER\
-- Functionality incorporated using the HTM (health thermometer) service from the Bluetooth stack\
-- Adaptive Tx power adjustment based on client proximity using RSSI values and the Tx power service\
-- Capability for signed and secired over-the-air (OTA) firmware updates\
-- Low-power implementation based on sleep modes and wait for event (WFE) functionality

Implementation:\
-- Hardware development platform: Silicon Labs wireless starter kit SLWSTK6010B, along with a Blue Gecko BGM121 BLE module\
-- IDE and tools: Silicon Labs Simplicity Studio\
-- Application interface: Silicon Labs Blue Gecko Android app\
-- Language: C
