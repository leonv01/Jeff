#!/usr/bin/env python3
"""
PCA9685 Channel Test Utility
Allows testing individual channels on PCA9685 boards (0x40 / 0x41)
to verify physical wiring and servo movements.
"""

import sys
import time
import smbus2

# PCA9685 Registers
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09

class PCA9685Controller:
    def __init__(self, bus_num=1, address=0x40):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.reset()
        self.set_pwm_freq(50)

    def reset(self):
        self.bus.write_byte_data(self.address, MODE1, 0x00)
        time.sleep(0.01)

    def set_pwm_freq(self, freq_hz):
        prescaleval = 25000000.0 / 4096.0 / float(freq_hz) - 1.0
        prescale = int(prescaleval + 0.5)
        oldmode = self.bus.read_byte_data(self.address, MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.bus.write_byte_data(self.address, MODE1, newmode)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on_tick, off_tick):
        base_reg = LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, base_reg, on_tick & 0xFF)
        self.bus.write_byte_data(self.address, base_reg + 1, (on_tick >> 8) & 0xFF)
        self.bus.write_byte_data(self.address, base_reg + 2, off_tick & 0xFF)
        self.bus.write_byte_data(self.address, base_reg + 3, (off_tick >> 8) & 0xFF)

    def set_servo_angle(self, channel, angle_deg, min_ticks=100, max_ticks=500):
        angle_deg = max(0.0, min(180.0, float(angle_deg)))
        off_tick = int(min_ticks + (angle_deg / 180.0) * (max_ticks - min_ticks))
        self.set_pwm(channel, 0, off_tick)
        return off_tick

def main():
    print("=== PCA9685 Channel Test Utility ===")
    board_addr_str = input("Select Board Address [0x40 (default) / 0x41]: ").strip()
    address = int(board_addr_str, 16) if board_addr_str else 0x40

    try:
        pca = PCA9685Controller(bus_num=1, address=address)
        print(f"Connected to PCA9685 at address {hex(address)}")
    except Exception as e:
        print(f"Error connecting to I2C device {hex(address)}: {e}")
        sys.exit(1)

    print("\nModes:")
    print(" 1. Sweep LF Channels (13, 14, 15) one by one")
    print(" 2. Test a specific channel manually (enter channel 0-15 and angle 0-180)")
    choice = input("Select option [1/2]: ").strip()

    if choice == "1":
        channels = [15, 14, 13]
        joint_names = ["Channel 15 (Coxa?)", "Channel 14 (Femur?)", "Channel 13 (Tibia?)"]
        for ch, name in zip(channels, joint_names):
            print(f"\n--- Testing {name} ---")
            print(f"Moving {hex(address)} Ch {ch} to 0 deg...")
            pca.set_servo_angle(ch, 0)
            time.sleep(1.0)
            print(f"Moving {hex(address)} Ch {ch} to 90 deg (Center)...")
            pca.set_servo_angle(ch, 90)
            time.sleep(1.0)
            print(f"Moving {hex(address)} Ch {ch} to 180 deg...")
            pca.set_servo_angle(ch, 180)
            time.sleep(1.0)
            print(f"Returning {hex(address)} Ch {ch} to 90 deg...")
            pca.set_servo_angle(ch, 90)
            time.sleep(0.5)
        print("\nSweep complete!")

    elif choice == "2":
        while True:
            try:
                ch_str = input("\nEnter channel number (0-15) or 'q' to quit: ").strip()
                if ch_str.lower() == 'q':
                    break
                ch = int(ch_str)
                angle_str = input(f"Enter target angle in degrees (0 - 180) for Ch {ch}: ").strip()
                angle = float(angle_str)
                ticks = pca.set_servo_angle(ch, angle)
                print(f"Set Ch {ch} to {angle}° ({ticks} PWM ticks)")
            except ValueError:
                print("Invalid input! Please enter numbers.")
            except Exception as e:
                print(f"Error setting PWM: {e}")

if __name__ == "__main__":
    main()
