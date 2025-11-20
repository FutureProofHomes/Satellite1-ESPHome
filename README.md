<a name="readme-top"></a>
<!--
*** Readme based upon Best-README-Template.
-->



<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/FutureProofHomes/Satellite1-ESPHome">
    <img src="assets/images/logo.png" alt="Logo" width="80" height="80" style="border-radius:10%">
  </a>

<h3 align="center">Satellite1 Core Board ESPHome Firmware</h3>

  <p align="center">
    Open-Source ESPHome Firmware for Your Private AI-Powered Satellite1 Voice Assistant & Multisensor
    <br />
    <a href="https://docs.futureproofhomes.net"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://www.youtube.com/@futureproofhomes">View Demos</a>
    ·
    <a href="https://github.com/FutureProofHomes/Satellite1-ESPHome/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/FutureProofHomes/Satellite1-ESPHome/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About the Project
The Satellite1 ESPHome firmware should be flashed on your [FutureProofHomes Core Board](https://futureproofhomes.net/products/satellite1-core-board). For flashing instructions please visit [Docs.FutureProofHomes.net](https://docs.futureproofhomes.net).  After the firmware is successfully flashed and your Core Board is connected to your Wifi it will appear in your Home Assistant as a new device called "Satellite1".

## Key Features of the Firmware
- Works with the Home Assistant Platform so you can control your home.
- This firmware enables your FutureProofHomes Core Board to mount with our [HAT board](https://futureproofhomes.net/products/satellite1-top-microphone-board) which then unlocks:
- On-Demand flashing of our open source XMOS firmware for audio echo cancellation and other audio processing algorithms.
- [On-Device WakeWord support.](https://github.com/kahrendt/microWakeWord)
- Temperature/Humidity/Light sensor readings of the room
- Attachable mmWave Radar for Human Presence Detection
- Music streaming via HA Media Browser or [Music Assistant](https://music-assistant.io/)
- Volume Up/Down & Action Buttons
- Hardware & Software Mute Buttons
- 360 degree LEDs & Notification Animations
- Support for TTS Announcements via Home Assistant
- USB-C Power Delivery for easy power input


## Why Open Source?
We believe it is irresponsible to ask customers to trust that our microphone and AI in-a-box protects your privacy.  To hold ourselves and the whole world accountable it is prudent to open-source our work so that we can all benefit from this amazing technology.  Let's build together.

## Why Purchase from FutureProofHomes?
Put simply, your purchase helps fund our team and further innovation.  Also, the FutureProofHomes team will work hard to give you top-quality products that are tested, fully-functional, in stock (as often as possible) and lead with great community support.  You can purchase Satellite1 components individually, or purchase the entire devkit as a package.  Help us, help you!

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

Go to [Docs.FutureProofHomes.net](https://docs.futureproofhomes.net) and follow the instructions to assemble, flash and set up your Core Board.

<!-- This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps. -->

### Prerequisites

- FutureProofHomes Core Board & USB-C cable to plug into your computer.
- Highly recommend our FutureProofHomes HAT board to unlock all the features.

<!-- This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g
  ``` -->


<!-- USAGE EXAMPLES -->
## Usage

<!-- Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources. -->

_For more examples, please refer to the [Documentation](https://docs.futureproofhomes.net)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Core Board Roadmap

See the [open issues](https://github.com/FutureProofHomes/Satellite1-ESPHome/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
3. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
4. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
5. Push to the Branch (`git push origin feature/AmazingFeature`)
6. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Customizing the Firmware
### ESPHome Device Builder
The easiest way to build customized firmware for your Satellite1 is through the ESPHome Device Builder dashboard. For a detailed guide, see https://docs.futureproofhomes.net/satellite1-modifying-the-firmware/. 

>**Note:** Due to breaking changes between ESPHome firmware releases, you must ensure that your ESPHome Device Builder version is compatible with the Satellite1 codebase configured in your YAML file. By default, the codebase targets the latest beta release (`staging` branch).
Before updating your ESPHome Device Builder, verify that the latest Satellite1 firmware beta release officially supports the ESPHome version. Failure to do so may result in build failures or unstable device behavior.


|Code Base|FW Release|ESPHome|
|-----|-----|-----|
| `develop`|-|![Dynamic YAML Badge](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fgithub.com%2FFutureProofHomes%2FSatellite1-ESPHome%2Fraw%2Fdevelop%2F.github%2Fworkflows%2Fbuild_latest.yaml&query=%24..esphome-version&label=ESPHome)| 
|`staging` (**default**)|![GitHub Release](https://img.shields.io/github/v/release/FutureProofHomes/Satellite1-ESPHome?filter=*-beta*)|![Dynamic YAML Badge](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fgithub.com%2FFutureProofHomes%2FSatellite1-ESPHome%2Fraw%2Fstaging%2F.github%2Fworkflows%2Fbuild_latest.yaml&query=%24..esphome-version&label=ESPHome)|
| `main`|![GitHub Release](https://img.shields.io/github/v/release/FutureProofHomes/Satellite1-ESPHome?filter=!*-beta*)|![Dynamic YAML Badge](https://img.shields.io/badge/dynamic/yaml?url=https%3A%2F%2Fgithub.com%2FFutureProofHomes%2FSatellite1-ESPHome%2Fraw%2Fmain%2F.github%2Fworkflows%2Fbuild_latest.yaml&query=%24..esphome-version&label=ESPHome)| 


### Terminal Builds
Create/activate environment by running from project root:
```bash
source scripts/setup_build_env.sh
```

Build firmware on your local machine:
```bash
esphome compile config/satellite1.yaml
```

Upload firmware to your Core Board:
```bash
esphome upload config/satellite1.yaml
```

Tail the ESPHome logs:
```bash
esphome logs config/satellite1.yaml
```
For WiFi setup and troubleshooting see also:
1. [Flashing via usb-c](https://docs.futureproofhomes.net/satellite1-flash-via-usb-c/)
2. [Troubleshooting](https://docs.futureproofhomes.net/satellite1-troubleshooting/)

## Home Assistant Voice Assistant Debugging

1. [Set up you local pipeline](https://www.home-assistant.io/voice_control/voice_remote_local_assistant/)
2. [Troubleshoot your pipeline](https://www.home-assistant.io/voice_control/troubleshooting/)


<!-- LICENSE -->
## License

Distributed under the ESPHOME License. See `LICENSE` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

FutureProofHomes  - [Website](https://futureproofhomes.net/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- YouTube -->
## YouTube

Checkout out our growing YouTube Channel  - [YouTube.com/@FutureProofHomes](https://www.youtube.com/@futureproofhomes)


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* @gnumpi for all the amazing C code
* @qnlbnsl for all the Github Action & automated release work
* [Nabu Casa](https://nabucasa.com) for making this all possible
* Your name here soon...

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/FutureProofHomes/Satellite1-ESPHome.svg?style=for-the-badge
[contributors-url]: https://github.com/FutureProofHomes/Satellite1-ESPHome/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/FutureProofHomes/Satellite1-ESPHome.svg?style=for-the-badge
[forks-url]: https://github.com/FutureProofHomes/Satellite1-ESPHome/network/members
[stars-shield]: https://img.shields.io/github/stars/FutureProofHomes/Satellite1-ESPHome.svg?style=for-the-badge
[stars-url]: https://github.com/FutureProofHomes/Satellite1-ESPHome/stargazers
[issues-shield]: https://img.shields.io/github/issues/FutureProofHomes/Satellite1-ESPHome.svg?style=for-the-badge
[issues-url]: https://github.com/FutureProofHomes/Satellite1-ESPHome/issues
[license-shield]: https://img.shields.io/github/license/FutureProofHomes/Satellite1-ESPHome.svg?style=for-the-badge
[license-url]: https://github.com/FutureProofHomes/Satellite1-ESPHome/blob/master/LICENSE
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/linkedin_username
[genaimockup]: assets/images/mockup.png
[combo_render]: assets/images/combo_render.png
[kicad.org]: https://img.shields.io/badge/KiCad-314CB0?style=for-the-badge&logo=kicad&logoColor=white
[kicad-url]: https://www.kicad.org/
[esphome.io]: https://img.shields.io/badge/-ESPHome-000000?style=for-the-badge&logo=esphome&logoColor=white
[esphome-url]: https://esphome.io/
