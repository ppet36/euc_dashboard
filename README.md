# EUC dashboard
Dashboard with ESP32, [Nextion NX4024T032](https://nextion.tech/datasheets/nx4024t032/) display for my DIY EUC based on Begode Nikola control board and [JK Smart BMS BD6A24S10P](https://www.jkbms.com/product/jk-bd6a24s10p/). I built them because the EUCs that are available in the market either have a small battery or are extremely expensive.

In my EUC there is a 24s10p battery composed of Samsung INR21700-50E 4900mAh cells with a total capacity of almost 5kWh. This allows me to go on all-day trips and up to a distance of 160 km on a single charge. I carry the charger with me, so I can recharge for additional kilometers during lunch or sightseeing at the destination.

![alt](/images/dash_int.jpeg?raw=true)
![alt](/images/dash.jpeg?raw=true)

The dashboard is attached to the EUC frame with Velcro. It is only powered from the EUC and all communication is via BlueTooth (BLE). It is connected to both EUC and BMS simultaneously and displays data from both boards.

![alt](/images/dash_euc.jpeg?raw=true)

The video of the older version is on YouTube [Test of EUC dashboard](https://youtu.be/FmaS4RxT6nU). The latest version is not on the video. In the latest version, it is also possible to modify some EUC parameters such as tiltback speed and pedal mode.

