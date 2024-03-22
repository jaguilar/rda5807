# RDA5807 Linux I2C Userspace Driver

This driver is only half complete. I wrote it to try to debug issues
I was having with an ICSTATION FM Radio kit 
([this one](https://www.amazon.com/gp/product/B09Y1BJZ6F/)). I couldn't
get the radio to do anything, and one of my friends gave me the idea of
trying to control the receiver directly through its I2C interface from
my Raspberry Pi.

I was able to get this device to partially work through the interface. I found
I was able to set the volume and mute the device. However, I could not cause
the device to seek nor could I get it to tune in to a new station. I was following
[this](https://web.archive.org/web/20240229170323/https://cdn-shop.adafruit.com/product-files/5651/5651_tuner84_RDA5807M_datasheet_v1.pdf)
datasheet to get information on the receiver's registers. However, I think that
the actual receiver I have is a RDA5807P, not M. I have no idea if this would
change how the receiver is programmed.

Anyway, maybe this will be helpful to someone. It was interesting at least to learn
how to interact with I2C via Linux!s