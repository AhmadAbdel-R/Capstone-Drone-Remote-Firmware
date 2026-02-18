#pragma once

// Compatible selector flow for PlatformIO + local setup headers.
#ifdef TFT_ESPI_USER_SETUP_PATH
  #ifndef USER_SETUP_LOADED
    #define USER_SETUP_LOADED
  #endif
  #include TFT_ESPI_USER_SETUP_PATH
#endif

#ifndef USER_SETUP_LOADED
  #include <User_Setup.h>
#endif

#define TFT_BGR 0
#define TFT_RGB 1

#if defined(ST7789_DRIVER)
  #include <TFT_Drivers/ST7789_Defines.h>
  #define TFT_DRIVER 0x7789
#else
  #define TFT_DRIVER 0x0000
#endif
