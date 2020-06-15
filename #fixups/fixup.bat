@rem There are several hacks made that are outside the 'protected' areas of
@rem generated code, alas, and will be overwritten each time STM32CubeMX is
@rem re-run (e.g. if you change some settings).  This will restore those hacks.

@rem put in our (expanded) heap manager
@rem there is now an 'advanced' config option on the freertos middleware in
@rem cubemx where we can omit the out-of-box allocator, allowing us to simply
@rem put our implementation in /src and be done with it
rem del ..\Middlewares\Third_Party\FreeRTOS\Source\portable\MemMang\heap_4.c
rem copy heap_x.c.MyHeap ..\Middlewares\Third_Party\FreeRTOS\Source\portable\MemMang\heap_x.c

@rem put in our (augmented) USB CDC drivers
@rem no USER blocks, alas
del ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc\usbd_cdc.h
copy usbd_cdc.h.MyCDCExt-V1.25.0 ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc\usbd_cdc.h

@rem this (finally) no longer seems necessary in 1.25.0
rem del ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c
rem copy usbd_cdc.c.MyCDCExt-V1.25.0 ..\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c

@rem most seems in user blocks in 1.25.0, but we want the 'presence' hack for the above
del ..\Src\usbd_cdc_if.c
copy usbd_cdc_if.c.MyCDCExt-V1.25.0 ..\Src\usbd_cdc_if.c
