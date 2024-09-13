#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;

pub mod smoltcp_phy;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART6, SPI4, SPI5])]
mod app {
    use core::fmt::Write;
    use core::panic::PanicInfo;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use rtic::export::CriticalSection;

    use smoltcp::socket;
    use stm32f4xx_hal::gpio::Analog;
    use stm32f4xx_hal::{
        gpio::*,
        i2c::{self, I2c, Mode as i2cMode},
        pac,
        pac::NVIC,
        pac::{interrupt, Interrupt},
        prelude::*,
        rtc::{self, Rtc},
        serial::{Config, Serial},
        spi::{self, Mode as spiMode, Spi},
        timer::{CounterHz, Delay, Event as TimerEvent, Timer, Pwm, Channel, Channel1, Channel2, Channel3, Channel4, Flag, Counter},
        adc::{config::AdcConfig, config::SampleTime, Adc, config::*},
    };
    use systick_monotonic::*;

    use enc28j60::{Enc28j60};

    use smoltcp::{
        iface::{Config as smolConfig, Interface, SocketSet},
        wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv6Address},
        time::Instant,
        socket::udp,
        socket::udp::{Socket},
        storage::{PacketBuffer, PacketMetadata, RingBuffer},
    };

    use heapless::{Vec};

    use crate::smoltcp_phy;

    

    //use shared_bus;


    #[shared]
    struct Shared {
        s2: Serial<pac::USART2>,
    }

    #[local]
    struct Local {
        led: Pin<'C', 13, Output<PushPull>>,
        pin_a0: Pin<'A', 0, Input>, //Button

        tim3: CounterHz<pac::TIM3>,

        //adc: Adc<pac::ADC1>,
        //pin_a6: Pin<'A', 6, Analog>, //Voltage Sense
        //pin_a7: Pin<'A', 7, Analog>, //Current Sense
    }

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<10000>;

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {

        let mut sys_cfg = ctx.device.SYSCFG.constrain();
        //RCC
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(25.MHz())
            .sysclk(100.MHz())
            .pclk1(50.MHz()) //max 50 MHz
            .pclk2(50.MHz()) //max 100 MHz
            .freeze();

        //GPIO SPLITS
        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();

        let led = gpioc.pc13.into_push_pull_output();

        //UART2
        //PA3 -> RX2
        //PA2 -> TX2

        let pin_a2 = gpioa.pa2.into_alternate();
        let pin_a3 = gpioa.pa3.into_alternate();

        let mut s2 = Serial::new(
            ctx.device.USART2,
            (pin_a2, pin_a3),
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();
        writeln!(s2, "Init...").ok();



        //TIMER3
        // 16 bit Timer 3u
        let mut tim3 = ctx.device.TIM3.counter_hz(&clocks);
        tim3.listen_only(TimerEvent::Update);
        

        //TIMER4
        // 16 bit Timer 4
        let mut tim4 = ctx.device.TIM4.counter_hz(&clocks);
        tim4.listen_only(TimerEvent::Update);

        //Timer10
        //as delay
        let mut del = ctx.device.TIM10.delay_us(&clocks);



        //SPI1
        let mut pin_a4 = gpioa.pa4.into_push_pull_output(); //CS
        pin_a4.set_high();

        let pin_a5 = gpioa.pa5.into_alternate(); //SCK1
        let pin_a6 = gpioa.pa6.into_alternate(); //MISO1
        let pin_a7 = gpioa.pa7.into_alternate(); //MOSI1

        //temporary for reset
        let pin_a8 = gpioa.pa8.into_push_pull_output();

        let mode = spiMode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        };

        let spi1 = ctx.device.SPI1.spi((pin_a5, pin_a6, pin_a7), mode, 1.MHz(), &clocks);

        
        let ex_spi1 = ExclusiveDevice::new_no_delay(spi1, pin_a4).unwrap();

        const MAC: [u8;6] = [0x20, 0x18, 0x03, 0x01, 0x00, 0x0A];
        const KB: u16 = 1024; // bytes
        let enc = Enc28j60::new(ex_spi1, enc28j60::Unconnected, pin_a8, &mut del, 7 * KB, MAC).unwrap();

        let mut ephy = smoltcp_phy::EncPhy::new(enc);

        let config = smolConfig::new(HardwareAddress::Ethernet(EthernetAddress(MAC)));
        let local_ip_addr = Ipv6Address::new(0xfe80, 0x0, 0x0, 0x0, 0xaaaa, 0x0, 0x0, 0x1);
        let ip_addr = IpCidr::new(IpAddress::from(local_ip_addr), 64);

        let mut iface = Interface::new(config, &mut ephy, Instant::from_micros(0));

        iface.update_ip_addrs(|a|{
            a.push(ip_addr);
        });

        let mut rx_s = Vec::<u8, 2048>::new();
        let mut rx_m = Vec::<udp::PacketMetadata, 8>::new();
        let udp_rx_buffer = PacketBuffer::new(
            rx_m.as_mut_slice(),
            rx_s.as_mut_slice(),
        );

        let mut tx_s = Vec::<u8, 2048>::new();
        let mut tx_m = Vec::<udp::PacketMetadata, 8>::new();
        let udp_tx_buffer = PacketBuffer::new(
            tx_m.as_mut_slice(),
            tx_s.as_mut_slice(),
        );

        let socketudp = Socket::new(udp_rx_buffer, udp_tx_buffer);

        socketudp.send_slice(data, meta);

        //ADC
        //let pin_a6 = gpioa.pa6.into_analog(); //Voltage Sense
        //let pin_a7 = gpioa.pa7.into_analog(); //Current Sense

        //let mut adc = Adc::adc1(ctx.device.ADC1, true, AdcConfig::default());
        
        

        //let sample = adc.convert(&pin_a7, SampleTime::Cycles_480);
        
        //writeln!(s2, "ADC output: {}", sample).ok();
        // 

        //Internal RTC
        //let mut irtc = Rtc::new(hal_p.RTC, &mut hal_p.PWR);
        //let d
        //irtc.set_datetime(date);
        //let time = irtc.get_datetime();
        //writeln!(s2, "{:#?}", time).unwrap();

        tim3.start(1.Hz()).ok();


        //Button Interrupt
        let mut pin_a0 = gpioa.pa0.into_pull_up_input();
        pin_a0.make_interrupt_source(&mut sys_cfg);
        pin_a0.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        pin_a0.enable_interrupt(&mut ctx.device.EXTI);



        let mono = Systick::new(ctx.core.SYST, 100_000_000);
        
        (Shared {s2}, Local {led, tim3, pin_a0}, init::Monotonics(mono))
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    //Timer3 Interrupt
    #[task(binds = TIM3, local = [tim3, led], shared = [s2], priority = 6)]
    fn on_tim3(mut ctx: on_tim3::Context) {
        ctx.local.tim3.clear_flags(Flag::Update);
        ctx.local.led.toggle();

    }

    #[task(binds = EXTI0, local = [pin_a0], shared = [s2])]
    fn on_button(mut ctx: on_button::Context){
        ctx.local.pin_a0.clear_interrupt_pending_bit();

        ctx.shared.s2.lock(|s2|{
            writeln!(s2, "button interrupt").ok();
        });

    }


    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {
            //maybe send panic info onto uart or wherever you want
        }
    }

}
