#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART6, SPI4, SPI5])]
mod app {
    use core::fmt::Write;
    use core::panic::PanicInfo;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use rtic::export::CriticalSection;

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



        //ADC
        //let pin_a6 = gpioa.pa6.into_analog(); //Voltage Sense
        //let pin_a7 = gpioa.pa7.into_analog(); //Current Sense

        //let mut adc = Adc::adc1(ctx.device.ADC1, true, AdcConfig::default());
        
        

        //let sample = adc.convert(&pin_a7, SampleTime::Cycles_480);
        
        //writeln!(s2, "ADC output: {}", sample).ok();


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
        loop {}
    }

}
