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
        serial::{Config, Serial, Event as SerialEvent},
        spi::{self, Mode as spiMode, Spi},
        timer::{CounterHz, Delay, Event as TimerEvent, Timer},
        adc::{config::AdcConfig, config::SampleTime, Adc, config::*},
    };
    use systick_monotonic::*;

    use shared_bus;
    //ADS1115
    use ads1x1x::{
        channel, ic as ads1x1xIc, mode, Ads1x1x, ComparatorPolarity, ComparatorQueue,
        DataRate16Bit, FullScaleRange, SlaveAddr as adsSlaveAddr,
    };

    #[derive(PartialEq)]
    pub enum State{
        Stopped,
        Running,
    }

    static COUNTER_SEC: AtomicUsize = AtomicUsize::new(0);  //N of Sekunden
    //static COUNTER_WS: AtomicUsize = AtomicUsize::new(0);   //Wattsekunden

    #[shared]
    struct Shared {
        s2: Serial<pac::USART2, (Pin<'A', 2, Alternate<7>>, Pin<'A', 3, Alternate<7>>)>,
        tim3: CounterHz<pac::TIM3>,
        s: State,
    }

    #[local]
    struct Local {
        led: Pin<'C', 13, Output<PushPull>>,
        pin_a0: Pin<'A', 0, Input>, //Button

        pin_a6: Pin<'A', 6, Analog>, //Current Sense
        pin_a7: Pin<'A', 7, Analog>, //Current Sense

        pin_b0: Pin<'B', 0, Output<PushPull>>, //MOSFET

        //tim3: CounterHz<pac::TIM3>,
        adc: Adc<pac::ADC1>,

        offset: f32,
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

        //s2.listen(SerialEvent::Rxne);

        //TIMER3
        // 16 bit Timer 3u
        let mut tim3 = ctx.device.TIM3.counter_hz(&clocks);
        tim3.listen(TimerEvent::Update);
        

        //TIMER4
        // 16 bit Timer 4
        let mut tim4 = ctx.device.TIM4.counter_hz(&clocks);
        tim4.listen(TimerEvent::Update);
        

        //ADC
        let pin_a6 = gpioa.pa6.into_analog(); //Voltage Sense
        let pin_a7 = gpioa.pa7.into_analog(); //Current Sense

        let mut adc = Adc::adc1(ctx.device.ADC1, true, AdcConfig::default());
        
        
        //MOSFET
        let mut pin_b0 = gpiob.pb0.into_push_pull_output();
        pin_b0.set_low();

        let mut s_a = [0u16; 8];

        for v in 0..8 {
            s_a[v] = adc.convert(&pin_a7, SampleTime::Cycles_480);
        }


        let mut sum: f32 = 0.0;
        for l in 0..8 {
            sum += s_a[l] as f32 * 3.3/4096.0;
        }

        let offset = sum/8.0;
        //writeln!(s2, "ADC output: {}", sample).ok();


        //Internal RTC
        //let mut irtc = Rtc::new(hal_p.RTC, &mut hal_p.PWR);
        //let d
        //irtc.set_datetime(date);
        //let time = irtc.get_datetime();
        //writeln!(s2, "{:#?}", time).unwrap();

        //tim3.start(1.Hz()).ok();

        let s = State::Stopped;

        writeln!(s2, "BatteryTester \n Press Button to Start").ok();
        

        //Button Interrupt
        let mut pin_a0 = gpioa.pa0.into_pull_up_input();
        pin_a0.make_interrupt_source(&mut sys_cfg);
        pin_a0.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        pin_a0.enable_interrupt(&mut ctx.device.EXTI);



        let mono = Systick::new(ctx.core.SYST, 100_000_000);
        
        (Shared {s2, tim3, s}, Local {led, pin_a0, pin_a6, pin_a7, pin_b0, adc, offset}, init::Monotonics(mono))
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    //Timer3 Interrupt
    #[task(binds = TIM3, local = [led, pin_a6, pin_a7, adc, offset], shared = [s2, tim3], priority = 6)]
    fn on_tim3(mut ctx: on_tim3::Context) {
        ctx.shared.tim3.lock(|tim3|{
            tim3.clear_interrupt(TimerEvent::Update);
        });

        //ctx.local.tim3.clear_interrupt(TimerEvent::Update);


        ctx.local.led.toggle();

        let samplei = ctx.local.adc.convert(ctx.local.pin_a7, SampleTime::Cycles_480);

        let v_acs: f32 = samplei as f32 * 3.3/4096.0;
        let v_acs_off = v_acs - *ctx.local.offset;
        let i_acs = v_acs_off * 10.0;

        let samplev = ctx.local.adc.convert(ctx.local.pin_a6, SampleTime::Cycles_480);

        let mut v_vol: f32 = samplev as f32 * 3.3/4096.0;

        v_vol = v_vol * 375.0/75.0;

        let s = COUNTER_SEC.fetch_add(1, Ordering::SeqCst);

        let ws = v_vol * i_acs;


        ctx.shared.s2.lock(|s2|{
            writeln!(s2, "RUNTIME: {} s", s).ok();
            writeln!(s2, "I_ACS  : {} A", i_acs).ok();
            writeln!(s2, "V_VOL  : {} V", v_vol).ok();
            writeln!(s2, "WS     : {} Ws", ws).ok();
        });
    }

    #[task(binds = EXTI0, local = [pin_a0, pin_b0], shared = [s2, s, tim3])]
    fn on_button(mut ctx: on_button::Context){
        ctx.local.pin_a0.clear_interrupt_pending_bit();
        
        ctx.shared.s.lock(|s|{
            if *s == State::Stopped{

                ctx.shared.s2.lock(|s2|{
                    writeln!(s2, "Starting").ok();
                });

                ctx.local.pin_b0.set_high();
                ctx.shared.tim3.lock(|tim3|{
                    tim3.start(1.Hz());
                });
                *s = State::Running;
            }
            else if *s == State::Running{

                ctx.shared.s2.lock(|s2|{
                    writeln!(s2, "Stopping").ok();
                });

                COUNTER_SEC.swap(0, Ordering::SeqCst);

                ctx.local.pin_b0.set_low();
                ctx.shared.tim3.lock(|tim3|{
                    tim3.cancel();
                });
                *s = State::Stopped;
            }
        });

    }





    //unlistend to interupt should never execute
    #[task(binds = USART2, local = [], shared = [s2])]
    fn uart2recv(mut ctx: uart2recv::Context){

        ctx.shared.s2.lock(|s2|{
            
            let mut buf = [0u8; 64];

            let mut i = 0;

            while s2.is_rx_not_empty() {
                if let Ok(r) = s2.read(){
                    buf[i] = r;
                    i += 1;
                }else{
                    break;
                }
            }

            writeln!(s2, "{}:{:?}", i, buf).ok();
            
        });

    }


    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }

}
