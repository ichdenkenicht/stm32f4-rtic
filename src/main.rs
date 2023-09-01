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
        timer::{CounterHz, Delay, Event as TimerEvent, Timer, Channel, PwmHz, Channel4, ChannelBuilder},
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
        //s2: Serial<pac::USART2, (Pin<'A', 2, Alternate<7>>, Pin<'A', 3, Alternate<7>>)>,
        s2: Serial<pac::USART2>,
        tim5: CounterHz<pac::TIM5>,
        s: State,
        ws: f32,
        adc: Adc<pac::ADC1>,
    }

    #[local]
    struct Local {
        led: Pin<'C', 13, Output<PushPull>>,
        pin_a0: Pin<'A', 0, Input>, //Button


        pin_a1: Pin<'A', 1, Analog>, //Poti Sense
        pin_a6: Pin<'A', 6, Analog>, //Voltage Sense
        pin_a7: Pin<'A', 7, Analog>, //Current Sense

        pin_b0: Pin<'B', 0, Output<PushPull>>, //MOSFET

        tim3pwm: PwmHz<pac::TIM3, ChannelBuilder<pac::TIM3, 3>>,
        

        offset: f32,

        tim4: CounterHz<pac::TIM4>,
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

        //TIMER5
        // 32? bit Timer 5
        let mut tim5 = ctx.device.TIM5.counter_hz(&clocks);
        tim5.listen(TimerEvent::Update);
        

        //TIMER4
        // 16 bit Timer 4
        let mut tim4 = ctx.device.TIM4.counter_hz(&clocks);
        tim4.listen(TimerEvent::Update);


        //Timer3 for 25khz PWM


        //let mut pin_b1 = gpiob.pb1.into_alternate();

        let channel = Channel4::new(gpiob.pb1);

        let mut tim3pwm = ctx.device.TIM3.pwm_hz(channel, 25.kHz(), &clocks);


  
        
        tim3pwm.enable(Channel::C4);
        let pwmmax = tim3pwm.get_max_duty();
        tim3pwm.set_duty(Channel::C4, pwmmax);
        //tim3pwm.set_period(period)
        

        //ADC
        let pin_a6 = gpioa.pa6.into_analog(); //Voltage Sense
        let pin_a7 = gpioa.pa7.into_analog(); //Current Sense
        let pin_a1 = gpioa.pa1.into_analog(); //Poti Sense

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

        tim4.start(10.Hz()).ok();

        let s = State::Stopped;
        let ws: f32 = 0.0;

        writeln!(s2, "BatteryTester \n Press Button to Start").ok();
        

        //Button Interrupt
        let mut pin_a0 = gpioa.pa0.into_pull_up_input();
        pin_a0.make_interrupt_source(&mut sys_cfg);
        pin_a0.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        pin_a0.enable_interrupt(&mut ctx.device.EXTI);



        let mono = Systick::new(ctx.core.SYST, 100_000_000);
        
        (Shared {s2, tim5, s, ws, adc}, Local {led, pin_a0, pin_a1, pin_a6, pin_a7, pin_b0, offset, tim4, tim3pwm}, init::Monotonics(mono))
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    //Timer5 Interrupt
    #[task(binds = TIM5, local = [led, pin_a6, pin_a7, offset], shared = [s2, tim5, ws, adc])]
    fn on_tim5(mut ctx: on_tim5::Context) {
        ctx.shared.tim5.lock(|tim5|{
            tim5.clear_interrupt(TimerEvent::Update);
        });

        ctx.local.led.toggle();

        let mut samplei: u16 = 0;
        let mut samplev: u16 = 0;


        ctx.shared.adc.lock(|adc|{
            samplei = adc.convert(ctx.local.pin_a7, SampleTime::Cycles_480);
        });

        ctx.shared.adc.lock(|adc|{
            samplev = adc.convert(ctx.local.pin_a6, SampleTime::Cycles_480);
        });

        let v_acs: f32 = samplei as f32 * 3.3/4096.0;
        let v_acs_off = v_acs - *ctx.local.offset;
        let i_acs = v_acs_off * 10.0;

        //let samplev = ctx.local.adc.convert(ctx.local.pin_a6, SampleTime::Cycles_480);

        let mut v_vol: f32 = samplev as f32 * 3.3/4096.0;

        v_vol = v_vol * 375.0/75.0;

        let s = COUNTER_SEC.fetch_add(1, Ordering::SeqCst);

        let ws_now = v_vol * i_acs;

        let mut ws_until = 0.0;

        ctx.shared.ws.lock(|ws|{
            ws_until = *ws;
            *ws += ws_now;
        });


        ctx.shared.s2.lock(|s2|{
            writeln!(s2, "RUNTIME: {} s", s).ok();
            writeln!(s2, "I_ACS  : {} A", i_acs).ok();
            writeln!(s2, "V_VOL  : {} V", v_vol).ok();
            writeln!(s2, "WS     : {} Ws", ws_until).ok();
        });
    }

    #[task(binds = EXTI0, local = [pin_a0, pin_b0], shared = [s2, s, tim5, ws])]
    fn on_button(mut ctx: on_button::Context){
        ctx.local.pin_a0.clear_interrupt_pending_bit();
        
        ctx.shared.s.lock(|s|{
            if *s == State::Stopped{

                ctx.shared.s2.lock(|s2|{
                    writeln!(s2, "Starting").ok();
                });

                ctx.local.pin_b0.set_high();
                ctx.shared.tim5.lock(|tim5|{
                    tim5.start(1.Hz());
                });
                *s = State::Running;
            }
            else if *s == State::Running{

                ctx.shared.s2.lock(|s2|{
                    writeln!(s2, "Stopping").ok();
                });

                ctx.shared.ws.lock(|ws|{
                    *ws = 0.0;
                });

                COUNTER_SEC.swap(0, Ordering::SeqCst);

                ctx.local.pin_b0.set_low();
                ctx.shared.tim5.lock(|tim5|{
                    tim5.cancel();
                });
                *s = State::Stopped;
            }
        });

    }


    #[task(binds = TIM4, local = [tim4, pin_a1, tim3pwm], shared = [s2, adc])]
    fn on_tim4(mut ctx: on_tim4::Context){
        ctx.local.tim4.clear_interrupt(TimerEvent::Update);


        let mut poti: u16 = 0;
        ctx.shared.adc.lock(|adc|{
            poti = adc.convert(ctx.local.pin_a1, SampleTime::Cycles_144);
        });

        ctx.local.tim3pwm.set_duty(Channel::C4, poti);

        /*
        ctx.shared.s2.lock(|s2|{
            writeln!(s2, "POT: {}", poti).ok();

        });
        */
        

    }


    //meh no fifo needs to stream of single bytes
    #[task(binds = USART2, local = [], shared = [s2])]
    fn uart2recv(mut ctx: uart2recv::Context){

        ctx.shared.s2.lock(|s2|{
            //s2.unlisten(SerialEvent::Rxne);
            
            let mut buf: u8 = 0;

            while s2.is_rx_not_empty() {
                if let Ok(r) = s2.read(){
                    buf = r;
                }else{
                    break;
                }
            }
            

            writeln!(s2, "{:?}", buf).ok();
            

            //s2.listen(SerialEvent::Rxne);
            
        });

    }


    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }

}
