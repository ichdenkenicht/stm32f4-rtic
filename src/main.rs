#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]
#![feature(type_alias_impl_trait)]

use rtic::app;



mod gp8403;
use max31865_rs;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI4, SPI5, SPI3, I2C2_ER, I2C2_EV])]
mod app {
    use core::fmt::{Write, write};
    use core::panic::PanicInfo;
    use core::sync::atomic::{AtomicU16, AtomicUsize, Ordering};
    use core::usize;
    
    use rtic_monotonics::systick::Systick;
    use rtic_sync::{channel::*, make_channel};
    use cortex_m;
    
    use rtic::export::CriticalSection;

    use stm32f4xx_hal::gpio::Analog;
    use stm32f4xx_hal::timer::PwmChannel;
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
    //use systick_monotonic::*;

    use heapless::String;

    use embedded_hal_bus::spi::ExclusiveDevice;

    use pid::Pid;

    use crate::gp8403::{GP8403Driver, Addr, OutputRange, Channel as gpchannel};

    //gp8403;

    use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};

    const CAPACITY: usize = 5;
    

    //use shared_bus;

    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    //static ERROR_CNT: AtomicUsize = AtomicUsize::new(0);
    static PT1000A: AtomicUsize = AtomicUsize::new(0);  //Atomic for PT1000
    static POTA: AtomicUsize = AtomicUsize::new(0);     //Atomic for Potentiometer
    static VALVEA: AtomicUsize = AtomicUsize::new(0);     //Atomic for Valve Return
    static OUTP: AtomicU16 = AtomicU16::new(2048);

    #[shared]
    struct Shared {
        s2: Serial<pac::USART2>,
    }

    #[local]
    struct Local {
        led: Pin<'C', 13, Output<PushPull>>,
        pin_a0: Pin<'A', 0, Input>, //Button

        pin_a5: Pin<'A', 5, Output<PushPull>>, //Display Backlight

        // pin_a4: Pin<'A', 4, Input>, //10V return from valve

        tim3: CounterHz<pac::TIM3>,
        tim5: Counter<pac::TIM5, 1000000>,

        adc: Adc<pac::ADC1>,
        pin_b0: Pin<'B', 0, Analog>, //Pot
        //pin_a4: Pin<'A', 4, Analog>, //Valve
        //pin_a7: Pin<'A', 7, Analog>, //PT1000

        gp: GP8403Driver<I2c<pac::I2C1>>,

        pid1: Pid<f32>,

        lcd: HD44780<hd44780_driver::bus::I2CBus<I2c<pac::I2C3>>>,
        del: Delay<pac::TIM10, 1000000>,

        sender1: Sender<'static, (f32, f32), CAPACITY>,
        sender2: Sender<'static, String::<16>, CAPACITY>,

    }

    //#[monotonic(binds = SysTick, default = true)]
    //type Tonic = Systick<1000>;

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {

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

        let mut led = gpioc.pc13.into_push_pull_output();
        led.set_high();

        

        //UART1
        //PA10 -> RX1
        //PA9 -> TX1
        /*
        let pin_a9 = gpioa.pa9.into_alternate();
        let pin_a10 = gpioa.pa10.into_alternate();

        let mut s1 = Serial::new(
            ctx.device.USART1,
            (pin_a9, pin_a10),
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();
        writeln!(s1, "Init... s1").ok();
        */

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
        writeln!(s2, "Init... s2").ok();
        

        //Timers

        let mono_token = rtic_monotonics::create_systick_token!();
        let mono = Systick::start(ctx.core.SYST, 100_000_000, mono_token);

        //TIMER3
        // 16 bit Timer 3
        //interrupt capable
        let mut tim3 = ctx.device.TIM3.counter_hz(&clocks);
        tim3.listen_only(TimerEvent::Update);
        
        // Timer9
        // 16bit only up
        //let mut tim9 = ctx.device.TIM9.counter_ms(&clocks);
        //tim9.start(10u32.secs()).unwrap();


        //TIMER4
        // 16 bit Timer 4
        //interrupt capable
        let mut tim5 = ctx.device.TIM5.counter_us(&clocks);
        tim5.listen_only(TimerEvent::Update);

        writeln!(s2, "timers ok").ok();

        //TIMER4
        // 16 bit Timer 4
        //let mut tim4 = ctx.device.TIM4.counter_hz(&clocks);
        //tim4.listen(Event::Update);

        let mut del = ctx.device.TIM10.delay_us(&clocks);
        
        let del2 = ctx.device.TIM11.delay_us(&clocks);
        writeln!(s2, "delay ok").ok();

        

        //I2C1 gp8403

        let pin_b6 = gpiob.pb6; //SCL1
        let pin_b7 = gpiob.pb7; //SDA1

        let i2c1 = ctx.device.I2C1.i2c(
            (pin_b6, pin_b7),
            i2cMode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        );
        writeln!(s2, "i2c1 ok").ok();

        //I2C3  display

        let pin_a8 = gpioa.pa8; //SCL3
        let pin_b4 = gpiob.pb4.into_push_pull_output(); //SDA3

        let i2c3 = ctx.device.I2C3.i2c(
            (pin_a8, pin_b4),
            i2cMode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        );
        writeln!(s2, "i2c3 ok").ok();

        let mut gp = GP8403Driver::new(i2c1, Addr::A58);

        gp.setOutputRange(OutputRange::V10).unwrap_or_else(|c|{writeln!(s2, "{:?}", c).ok(); loop{}});
        gp.setOutput(gpchannel::Channel0, 0x7FF).unwrap_or_else(|c|{writeln!(s2, "{:?}", c).ok(); loop{}});


        



        //Display
        
        let mut pin_a5 = gpioa.pa5.into_push_pull_output();
        pin_a5.set_high();

        const I2C_ADDRESS: u8 = 0x27;
        let mut lcd = HD44780::new_i2c(i2c3, I2C_ADDRESS, &mut del).unwrap();

        
        lcd.reset(&mut del).unwrap_or_else(|c|{writeln!(s2, "{:?}", c).ok(); loop{}});
        lcd.clear(&mut del).unwrap_or_else(|c|{writeln!(s2, "{:?}", c).ok(); loop{}});

        lcd.set_display_mode(
            DisplayMode {
                display: Display::On,
                cursor_visibility: Cursor::Visible,
                cursor_blink: CursorBlink::On,
            },
            &mut del
        ).unwrap_or_else(|c|{writeln!(s2, "{:?}", c).ok(); loop{}});
        


        //let _ = lcd.write_str("Hello, world!", &mut del);
        //lcd.set_cursor_xy((0,1), &mut del);
        //let _ = lcd.write_str("test123!", &mut del);


        //SPI2
        //CS
        let mut pin_b12 = gpiob.pb12.into_push_pull_output().speed(Speed::VeryHigh);
        pin_b12.set_high();

        let pin_b13 = gpiob.pb13.into_alternate().speed(Speed::VeryHigh); //SCK2
        let pin_b14 = gpiob.pb14.into_alternate().speed(Speed::VeryHigh); //MISO2
        let pin_b15 = gpiob.pb15.into_alternate().speed(Speed::VeryHigh); //MOSI2

        let mode = spiMode {
            polarity: spi::Polarity::IdleLow,
            phase: spi::Phase::CaptureOnFirstTransition,
        };

        let mut spi2 = ctx
            .device
            .SPI2
            .spi((pin_b13, pin_b14, pin_b15), mode, 10.MHz(), &clocks);

        let ex_spi2 = ExclusiveDevice::new(spi2, pin_b12, del2).unwrap();
        
        let mut mx31 = max31865_rs::MAX31865::new(ex_spi2, 430);
        
        mx31.set_filter(max31865_rs::FILTER::HZ50);
        mx31.set_wire(max31865_rs::WIRE3::WIRE_3);
        mx31.set_vbias(max31865_rs::VBIAS::OFF);
        mx31.set_mode(max31865_rs::MODE::AUTO);

        writeln!(s2, "{:#?}", mx31.measure_temp());

        
        //PIDController
        let mut pid1: Pid<f32> = Pid::new(55.0, 50.0);   //maybe -50.0 - 50.0 -> +50 und /10 to get to 0 - 10 V?
        pid1.p(0.80, 20.0);
        pid1.i(0.25, 2.0);
        pid1.d(0.03, 3.0);



        //ADC
        let pin_b0 = gpiob.pb0.into_analog(); //Potentiometer Sense
        let pin_a4 = gpioa.pa4.into_analog(); //return from Valve Sense
        //let pin_a7 = gpioa.pa7.into_analog(); //PT1000

        let adc = Adc::adc1(ctx.device.ADC1, true, AdcConfig::default());
        writeln!(s2, "ADC ok").ok();

        
        //Internal RTC
        //let mut irtc = Rtc::new(hal_p.RTC, &mut hal_p.PWR);
        //let d
        //irtc.set_datetime(date);
        //let time = irtc.get_datetime();
        //writeln!(s2, "{:#?}", time).unwrap();

        
        let (cs1, cr1) = make_channel!((f32, f32), CAPACITY);
        let (mut cs2, cr2) = make_channel!(String::<16>, CAPACITY);


        

        //Button Interrupt
        let mut pin_a0 = gpioa.pa0.into_pull_up_input();
        pin_a0.make_interrupt_source(&mut sys_cfg);
        pin_a0.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        pin_a0.enable_interrupt(&mut ctx.device.EXTI);



        lcd::spawn(cr1, cr2).unwrap();
        //tim3.start(1_000u32.millis()).ok();
        //tim3.start(1u32.secs()).ok();
        //tim3.start(1.Hz()).ok();
        
        tim5.start(100.millis()).ok();
        

        //let mut s_ok = String::<16>::new();
        //write!(s_ok, "Ok").ok();
        //cs2.try_send(s_ok).ok();
        
        (Shared {s2}, Local {led, tim3, tim5, pin_a0, pid1, pin_a5, adc, pin_b0, lcd, del, gp, sender1: cs1.clone(), sender2: cs2.clone()})
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    //Timer5 Interrupt
    #[task(binds = TIM5, local = [tim5, adc, pin_b0, led], shared = [s2], priority = 7)]
    fn on_tim5(mut ctx: on_tim5::Context) {
        ctx.local.tim5.clear_flags(Flag::Update);

        ctx.local.led.toggle();

        let cnt = COUNTER.fetch_add(1, Ordering::SeqCst);
        


        const OVERSAMPLING: usize = 4;

        //Pot - bin_b0
        let mut pot = 0usize;
        for _ in 0..OVERSAMPLING{
            pot += ctx.local.adc.convert(ctx.local.pin_b0, SampleTime::Cycles_480) as usize;
        }
        pot = pot / 4;

        POTA.fetch_add(pot, Ordering::SeqCst);


        //PT1000 - pin_a7
        //let mut pt1000 = 0usize;
        //for i in 0..OVERSAMPLING{
        //    pt1000 += ctx.local.adc.convert(ctx.local.pin_a7, SampleTime::Cycles_480) as usize;
        //}
        //pt1000 = pt1000 / 4;

        //PT1000A.fetch_add(pt1000, Ordering::SeqCst);


        if cnt == 19 {
            COUNTER.swap(0, Ordering::SeqCst);
            let pt1 = PT1000A.swap(0, Ordering::SeqCst) / 20;
            let pot = POTA.swap(0, Ordering::SeqCst) / 20;

            if pt1 > 4000 {
                //send error and return
            }

            let nextsetpoint = (((pot as f32) - 201.0) * (75.0 - 45.0) / (3900.0 - 201.0)) + 45.0;
            let mut pt1000temp =  ((pt1 as f32 * 0.3029) - 625.70);
            pt1000temp = pt1000temp.clamp(5.0, 100.0);


            ctx.shared.s2.lock(|s2|{
                writeln!(s2, "pt1000 raw:{} pot raw: {}", pt1, pot).ok();
                writeln!(s2, "S: {} I: {}", nextsetpoint, pt1000temp).ok();
            });

            pidf::spawn(nextsetpoint, pt1000temp).ok();
        }

    }


    #[task(local = [pid1, gp, sender1, sender2, out: i16 = 3856], shared = [s2], priority = 3)]
    async fn pidf(mut ctx: pidf::Context, setpoint: f32, istpoint: f32) {
        
        match setpoint {
            ..=45.0 => {
                ctx.local.gp.setOutput(gpchannel::Channel0, 0xFFF).ok(); // 0 oder 10V
                // 0xFFF bedeutet Kreis geschlossen
            },
            45.0..=75.0 => {
                
                ctx.local.sender1.try_send((setpoint, istpoint)).ok();


                let nco = ctx.local.pid1.next_control_output(istpoint);
                ctx.local.pid1.setpoint(setpoint);

                //let out = ((nco.output*40.96)+2048.0).clamp(0.0, 4095.0) as u16;
                //let out = ((-nco.output*130.146)+2048.0).clamp(0.0, 4095.0) as u16;

                *ctx.local.out -= nco.output as i16;
                *ctx.local.out = *ctx.local.out.clamp(&mut 0i16, &mut 4095i16);

            
                ctx.local.gp.setOutput(gpchannel::Channel0, *ctx.local.out as u16).ok();

                ctx.shared.s2.lock(|s2|{
                    writeln!(s2, "out: {}, nco: {:?}", *ctx.local.out, nco).ok();
                });

            },
            75.0.. => {
                ctx.local.gp.setOutput(gpchannel::Channel0, 0x000).ok(); // 0 oder 10V
                // 0x000 bedeutet Kreis offen
            },
            _ => {

            }
        }



    }


    #[task(local = [lcd, del], priority = 2)]
    async fn lcd(ctx: lcd::Context, mut receiver1: Receiver<'static, (f32, f32), CAPACITY>, mut receiver2: Receiver<'static, String::<16>, CAPACITY>) {
        
        while let Ok(val) = receiver1.recv().await {
            let mut line0 = String::<16>::new();
            write!(line0, "S:{:.1} I:{:.1}", val.0, val.1);
            ctx.local.lcd.set_cursor_xy((0,0), ctx.local.del);
            ctx.local.lcd.write_str(&line0 ,ctx.local.del);

            if let Ok(val) = receiver2.try_recv() {
                ctx.local.lcd.set_cursor_xy((0,1), ctx.local.del);
                ctx.local.lcd.write_str(&val ,ctx.local.del);
            };

        }
    }

    /*
    //Timer3 Interrupt
    #[task(binds = TIM3, local = [tim3, led, pid1, adc, pin_b0, pin_a7, gp, sender1, sender2], shared = [s2], priority = 6)]
    fn on_tim3(mut ctx: on_tim3::Context) {
        ctx.local.tim3.clear_flags(Flag::Update);
        

        ctx.local.led.toggle();

        //ctx.local.pin_a5.toggle();

        let pot = ctx.local.adc.convert(ctx.local.pin_b0, SampleTime::Cycles_480);
        //let valve = ctx.local.adc.convert(ctx.local.pin_a4, SampleTime::Cycles_56);

        const OVERSAMPLING: usize = 60;

        let mut pt1000 = [0u16; OVERSAMPLING];
        for i in 0..OVERSAMPLING{
            pt1000[i] = ctx.local.adc.convert(ctx.local.pin_a7, SampleTime::Cycles_480);
        }

        let mut sum = 0u32;
        for u in 0..OVERSAMPLING {
            sum += pt1000[u] as u32;
        }
        let pt1000f = sum as f32 / OVERSAMPLING as f32;

        if pt1000f > 4000.0 {
            let mut s = String::<16>::new();
            write!(s, "Error PT1000");
            ctx.local.sender2.try_send(s);
            return;
        }

        
        match pot {
            ..=0xC8 => {    //Unter 100 sperr ventil
                ctx.local.gp.setOutput(gpchannel::Channel0, 0x000); // 0 oder 10V
                let mut s = String::<16>::new();
                write!(s, "Ventil gesperrt");
                ctx.local.sender2.try_send(s);

            },
            0xC9..=0xF3C => { //map to 45 - 75 Grad C
                let nextsetpoint = (((pot as f32) - 201.0) * (75.0 - 45.0) / (3900.0 - 201.0)) + 45.0;
                let mut pt1000temp =  ((pt1000f * 0.3029) - 625.70);
                pt1000temp = pt1000temp.clamp(5.0, 100.0);
                

                ctx.local.sender1.try_send((nextsetpoint, pt1000temp)).ok();
                
                let nco = ctx.local.pid1.next_control_output(pt1000temp);
                ctx.local.pid1.setpoint(nextsetpoint);


                //let out = ((nco.output*40.96)+2048.0).clamp(0.0, 4095.0) as u16;
                let out = ((nco.output*73.146)+2048.0).clamp(0.0, 4095.0) as u16;

                ctx.local.gp.setOutput(gpchannel::Channel0, out).ok();

                ctx.shared.s2.lock(|s2|{
                    writeln!(s2, "raw: S: {} I: {:.4}", pot, pt1000f).ok();
                    writeln!(s2, "S: {} I: {}", nextsetpoint, pt1000temp).ok();
                    writeln!(s2, "{:#?}", nco).ok();
                    writeln!(s2, "0-10V: {:X}", out).ok();
                });

            },
            0xF3D..=u16::MAX => { //oeffne vollstaendig
                ctx.local.gp.setOutput(gpchannel::Channel0, 0xFFF); //10 oder 0V >D
                let mut s = String::<16>::new();
                write!(s, "Ventil offen");
                ctx.local.sender2.try_send(s);
            }
        }
        

        //let val = COUNTER_INT.fetch_add(100, Ordering::SeqCst);
        //let val: usize = COUNTER_INT.swap(0, Ordering::SeqCst);

        //let errors = ERROR_CNT.fetch_add(0, Ordering::SeqCst);

        //ctx.shared.s2.lock(|s2|{
            //writeln!(s2, "Error CNT: {}", errors).ok();
        //});

    }
    */

    #[task(binds = EXTI0, local = [pin_a0], shared = [s2])]
    fn on_button(mut ctx: on_button::Context){
        ctx.local.pin_a0.clear_interrupt_pending_bit();

        ctx.shared.s2.lock(|s2|{
            writeln!(s2, "button interrupt").ok();
        });

    }


    /*
    #[task(local = [t1ch0, t1ch1, t1ch2, t1ch3, pid1, pid2], shared = [s2])]
    fn level(mut ctx: level::Context, a0: f32, a1: f32, a2: f32 ){
        //ctx.local.t1ch0.set_duty(((a0 * 100.0) as i16).abs() as u16 * 8);
        //ctx.local.t1ch1.set_duty(((a1 * 100.0) as i16).abs() as u16 * 8);

        let p1 = ctx.local.pid1.next_control_output(a0);
        let p2 = ctx.local.pid2.next_control_output(a1);

        
        ctx.shared.s2.lock(|s2|{
            //writeln!(s2, "p1: {:?}", p1).ok();
            writeln!(s2, "/*{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}*/", a0, a1, a2, p1.p, p1.i, p1.d, p1.output, p2.p, p2.i, p2.d, p2.output).ok();
        });

        //ctx.local.t1ch0.set_duty((p1.output as u16) * 1000);
        //ctx.local.t1ch1.set_duty((p2.output as u16) * 1000);


    }
    */

    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        //cortex_m::asm::bootload(vector_table)
        loop {}
    }

}
