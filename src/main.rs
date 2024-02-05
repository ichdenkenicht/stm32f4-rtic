#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;

mod gp8403;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI4, SPI5])]
mod app {
    use core::fmt::{Write, write};
    use core::panic::PanicInfo;
    use core::sync::atomic::{AtomicU16, AtomicUsize, Ordering};
    use core::usize;
    
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
        timer::{CounterHz, Delay, Event as TimerEvent, Timer, Pwm, Channel, Channel1, Channel2, Channel3, Channel4, Flag},
        adc::{config::AdcConfig, config::SampleTime, Adc, config::*},
    };
    use systick_monotonic::*;

    use pid::Pid;

    use crate::gp8403::{GP8403Driver, Addr, OutputRange, Channel as gpchannel};

    //gp8403;

    use hd44780_driver::{Cursor, CursorBlink, Display, DisplayMode, HD44780};
    

    //use shared_bus;

    static COUNTER_INT: AtomicU16 = AtomicU16::new(0);

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

        //gp: GP8403Driver<I2c<pac::I2C3>>,

        pid1: Pid<f32>,

    }

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<1000>;

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
        // 16 bit Timer 3
        let mut tim3 = ctx.device.TIM3.counter_hz(&clocks);
        tim3.listen(TimerEvent::Update);
        

        //TIMER4
        // 16 bit Timer 4
        //let mut tim4 = ctx.device.TIM4.counter_hz(&clocks);
        //tim4.listen(Event::Update);

        let mut del = ctx.device.TIM4.delay_us(&clocks);
        writeln!(s2, "timer ok").ok();

        

        //I2C1

        let pin_b6 = gpiob.pb6; //SCL1
        let pin_b7 = gpiob.pb7; //SDA1

        let i2c1 = ctx.device.I2C1.i2c(
            (pin_b6, pin_b7),
            i2cMode::Standard {
                frequency: 400.kHz(),
            },
            &clocks,
        );
        writeln!(s2, "i2c1 ok").ok();

        //I2C3

        let pin_a8 = gpioa.pa8; //SCL3
        let pin_b4 = gpiob.pb4.into_push_pull_output(); //SDA3

        let i2c3 = ctx.device.I2C3.i2c(
            (pin_a8, pin_b4),
            i2cMode::Standard {
                frequency: 40.kHz(),
            },
            &clocks,
        );
        writeln!(s2, "i2c3 ok").ok();

        //let mut gp = GP8403Driver::new(i2c3, Addr::A58);

        //gp.setOutputRange(OutputRange::V10).ok();
        //gp.setOutput(gpchannel::Channel0, 0xFFF).ok();

        /*
        const I2C_ADDRESS: u8 = 0x27;
        let mut lcd = HD44780::new_i2c(i2c3, I2C_ADDRESS, &mut del).unwrap();

        lcd.reset(&mut del);
        lcd.clear(&mut del);

        
        
        lcd.set_display_mode(
            DisplayMode {
                display: Display::On,
                cursor_visibility: Cursor::Visible,
                cursor_blink: CursorBlink::On,
            },
            &mut del
        );
        let _ = lcd.write_str("Hello, world!", &mut del);
        lcd.set_cursor_xy((0,1), &mut del);
        let _ = lcd.write_str("test123!", &mut del);
        */

        
        
        //TestPIDController
        let mut pid1: Pid<f32> = Pid::new(55.0, 50.0);   //maybe -50.0 - 50.0 -> +50 und /10 to get to 0 - 10 V?
        pid1.p(0.09, 5.0);
        pid1.i(0.04, 5.0);
        pid1.d(0.01, 3.0);



        //ADC
        let pin_b0 = gpiob.pb0.into_analog(); //Potentiometer Sense
        let pin_b1 = gpiob.pb1.into_analog(); //return from Valve Sense

        let mut adc = Adc::adc1(ctx.device.ADC1, true, AdcConfig::default());
        //let sample = adc.convert(&pin_b0, SampleTime::Cycles_144);
        //let sample = adc.convert(&pin_b1, SampleTime::Cycles_28); 
        
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
        
        (Shared {s2}, Local {led, tim3, pin_a0, pid1}, init::Monotonics(mono))
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    //Timer3 Interrupt
    #[task(binds = TIM3, local = [tim3, led, pid1], shared = [s2], priority = 6)]
    fn on_tim3(mut ctx: on_tim3::Context) {
        ctx.local.tim3.clear_flags(Flag::Update);

        
        ctx.local.led.toggle();

        //ctx.local.pid1.next_control_output();

        let val = COUNTER_INT.fetch_add(100, Ordering::SeqCst);

        //let val: usize = COUNTER_INT.swap(0, Ordering::SeqCst);

        //ctx.local.gp.setOutput(gpchannel::Channel0, val);


        ctx.shared.s2.lock(|s2|{
            //writeln!(s2, "last second: {}", val).ok();
        });

    }

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
        
        
        loop {}
    }

}
