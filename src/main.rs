#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(unused_imports)]

use rtic::app;

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI4, SPI5])]
mod app {
    use core::fmt::{Write, write};
    use core::panic::PanicInfo;
    use core::sync::atomic::{AtomicUsize, Ordering};
    use core::usize;
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
        timer::{CounterHz, Delay, Event, Timer, Pwm, Channel, Channel1, Channel2, Channel3, Channel4},
        adc::{config::AdcConfig, config::SampleTime, Adc, config::*},
    };
    use systick_monotonic::*;

    use mpu9250::{Mpu9250, ImuMeasurements, Imu, I2cDevice, Dlpf, MpuConfig, AccelDataRate, GyroTempDataRate};

    //use shared_bus;

    static COUNTER_INT: AtomicUsize = AtomicUsize::new(0);


    #[shared]
    struct Shared {
        s2: Serial<pac::USART2>,
    }

    #[local]
    struct Local {
        led: Pin<'C', 13, Output<PushPull>>,
        pin_a0: Pin<'A', 0, Input>, //Button
        pin_b5: Pin<'B', 5, Input>,

        tim3: CounterHz<pac::TIM3>,

        imu: Mpu9250<I2cDevice<I2c<pac::I2C1>>, Imu>,

        //adc: Adc<pac::ADC1>,
        //pin_a6: Pin<'A', 6, Analog>, //Voltage Sense
        //pin_a7: Pin<'A', 7, Analog>, //Current Sense
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

        //I2C

        let pin_b6 = gpiob.pb6; //SCL
        let pin_b7 = gpiob.pb7; //SDA

        let i2c = ctx.device.I2C1.i2c(
            (pin_b6, pin_b7),
            i2cMode::Standard {
                frequency: 400.kHz(),
            },
            &clocks,
        );
        writeln!(s2, "i2c ok").ok();



        //TIMER1
        //advanced motor Timer

        let atimpins = (Channel1::new(gpioa.pa8),
                                                      Channel2::new(gpioa.pa9),
                                                      Channel3::new(gpioa.pa10),
                                                      Channel4::new(gpioa.pa11));

        let mut atim1 = ctx.device.TIM1.pwm_hz(atimpins, 1000.Hz(), &clocks);

        
        
        let (mut t1ch1, mut t1ch2, mut t1ch3, mut t1ch4) = atim1.split();

        
        

        //TIMER3
        // 16 bit Timer 3
        let mut tim3 = ctx.device.TIM3.counter_hz(&clocks);
        tim3.listen(Event::Update);
        

        //TIMER4
        // 16 bit Timer 4
        //let mut tim4 = ctx.device.TIM4.counter_hz(&clocks);
        //tim4.listen(Event::Update);

        let mut del = ctx.device.TIM4.delay_us(&clocks);

        writeln!(s2, "timer ok").ok();

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

        
        

        let mut imu = Mpu9250::imu_default(i2c, &mut del).unwrap();

        //let temp = imu.temp().unwrap();
        //writeln!(s2, "{}", temp).ok();

        imu.sample_rate_divisor(99).unwrap();
        
        imu.accel_data_rate(AccelDataRate::DlpfConf(Dlpf::_3)).unwrap();
        imu.gyro_temp_data_rate(GyroTempDataRate::DlpfConf(Dlpf::_3)).unwrap();


        
        //super precise :)
        
        let mut valuesatrest: [ImuMeasurements<[f32; 3]>; 10] = [ImuMeasurements {accel: [0.0; 3], gyro: [0.0; 3], temp: 0.0 }; 10];
        let mut means: ImuMeasurements<[f32; 3]> = ImuMeasurements {accel: [0.0; 3], gyro: [0.0; 3], temp: 0.0 };
        let mut sum: ImuMeasurements<[f32; 3]> = ImuMeasurements {accel: [0.0; 3], gyro: [0.0; 3], temp: 0.0 };
        writeln!(s2, "Calibrating...").ok();

        for n in 0..10 as usize{
            del.delay_us(1000000u32);
            valuesatrest[n] = imu.all().unwrap();

            
        }

        //writeln!(s2, "{:#?}", valuesatrest).ok();

        for n in valuesatrest{
            for k in 0..3{
                sum.accel[k] += n.accel[k];
                sum.gyro[k] +=  n.gyro[k];
            }
        }
        for l in 0..3{
            means.accel[l] = (-1.0)*sum.accel[l]/10.0;
            means.gyro[l] = (-1.0)*sum.gyro[l]/10.0;
        }
        means.accel[2] -= 9.81;
        //writeln!(s2, "{:#?}", means).ok();

        imu.set_accel_bias(true, means.accel).unwrap();
        imu.set_gyro_bias(true, means.gyro).unwrap();
        

        //imu.calibrate_at_rest::<stm32f4xx_hal::timer::Delay<stm32f4xx_hal::pac::TIM4, 1000000>, [f32; 3]>(&mut del).unwrap();

        imu.enable_interrupts(mpu9250::InterruptEnable::RAW_RDY_EN).unwrap();

        //let all: ImuMeasurements<[f32; 3]> = imu.all().unwrap();

        //writeln!(s2, "{:#?}", all).ok();


        tim3.start(1.Hz()).ok();


        //Button Interrupt
        let mut pin_a0 = gpioa.pa0.into_pull_up_input();
        pin_a0.make_interrupt_source(&mut sys_cfg);
        pin_a0.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);
        pin_a0.enable_interrupt(&mut ctx.device.EXTI);

        //MPU Interrupt
        let mut pin_b5 = gpiob.pb5.into_pull_up_input();
        pin_b5.make_interrupt_source(&mut sys_cfg);
        pin_b5.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);
        pin_b5.enable_interrupt(&mut ctx.device.EXTI);



        let mono = Systick::new(ctx.core.SYST, 100_000_000);
        
        (Shared {s2}, Local {led, tim3, pin_a0, pin_b5, imu}, init::Monotonics(mono))
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
        ctx.local.tim3.clear_interrupt(Event::Update);
        ctx.local.led.toggle();

        let val = COUNTER_INT.swap(0, Ordering::SeqCst);

        ctx.shared.s2.lock(|s2|{
            writeln!(s2, "last second: {}", val).ok();
        });

    }

    #[task(binds = EXTI0, local = [pin_a0], shared = [s2])]
    fn on_button(mut ctx: on_button::Context){
        ctx.local.pin_a0.clear_interrupt_pending_bit();

        ctx.shared.s2.lock(|s2|{
            writeln!(s2, "button interrupt").ok();
        });

    }

    #[task(binds = EXTI9_5, local = [pin_b5, imu], shared = [s2])]
    fn on_mpu(mut ctx: on_mpu::Context){
        
        COUNTER_INT.fetch_add(1, Ordering::SeqCst);

        let all: ImuMeasurements<[f32; 3]> = ctx.local.imu.all().unwrap();

        
        ctx.shared.s2.lock(|s2|{
            //writeln!(s2, "{:#?}", all).ok();
            writeln!(s2, "/*{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}*/", all.gyro[0], all.gyro[1], all.gyro[2], all.accel[0], all.accel[1], all.accel[2]).ok();
        });
        

        ctx.local.pin_b5.clear_interrupt_pending_bit();
    }


    #[panic_handler]
    fn panic(_info: &PanicInfo) -> ! {
        loop {}
    }

}
