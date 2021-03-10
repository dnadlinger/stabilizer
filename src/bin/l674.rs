// Cascaded IIR filter setup, where the output of channel 0 is used as the input
// to channel 1 (e.g. to drive both the slow and fast PZTs of a M-Squared SolsTiS
// laser).
//
// ADC1 can be summed to the first ADC0 IIR filter input or output for testing.

#![deny(warnings)]
#![no_std]
#![no_main]

use stm32h7xx_hal as hal;

use stabilizer::hardware;

use core::fmt::Write;
use heapless::{consts, String};
use log::{info, warn};
use miniconf::{
    embedded_nal::{IpAddr, Ipv4Addr},
    minimq, Miniconf, MqttInterface,
};
use serde::Deserialize;

use dsp::{iir, Lowpass};
use embedded_hal::digital::v2::OutputPin;
use hardware::{
    Adc0Input, Adc1Input, AfeGain, CycleCounterClock, Dac0Output, Dac1Output,
    NetworkStack, AFE0, AFE1,
};

const SCALE: f32 = i16::MAX as _;

const IIR_CASCADE_LENGTH: usize = 2;

/// Amount of bits to shift ADC1 samples before feeding into averaging low-pass filter.
/// 15 might be possible also (would need to look more closely at saturation behaviour).
const ADC1_LOWPASS_SHIFT: u8 = 14;

/// log2 of time constant of ADC1 lowpass filter in sample units, about 10 ms.
const ADC1_LOWPASS_LOG2_TC: u8 = 13;

const ADC1_FILTERED_TOPIC: &str = "dt/sinara/stabilizer/l674/read_adc1_filtered";

#[derive(Clone, Copy, Debug, Deserialize, Miniconf)]
pub enum ADC1Routing {
    Ignore,
    SumWithADC0,
    SumWithIIR0Output,
}

#[derive(Clone, Copy, Debug, Deserialize, Miniconf, PartialEq)]
pub enum LockMode {
    Disabled,
    RampPassThrough,
    Enabled,
}

#[derive(Debug, Default, Deserialize, Miniconf)]
pub struct Gains {
    proportional: f32,
    integral: f32,
}

#[derive(Debug, Default, Deserialize, Miniconf)]
pub struct Notch {
    frequency: f32,
    quality_factor: f32,
}

#[derive(Debug, Default, Deserialize, Miniconf)]
pub struct LockDetectConfig {
    adc1_threshold: f32,
    reset_time: f32,
}

#[derive(Debug, Deserialize, Miniconf)]
pub struct Settings {
    lock_mode: LockMode,
    gain_ramp_time: f32,

    fast_gains: Gains,

    fast_notch: Notch,
    fast_notch_enable: bool,

    slow_gains: Gains,
    slow_enable: bool,

    adc1_routing: ADC1Routing,
    lock_detect: LockDetectConfig,

    aux_ttl_out: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            lock_mode: LockMode::Disabled,
            gain_ramp_time: 0.0,
            fast_gains: Default::default(),
            fast_notch: Default::default(),
            fast_notch_enable: false,
            slow_gains: Default::default(),
            slow_enable: false,
            adc1_routing: ADC1Routing::Ignore,
            lock_detect: Default::default(),
            aux_ttl_out: false,
        }
    }
}

pub struct GainRampState {
    current: f32,
    increment: f32,
}

pub struct LockDetectState {
    decrement: u32,
    counter: u32,
    threshold: i16,
    ///< in units of ADC1 codes
    pin: hal::gpio::gpiod::PD3<hal::gpio::Output<hal::gpio::PushPull>>,
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        mqtt_interface: MqttInterface<
            Settings,
            NetworkStack,
            minimq::consts::U256,
            CycleCounterClock,
        >,

        // Format: iir_state[ch][cascade-no][coeff]
        #[init([[[0.; 5]; IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::Vec5; IIR_CASCADE_LENGTH]; 2],
        #[init([[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2])]
        iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
        #[init(LockMode::Disabled)]
        current_mode: LockMode,
        #[init(GainRampState { current: 1.0, increment: 0.0 })]
        gain_ramp: GainRampState,

        #[init(ADC1Routing::Ignore)]
        adc1_routing: ADC1Routing,

        adc1_filter: Lowpass<consts::U4>,
        #[init(0)]
        adc1_filtered: i32,

        lock_detect: LockDetectState,

        aux_ttl_out:
            hal::gpio::gpiod::PD4<hal::gpio::Output<hal::gpio::PushPull>>,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder, eem_gpio) =
            hardware::setup(c.core, c.device);

        let mqtt_interface = {
            let mqtt_client = {
                let broker = IpAddr::V4(Ipv4Addr::new(10, 255, 6, 4));
                minimq::MqttClient::new(
                    broker,
                    "stabilizer674",
                    stabilizer.net.stack,
                    CycleCounterClock::new(stabilizer.cycle_counter),
                )
                .unwrap()
            };

            MqttInterface::new(
                mqtt_client,
                "dt/sinara/stabilizer/l674",
                Settings::default(),
            )
            .unwrap()
        };

        // We hard-code gains here to save some complexity w.r.t. converting the
        // lock detect threshold from volts to codes.
        stabilizer.afes.0.set_gain(AfeGain::G1);
        stabilizer.afes.1.set_gain(AfeGain::G10);

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        // Set up lock detect GPIO output pins, which are specific to this
        // configuration.
        let (mut lock_detect_output, aux_ttl_out) = match eem_gpio {
            Some(eem_gpio) => (eem_gpio.lvds6, eem_gpio.lvds7),
            None => panic!("Pounder detected; GPIO pins not usable."),
        };
        lock_detect_output.set_low().unwrap();
        let lock_detect = LockDetectState {
            decrement: 1,
            counter: 0,
            threshold: i16::MAX,
            pin: lock_detect_output,
        };

        init::LateResources {
            mqtt_interface,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            adc1_filter: Lowpass::default(),
            lock_detect,
            aux_ttl_out,
        }
    }

    /// Main DSP processing routine for Stabilizer.
    ///
    /// # Note
    /// Processing time for the DSP application code is bounded by the following constraints:
    ///
    /// DSP application code starts after the ADC has generated a batch of samples and must be
    /// completed by the time the next batch of ADC samples has been acquired (plus the FIFO buffer
    /// time). If this constraint is not met, firmware will panic due to an ADC input overrun.
    ///
    /// The DSP application code must also fill out the next DAC output buffer in time such that the
    /// DAC can switch to it when it has completed the current buffer. If this constraint is not met
    /// it's possible that old DAC codes will be generated on the output and the output samples will
    /// be delayed by 1 batch.
    ///
    /// Because the ADC and DAC operate at the same rate, these two constraints actually implement
    /// the same time bounds, meeting one also means the other is also met.
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, iir_state, iir_ch, adc1_routing, adc1_filter, adc1_filtered, gain_ramp, lock_detect], priority=2)]
    fn process(c: process::Context) {
        let adc_samples = [
            c.resources.adcs.0.acquire_buffer(),
            c.resources.adcs.1.acquire_buffer(),
        ];

        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        fn to_dac(val: f32) -> u16 {
            // Note(unsafe): The filter limits ensure that the value is in range.
            // The truncation introduces 1/2 LSB distortion.
            let y = unsafe { val.to_int_unchecked::<i16>() };
            // Convert to DAC code
            y as u16 ^ 0x8000
        }

        let gain_ramp = c.resources.gain_ramp;
        for sample_idx in 0..adc_samples[0].len() {
            let adc1_int = adc_samples[1][sample_idx] as i16;
            *c.resources.adc1_filtered = c.resources.adc1_filter.update(
                (adc1_int as i32) << ADC1_LOWPASS_SHIFT,
                ADC1_LOWPASS_LOG2_TC,
            );

            {
                // Lock detect.
                let ld = &mut *c.resources.lock_detect;
                if adc1_int < ld.threshold {
                    ld.pin.set_low().unwrap();
                    ld.counter = u32::MAX;
                } else if ld.counter > 0 {
                    ld.counter = ld.counter.saturating_sub(ld.decrement);
                    if ld.counter == 0 {
                        ld.pin.set_high().unwrap();
                    }
                }
            }

            // Cascaded PID controllers.
            let adc1 = f32::from(adc1_int);
            let mut x = f32::from(adc_samples[0][sample_idx] as i16)
                * gain_ramp.current;
            if let ADC1Routing::SumWithADC0 = c.resources.adc1_routing {
                x += adc1;
            }

            let y0 = {
                let mut y = c.resources.iir_ch[0][0]
                    .update(&mut c.resources.iir_state[0][0], x);
                if let ADC1Routing::SumWithIIR0Output = c.resources.adc1_routing
                {
                    y += adc1;
                }
                c.resources.iir_ch[0][1]
                    .update(&mut c.resources.iir_state[0][1], y)
            };
            dac_samples[0][sample_idx] = to_dac(y0);

            let y1 = {
                let y = c.resources.iir_ch[1][0]
                    .update(&mut c.resources.iir_state[1][0], y0);
                c.resources.iir_ch[1][1]
                    .update(&mut c.resources.iir_state[1][1], y)
            };
            dac_samples[1][sample_idx] = to_dac(y1);

            if gain_ramp.current < 1.0 {
                gain_ramp.current += gain_ramp.increment;
                if gain_ramp.current > 1.0 {
                    gain_ramp.current = 1.0;
                }
            }
        }
    }

    #[idle(resources=[mqtt_interface, adc1_filtered], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
        info!("Starting idle task...");

        c.spawn.settings_update().unwrap();
        info!("Initial settings written.");

        let mut adc1_filtered = c.resources.adc1_filtered;
        let mut subscribed = false;
        loop {
            let sleep = c.resources.mqtt_interface.lock(|interface| {
                let now_ms = interface.client(|c| {
                    c.clock.cycle_counter.borrow_mut().current_ms()
                });
                !interface.network_stack().poll(now_ms)
            });

            match c.resources.mqtt_interface.lock(|interface| {
                if !subscribed {
                    interface.client(|c| match c.subscribe(ADC1_FILTERED_TOPIC, &[]) {
                        Ok(_) => {subscribed = true;},
                        Err(_) => {}
                    })
                }
                interface.update_or_process(
                    |client, topic, _message, properties| {
                        let mut payload_buf: String<consts::U64> =
                            String::new();
                        let payload = match topic {
                            ADC1_FILTERED_TOPIC => {
                                let filtered_int: i32 =
                                    adc1_filtered.lock(|a| *a);
                                // 16 signed ADC bits (set to 1V full-range).
                                const FULL_RANGE: i32 =
                                    1 << (15 + ADC1_LOWPASS_SHIFT);
                                write!(
                                    &mut payload_buf,
                                    "{}",
                                    (filtered_int as f32) / (FULL_RANGE as f32)
                                )
                                .unwrap();
                                payload_buf.as_bytes()
                            }
                            _ => "Unexpected topic".as_bytes(),
                        };
                        let response_topic = if let Some(
                            minimq::Property::ResponseTopic(topic),
                        ) =
                            properties.iter().find(|&prop| {
                                matches!(
                                    *prop,
                                    minimq::Property::ResponseTopic(_)
                                )
                            }) {
                            topic
                        } else {
                            "dt/sinara/stabilizer/l674/log"
                        };
                        client
                            .publish(
                                response_topic,
                                payload,
                                minimq::QoS::AtMostOnce,
                                &[],
                            )
                            .ok();
                    },
                )
            }) {
                Ok(updated) => {
                    if updated {
                        c.spawn.settings_update().unwrap()
                    } else if sleep {
                        cortex_m::asm::wfi();
                    }
                }
                Err(miniconf::MqttError::Disconnected) => {
                    if subscribed {
                        warn!("MQTT broker connection lost.");
                    }
                    subscribed = false;
                }
                Err(e) => {
                    warn!("Miniconf update failure: {:?}", e)
                }
            }
        }
    }

    #[task(priority = 1, resources=[mqtt_interface, afes, iir_ch, current_mode, gain_ramp, adc1_routing, lock_detect, aux_ttl_out])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = &c.resources.mqtt_interface.settings;

        let clk: hal::time::MegaHertz =
            hardware::design_parameters::TIMER_FREQUENCY;
        let sample_freq = (clk.0 as f32) * 1.0e6
            / (hardware::design_parameters::ADC_SAMPLE_TICKS as f32);
        let freq_factor = 2.0 / sample_freq;

        c.resources.iir_ch.lock(|iir| {
            match settings.lock_mode {
                LockMode::Disabled => {
                    iir[0][0].set_pi(0.0, 0.0, 0.0).unwrap();
                    iir[1][0].set_pi(0.0, 0.0, 0.0).unwrap();
                }
                LockMode::RampPassThrough => {
                    // Gain 5 gives approximately ±10 V when driven using the
                    // Vescent servo box ramp.
                    iir[0][0].set_pi(5.0, 0.0, 0.0).unwrap();
                    iir[1][0].set_pi(0.0, 0.0, 0.0).unwrap();
                }
                LockMode::Enabled => {
                    // Negative sign in fast branch to match AOM lock; both PZTs
                    // have same sign.
                    let fast_p = {
                        // KLUDGE: For whatever reason, copysign()-ing the I gain
                        // doesn't work for signed zero, so lower-bound P gain to
                        // just above zero.
                        let mut p = settings.fast_gains.proportional;
                        if p == 0.0 {
                            p = f32::MIN_POSITIVE;
                        }
                        p
                    };
                    iir[0][0]
                        .set_pi(
                            -fast_p,
                            freq_factor * settings.fast_gains.integral,
                            0.0,
                        )
                        .unwrap();
                    iir[1][0]
                        .set_pi(
                            settings.slow_gains.proportional,
                            freq_factor * settings.slow_gains.integral,
                            0.0,
                        )
                        .unwrap();
                }
            }
        });
        if settings.lock_mode != *c.resources.current_mode {
            c.resources.gain_ramp.lock(|gr| {
                if settings.lock_mode == LockMode::Enabled
                    && settings.gain_ramp_time > 0.0
                {
                    gr.current = 0.0;
                    gr.increment =
                        1.0 / (sample_freq * settings.gain_ramp_time);
                } else {
                    gr.current = 1.0;
                    gr.increment = 0.0;
                }
            });
        }
        *c.resources.current_mode = settings.lock_mode;

        {
            let threshold = settings.lock_detect.adc1_threshold * 32768.0;
            if threshold < -32768.0 || threshold > 32767.0 {
                warn!("Ignoring invalid lock detect threshold: {}", threshold);
            } else {
                c.resources.lock_detect.lock(|state| {
                    state.threshold = threshold as i16;
                });
            }

            let mut reset_samples =
                settings.lock_detect.reset_time * sample_freq;
            if reset_samples < 1.0 {
                warn!(
                    "Lock detect reset time too small, clamping: {}",
                    settings.lock_detect.reset_time
                );
                reset_samples = 1.0;
            }
            let mut decrement = ((u32::MAX as f32) / reset_samples) as u32;
            if decrement == 0 {
                warn!(
                    "Lock detect reset time too large, clamping: {}",
                    settings.lock_detect.reset_time
                );
                decrement = 1;
            }
            c.resources.lock_detect.lock(|state| {
                state.decrement = decrement;
            });
        }

        c.resources.iir_ch.lock(|iir| {
            if settings.fast_notch_enable {
                iir[0][1]
                    .set_notch(
                        freq_factor * settings.fast_notch.frequency,
                        settings.fast_notch.quality_factor,
                    )
                    .unwrap();
            } else {
                iir[0][1].set_scale(1.0).unwrap();
            }
        });

        c.resources.iir_ch.lock(|iir| {
            // Second IIR on slow PZT unused - optional low-pass filter?
            iir[1][1].set_scale(1.0).unwrap();
        });

        let iir = c.resources.iir_ch.lock(|iir| *iir);
        {
            info!("IIR settings:");
            info!(" [0][0]: {:?}", iir[0][0]);
            info!(" [0][1]: {:?}", iir[0][1]);
            info!(" [1][0]: {:?}", iir[1][0]);
            info!(" [1][1]: {:?}", iir[1][1]);
        }

        c.resources
            .adc1_routing
            .lock(|r| *r = settings.adc1_routing);

        if settings.aux_ttl_out {
            c.resources.aux_ttl_out.set_high().unwrap();
        } else {
            c.resources.aux_ttl_out.set_low().unwrap();
        }
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 3)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI3, priority = 3)]
    fn spi3(_: spi3::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI4, priority = 3)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 output error");
    }

    #[task(binds = SPI5, priority = 3)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 output error");
    }

    extern "C" {
        // hw interrupt handlers for RTIC to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};
