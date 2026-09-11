//! CRSF RC channels → Linux uinput virtual joystick.
//!
//! Pure mapping layer: takes a 16-channel CRSF RC frame and emits the
//! corresponding evdev `InputEvent`s on a virtual `/dev/uinput` device
//! that appears to userspace as a `CRSF Joystick` controller.
//!
//! No Zenoh, no protocol parsing — the binary in `src/main.rs` glues
//! CRSF subscriptions to this layer. The mapping is opinionated towards
//! a Radiomaster Pocket layout (sticks → ABS_X/Y/Z/RX, S1 pot → ABS_RUDDER,
//! SD/SE/SA switches → buttons + ABS_THROTTLE/WHEEL, four trims → eight
//! more buttons), with the bus identifying as the radio's USB
//! vendor/product so flight sims that have a per-controller bind file
//! match it the same way.

use evdev::uinput::VirtualDevice;
use evdev::{AbsoluteAxisCode, AttributeSet, InputId, KeyCode, MiscCode, UinputAbsSetup};
use metrics::counter;

/// CRSF physical maximum (PWM 2012 µs).
pub const AXIS_MAX: u16 = 1811;

/// CRSF 11-bit nominal minimum (PWM 988 µs).
pub const AXIS_MIN: u16 = 172;

/// Channel midpoint — splits 2-pos switches and gates source selection.
pub const AXIS_MID: u16 = 992;

/// 3-position switch thresholds. Below LEFT = position-0, above RIGHT =
/// position-2, in between = position-1.
pub const AXIS_3POS_LEFT: u16 = 592;
pub const AXIS_3POS_RIGHT: u16 = 1392;

/// Safe default channel values used when the active RC source goes silent.
/// Sticks are centered, throttle is low, and switches are in safe/low
/// positions.
pub const SAFE_DEFAULT_CHANNELS: [u16; 16] = {
    let mut c = [AXIS_MID; 16];
    c[2] = AXIS_MIN; // throttle low
    c[4] = AXIS_MIN; // SD / arm-disarm low
    c[5] = AXIS_MIN; // SE low
    c[7] = AXIS_MIN; // SA low
    c
};

/// A virtual joystick driven by 16-channel CRSF RC frames.
pub struct Joystick {
    old_channels: [u16; 16],
    device: VirtualDevice,
}

impl Joystick {
    /// Create the virtual device. Requires write access to `/dev/uinput`.
    pub fn new() -> std::io::Result<Self> {
        let mut keys = AttributeSet::<KeyCode>::new();
        for k in [
            KeyCode::BTN_TRIGGER,
            KeyCode::BTN_THUMB,
            KeyCode::BTN_THUMB2,
            KeyCode::BTN_TOP,
            KeyCode::BTN_TOP2,
            KeyCode::BTN_PINKIE,
            KeyCode::BTN_BASE,
            KeyCode::BTN_BASE2,
            KeyCode::BTN_BASE3,
            KeyCode::BTN_BASE4,
            KeyCode::BTN_BASE5,
            KeyCode::BTN_BASE6,
            KeyCode::new(KeyCode::BTN_BASE6.code() + 1), // 0x12d
        ] {
            keys.insert(k);
        }

        let abs_setup = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_X,
            evdev::AbsInfo::new(AXIS_MID.into(), AXIS_MIN.into(), AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_y = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_Y,
            evdev::AbsInfo::new(AXIS_MID.into(), AXIS_MIN.into(), AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_z = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_Z,
            evdev::AbsInfo::new(AXIS_MIN.into(), AXIS_MIN.into(), AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_rx = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_RX,
            evdev::AbsInfo::new(AXIS_MID.into(), AXIS_MIN.into(), AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_throttle = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_THROTTLE,
            evdev::AbsInfo::new(AXIS_MIN.into(), AXIS_MIN.into(), AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_rudder = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_RUDDER,
            evdev::AbsInfo::new(AXIS_MID.into(), AXIS_MIN.into(), AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_wheel = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_WHEEL,
            evdev::AbsInfo::new(AXIS_MIN.into(), AXIS_MIN.into(), AXIS_MAX.into(), 7, 127, 0),
        );

        let mut msc_set = AttributeSet::<MiscCode>::new();
        msc_set.insert(MiscCode::MSC_SCAN);

        let device = VirtualDevice::builder()?
            .name("CRSF Joystick")
            .input_id(InputId::new(evdev::BusType::BUS_USB, 0x1209, 0x4f54, 0)) // Radiomaster Pocket vendor/product
            .with_keys(&keys)?
            .with_absolute_axis(&abs_setup)?
            .with_absolute_axis(&abs_y)?
            .with_absolute_axis(&abs_z)?
            .with_absolute_axis(&abs_rx)?
            .with_absolute_axis(&abs_throttle)?
            .with_absolute_axis(&abs_rudder)?
            .with_absolute_axis(&abs_wheel)?
            .with_msc(&msc_set)?
            .build()?;

        Ok(Self {
            old_channels: [0xffff; 16], // Different initial value to force update
            device,
        })
    }

    /// Update the virtual joystick from a 16-channel CRSF RC frame. Only
    /// channels that differ from the previous call generate InputEvents.
    pub fn update(&mut self, channels: [u16; 16]) -> std::io::Result<()> {
        let mut events = Vec::<evdev::InputEvent>::new();
        let dev = &mut self.device;
        let old = self.old_channels;

        // 0 AIL (ABS_X)
        if channels[0] != old[0] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_X.0,
                channels[0] as i32,
            )]);
        }
        // 1 ELE (ABS_Y)
        if channels[1] != old[1] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_Y.0,
                channels[1] as i32,
            )]);
        }
        // 2 THR (ABS_Z)
        if channels[2] != old[2] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_Z.0,
                channels[2] as i32,
            )]);
        }
        // 3 RUD (ABS_RX)
        if channels[3] != old[3] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_RX.0,
                channels[3] as i32,
            )]);
        }

        // 4 SD disarm/arm button(s) + ABS_THROTTLE
        if channels[4] != old[4] {
            let val = channels[4] as i32;
            events.extend(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_TRIGGER.0,
                    if channels[4] < AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_THUMB.0,
                    if channels[4] >= AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::ABSOLUTE.0,
                    AbsoluteAxisCode::ABS_THROTTLE.0,
                    val,
                ),
            ]);
        }

        // 5 button SE (2POS, momentary) -> BTN_THUMB2
        if channels[5] != old[5] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::KEY.0,
                KeyCode::BTN_THUMB2.0,
                if channels[5] >= AXIS_MID { 1 } else { 0 },
            )]);
        }

        // 6 S1-pot -> ABS_RUDDER
        if channels[6] != old[6] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_RUDDER.0,
                channels[6] as i32,
            )]);
        }

        // 7 button SA (2POS, fixed) -> BTN_BASE6 / BTN_BASE6+1 + ABS_WHEEL
        if channels[7] != old[7] {
            let val = channels[7] as i32;
            events.extend(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE6.0,
                    if channels[7] < AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::new(KeyCode::BTN_BASE6.code() + 1).0,
                    if channels[7] >= AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::ABSOLUTE.0,
                    AbsoluteAxisCode::ABS_WHEEL.0,
                    val,
                ),
            ]);
        }

        // 8: RUD trim
        if channels[8] != old[8] {
            events.extend(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_TOP.0,
                    if channels[8] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_TOP2.0,
                    if channels[8] >= AXIS_3POS_RIGHT { 1 } else { 0 },
                ),
            ]);
        }
        // 9: ELE trim
        if channels[9] != old[9] {
            events.extend(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_PINKIE.0,
                    if channels[9] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE.0,
                    if channels[9] >= AXIS_3POS_RIGHT { 1 } else { 0 },
                ),
            ]);
        }
        // 10: THR trim
        if channels[10] != old[10] {
            events.extend(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE2.0,
                    if channels[10] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE3.0,
                    if channels[10] >= AXIS_3POS_RIGHT {
                        1
                    } else {
                        0
                    },
                ),
            ]);
        }
        // 11: AIL trim
        if channels[11] != old[11] {
            events.extend(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE4.0,
                    if channels[11] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE5.0,
                    if channels[11] >= AXIS_3POS_RIGHT {
                        1
                    } else {
                        0
                    },
                ),
            ]);
        }

        self.old_channels = channels;

        if !events.is_empty() {
            counter!("joystick.uinput.update").increment(1);
            dev.emit(&events)?;
        }
        Ok(())
    }
}
