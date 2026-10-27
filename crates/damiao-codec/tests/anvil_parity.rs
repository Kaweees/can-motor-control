//! Golden data captured by invoking the installed Anvil control library offline.
//! See fixtures/README.md for the exact reference and capture procedure.
use damiao_codec::DamiaoCodec;
use motor_codec::{CanFrame, Command, Event, MotorCodec, MotorRef, MotorTypeId};

fn bytes(hex: &str) -> Vec<u8> {
    hex.as_bytes()
        .chunks_exact(2)
        .map(|s| u8::from_str_radix(core::str::from_utf8(s).unwrap(), 16).unwrap())
        .collect()
}

#[test]
fn installed_anvil_limits_and_mit_frames() {
    let codec = DamiaoCodec::new();
    for line in include_str!("fixtures/anvil_control.csv").lines() {
        let cols: Vec<_> = line.split(',').collect();
        let motor_type = MotorTypeId::Damiao(cols[1].parse().unwrap());
        let number = |i: usize| cols[i].parse::<f64>().unwrap();
        match cols[0] {
            "limits" => {
                let lim = codec.limits(motor_type).unwrap();
                assert_eq!(
                    (lim.p_max, lim.v_max, lim.t_max),
                    (number(2), number(3), number(4)),
                    "{line}"
                );
            }
            "mit" => {
                let motor = MotorRef {
                    motor_type,
                    send_id: 1,
                    recv_id: 0x11,
                    name: "j1",
                };
                let frame = codec
                    .encode_command(
                        motor,
                        &Command::Mit {
                            kp: number(2),
                            kd: number(3),
                            q: number(4),
                            dq: number(5),
                            tau: number(6),
                        },
                    )
                    .unwrap_or_else(|e| panic!("{line}: {e}"));
                assert_eq!(frame.id, 1);
                assert_eq!(frame.payload(), bytes(cols[7]), "{line}");
            }
            "state" => {
                let motor = MotorRef {
                    motor_type,
                    send_id: 1,
                    recv_id: 0x11,
                    name: "j1",
                };
                let frame = CanFrame::classical(0x11, &bytes(cols[2])).unwrap();
                let Some(Event::Feedback { feedback, .. }) =
                    codec.decode_for_motor(&frame, motor).unwrap()
                else {
                    panic!("missing feedback: {line}");
                };
                for (actual, expected) in [
                    (feedback.q, number(3)),
                    (feedback.dq, number(4)),
                    (feedback.tau, number(5)),
                ] {
                    assert!(
                        (actual - expected).abs() < 1e-12,
                        "{line}: {actual} != {expected}"
                    );
                }
                assert_eq!((feedback.t_mos, feedback.t_rotor), (30, 35));
                assert!(feedback.enabled);
                assert_eq!(feedback.fault, None);
            }
            _ => {}
        }
    }
}

#[test]
fn short_feedback_is_ignored_without_panicking() {
    for n in 0..8 {
        let frame = CanFrame::classical(0x11, &vec![0x11; n]).unwrap();
        assert!(DamiaoCodec::new().decode(&frame).unwrap().is_none());
    }
}

#[test]
fn position_bytes_are_not_parameter_opcodes() {
    let motor = MotorRef {
        motor_type: MotorTypeId::Damiao(3),
        send_id: 1,
        recv_id: 0x11,
        name: "j1",
    };
    for byte in [0x33, 0x55] {
        let frame = CanFrame::classical(0x11, &[0x11, 0x91, byte, 0x90, 9, 0, 30, 35]).unwrap();
        assert!(DamiaoCodec::new()
            .decode_for_motor(&frame, motor)
            .unwrap()
            .is_some());
    }
}

#[test]
fn context_free_decode_never_guesses_motor_type() {
    let frame = CanFrame::classical(0x11, &[0x11, 255, 255, 255, 255, 255, 30, 35]).unwrap();
    let error = DamiaoCodec::new().decode(&frame).unwrap_err();
    assert!(error.to_string().contains("decode_for_motor"));
}
