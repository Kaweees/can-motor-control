//! Golden data captured by invoking the installed Anvil control library offline.
//! See fixtures/README.md for the exact reference and capture procedure.
use damiao_codec::DamiaoCodec;
use motor_codec::{Command, MotorCodec, MotorRef, MotorTypeId};

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
            _ => {}
        }
    }
}
