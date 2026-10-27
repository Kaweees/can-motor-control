use can_motor_control::{CanFrame, MockCanBus, MotorSpec, RobotBuilder};
use damiao_codec::{DamiaoCodec, DamiaoMotorType};
use std::time::Duration;

#[test]
fn mixed_openyam_motors_decode_with_their_own_limits() {
    let bus = MockCanBus::new("follower_l");
    let mut robot = RobotBuilder::new()
        .add_bus(
            "follower_l",
            Box::new(bus.clone()),
            Box::new(DamiaoCodec::new()),
        )
        .add_arm(
            "arm",
            "follower_l",
            vec![
                MotorSpec::new("j1", DamiaoMotorType::DM4340, 1, 0x11),
                MotorSpec::new("j4", DamiaoMotorType::DM4310, 4, 0x14),
            ],
        )
        .build()
        .unwrap();
    robot.connect().unwrap();
    for (recv, send) in [(0x11, 1), (0x14, 4)] {
        bus.inject_frame(
            CanFrame::classical(recv, &[0x10 | send, 255, 255, 255, 255, 255, 30, 35]).unwrap(),
        );
    }
    robot.tick(Duration::ZERO).unwrap();
    let arm = robot.group("arm").unwrap().as_arm().unwrap();
    assert_eq!(arm.velocities(), vec![8.0, 30.0]);
    assert_eq!(arm.torques(), vec![28.0, 10.0]);
}

#[test]
fn feedback_reports_faults_without_losing_mechanical_state() {
    let bus = MockCanBus::new("follower_l");
    let mut robot = RobotBuilder::new()
        .add_bus(
            "follower_l",
            Box::new(bus.clone()),
            Box::new(DamiaoCodec::new()),
        )
        .add_arm(
            "arm",
            "follower_l",
            vec![MotorSpec::new("j1", DamiaoMotorType::DM4340, 1, 0x11)],
        )
        .build()
        .unwrap();
    robot.connect().unwrap();
    for status in [8u8, 1, 0] {
        bus.inject_frame(
            CanFrame::classical(0x11, &[status << 4 | 1, 255, 255, 255, 255, 255, 30, 35]).unwrap(),
        );
        robot.tick(Duration::ZERO).unwrap();
        let motor = robot
            .group("arm")
            .unwrap()
            .as_arm()
            .unwrap()
            .motor("j1")
            .unwrap();
        assert_eq!(motor.position(), 12.5);
        assert_eq!(
            motor.fault().map(|f| f.0),
            (status >= 8).then_some(status as u16)
        );
        assert_eq!(motor.is_enabled(), status == 1);
    }
}
