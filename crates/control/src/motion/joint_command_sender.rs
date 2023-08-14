use std::collections::HashMap;

use byteorder::{ByteOrder, LittleEndian};
use color_eyre::{eyre::WrapErr, Result};
use context_attribute::context;
use framework::AdditionalOutput;
use hardware::ActuatorInterface;
use simple_websockets::{Event, EventHub, Responder};
use types::{
    BodyJointsCommand, ForceSensitiveResistors, HeadJointsCommand, InertialMeasurementUnitData, Joints, JointsCommand, Leds,
    MotionSafeExits, MotionSelection, MotionType, SensorData, WorldState,
};

const ACTION_SIZE: usize = 26;
const OBSERVATION_SIZE: usize = 2 * 26 + 2 * 8 + 1; // + 2;

pub struct JointCommandSender {
    positions_residual: Joints<f32>,
    event_hub: EventHub,
    clients: HashMap<u64, Responder>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    positions: AdditionalOutput<Joints<f32>, "positions">,
    compensated_positions: AdditionalOutput<Joints<f32>, "compensated_positions">,
    positions_difference: AdditionalOutput<Joints<f32>, "positions_difference">,
    positions_residual: AdditionalOutput<Joints<f32>, "positions_offset">,
    stiffnesses: AdditionalOutput<Joints<f32>, "stiffnesses">,
    motion_safe_exits_output: AdditionalOutput<MotionSafeExits, "motion_safe_exits_output">,
    motion_safe_exits: PersistentState<MotionSafeExits, "motion_safe_exits">,
    joint_calibration_offsets: Parameter<Joints<f32>, "joint_calibration_offsets">,
    penalized_pose: Parameter<Joints<f32>, "penalized_pose">,
    arms_up_squat_joints_command: Input<JointsCommand<f32>, "arms_up_squat_joints_command">,
    dispatching_command: Input<JointsCommand<f32>, "dispatching_command">,
    energy_saving_stand_command: Input<BodyJointsCommand<f32>, "energy_saving_stand_command">,
    fall_protection_command: Input<JointsCommand<f32>, "fall_protection_command">,
    head_joints_command: Input<HeadJointsCommand<f32>, "head_joints_command">,
    jump_left_joints_command: Input<JointsCommand<f32>, "jump_left_joints_command">,
    jump_right_joints_command: Input<JointsCommand<f32>, "jump_right_joints_command">,
    motion_selection: Input<MotionSelection, "motion_selection">,
    sensor_data: Input<SensorData, "sensor_data">,
    sit_down_joints_command: Input<JointsCommand<f32>, "sit_down_joints_command">,
    stand_up_back_positions: Input<Joints<f32>, "stand_up_back_positions">,
    stand_up_front_positions: Input<Joints<f32>, "stand_up_front_positions">,
    walk_joints_command: Input<BodyJointsCommand<f32>, "walk_joints_command">,
    world_state: Input<WorldState, "world_state">,
    hardware_interface: HardwareInterface,
    leds: Input<Leds, "leds">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {}

impl JointCommandSender {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            positions_residual: Joints::<f32>::default(),
            event_hub: simple_websockets::launch(9990).expect("failed to listen on port 9990"),
            clients: HashMap::new(),
        })
    }

    pub fn cycle(
        &mut self,
        mut context: CycleContext<impl ActuatorInterface>,
    ) -> Result<MainOutputs> {
        let current_positions = context.sensor_data.positions;
        let dispatching_command = context.dispatching_command;
        let fall_protection_positions = context.fall_protection_command.positions;
        let fall_protection_stiffnesses = context.fall_protection_command.stiffnesses;
        let head_joints_command = context.head_joints_command;
        let motion_selection = context.motion_selection;
        let arms_up_squat = context.arms_up_squat_joints_command;
        let inertial_measurement_unit = &context.sensor_data.inertial_measurement_unit;
        let force_sensitive_resistors = &context.sensor_data.force_sensitive_resistors;
        let jump_left = context.jump_left_joints_command;
        let jump_right = context.jump_right_joints_command;
        let sit_down = context.sit_down_joints_command;
        let stand_up_back_positions = context.stand_up_back_positions;
        let stand_up_front_positions = context.stand_up_front_positions;
        let walk = context.walk_joints_command;

        let (positions, stiffnesses) = match motion_selection.current_motion {
            MotionType::ArmsUpSquat => (arms_up_squat.positions, arms_up_squat.stiffnesses),
            MotionType::Dispatching => (
                dispatching_command.positions,
                dispatching_command.stiffnesses,
            ),
            MotionType::FallProtection => (fall_protection_positions, fall_protection_stiffnesses),
            MotionType::JumpLeft => (jump_left.positions, jump_left.stiffnesses),
            MotionType::JumpRight => (jump_right.positions, jump_right.stiffnesses),
            MotionType::Penalized => (*context.penalized_pose, Joints::fill(0.8)),
            MotionType::SitDown => (sit_down.positions, sit_down.stiffnesses),
            MotionType::Stand => (
                Joints::from_head_and_body(head_joints_command.positions, walk.positions),
                Joints::from_head_and_body(head_joints_command.stiffnesses, walk.stiffnesses),
            ),
            MotionType::StandUpBack => (*stand_up_back_positions, Joints::fill(1.0)),
            MotionType::StandUpFront => (*stand_up_front_positions, Joints::fill(1.0)),
            MotionType::Unstiff => (current_positions, Joints::fill(0.0)),
            MotionType::Walk => (
                Joints::from_head_and_body(head_joints_command.positions, walk.positions),
                Joints::from_head_and_body(head_joints_command.stiffnesses, walk.stiffnesses),
            ),
            MotionType::EnergySavingStand => (
                Joints::from_head_and_body(
                    head_joints_command.positions,
                    context.energy_saving_stand_command.positions,
                ),
                Joints::from_head_and_body(
                    head_joints_command.stiffnesses,
                    context.energy_saving_stand_command.stiffnesses,
                ),
            ),
        };

        // The actuators uses the raw sensor data (not corrected like current_positions) in their feedback loops,
        // thus the compensation is required to make them reach the actual desired position.
        let compensated_positions = positions + *context.joint_calibration_offsets;

        while !self.event_hub.is_empty() {
            match self.event_hub.poll_event() {
                Event::Connect(client_id, responder) => {
                    self.clients.insert(client_id, responder);
                }
                Event::Disconnect(client_id) => {
                    self.clients.remove(&client_id);
                }
                Event::Message(client_id, message) => {
                    // read action
                    let bytes: Vec<u8> = match message {
                        simple_websockets::Message::Text(_) => todo!(),
                        simple_websockets::Message::Binary(bin) => bin,
                    };
                    let mut action = [0.0; ACTION_SIZE];
                    LittleEndian::read_f32_into(&bytes, &mut action);

                    // apply action
                    self.positions_residual = Joints::from_angles(action);

                    // respond with observation
                    let responder = self.clients.get(&client_id).unwrap();
                    let mut bytes = [0; OBSERVATION_SIZE * 4];
                    let observation = compose_observation(
                        &current_positions,
                        &compensated_positions,
                        flat_imu(inertial_measurement_unit),
                        flat_fsr(force_sensitive_resistors),
                        context.world_state,
                    );
                    LittleEndian::write_f32_into(&observation, &mut bytes);
                    responder.send(simple_websockets::Message::Binary(bytes.into()));
                }
            }
        }

        context
            .hardware_interface
            .write_to_actuators(
                compensated_positions + self.positions_residual,
                stiffnesses,
                *context.leds,
            )
            .wrap_err("failed to write to actuators")?;

        context.positions.fill_if_subscribed(|| positions);

        context
            .compensated_positions
            .fill_if_subscribed(|| compensated_positions);

        context
            .positions_residual
            .fill_if_subscribed(|| self.positions_residual);

        context
            .positions_difference
            .fill_if_subscribed(|| positions - current_positions);

        context.stiffnesses.fill_if_subscribed(|| stiffnesses);

        context
            .motion_safe_exits_output
            .fill_if_subscribed(|| context.motion_safe_exits.clone());

        Ok(MainOutputs {})
    }
}

fn flat_imu(imu: &InertialMeasurementUnitData) -> [f32; 8] {
    [
        imu.linear_acceleration[0],
        imu.linear_acceleration[1],
        imu.linear_acceleration[2],
        imu.angular_velocity[0],
        imu.angular_velocity[1],
        imu.angular_velocity[2],
        imu.roll_pitch[0],
        imu.roll_pitch[1],
    ]
}

fn flat_fsr(fsr: &ForceSensitiveResistors) -> [f32; 8] {
    [
        fsr.left.front_left,
        fsr.left.front_right,
        fsr.left.rear_left,
        fsr.left.rear_right,
        fsr.right.front_left,
        fsr.right.front_right,
        fsr.right.rear_left,
        fsr.right.rear_right,
    ]
}

fn compose_observation(
    current_positions: &Joints<f32>,
    positions: &Joints<f32>,
    inertial_measurement_unit: [f32; 8],
    force_sensitive_resistors: [f32; 8],
    world_state: &WorldState,
) -> [f32; OBSERVATION_SIZE] {
    let mut distance_to_kick_pose = [10.0];
    if world_state.kick_decisions.is_some() {
        distance_to_kick_pose = [world_state.kick_decisions.as_ref().unwrap()[0]
            .kick_pose
            .translation
            .vector
            .norm()]
    }
    <[f32; OBSERVATION_SIZE]>::try_from(
        [
            current_positions.to_angles().as_slice(),
            positions.to_angles().as_slice(),
            inertial_measurement_unit.as_slice(),
            force_sensitive_resistors.as_slice(),
            distance_to_kick_pose.as_slice(),
        ]
        .concat(),
    )
    .unwrap()
}
