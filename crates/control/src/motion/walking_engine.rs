use color_eyre::Result;
use context_attribute::context;
use framework::{AdditionalOutput, MainOutput};
use types::{
    configuration::{KickSteps, WalkingEngine as WalkingEngineConfiguration},
    BodyJointsCommand, Joints, MotionCommand, MotionSafeExits, RobotKinematics, SensorData, Step,
    StepAdjustment, SupportFoot, WalkCommand,
};

pub struct WalkingEngine {}

#[context]
pub struct CreationContext {
    pub config: Parameter<WalkingEngineConfiguration, "control/walking_engine">,
    pub kick_steps: Parameter<KickSteps, "control/kick_steps">,
    pub ready_pose: Parameter<Joints, "control/ready_pose">,

    pub motion_safe_exits: PersistentState<MotionSafeExits, "motion_safe_exits">,
    pub walk_return_offset: PersistentState<Step, "walk_return_offset">,
}

#[context]
pub struct CycleContext {
    pub step_adjustment: AdditionalOutput<StepAdjustment, "step_adjustment">,
    pub walking_engine: AdditionalOutput<WalkingEngineConfiguration, "walking_engine">,

    pub config: Parameter<WalkingEngineConfiguration, "control/walking_engine">,
    pub kick_steps: Parameter<KickSteps, "control/kick_steps">,
    pub ready_pose: Parameter<Joints, "control/ready_pose">,

    pub motion_safe_exits: PersistentState<MotionSafeExits, "motion_safe_exits">,
    pub walk_return_offset: PersistentState<Step, "walk_return_offset">,

    pub motion_command: RequiredInput<Option<MotionCommand>, "motion_command?">,
    pub robot_kinematics: Input<RobotKinematics, "robot_kinematics">,
    pub sensor_data: Input<SensorData, "sensor_data">,
    pub support_foot: RequiredInput<Option<SupportFoot>, "support_foot?">,
    pub walk_command: RequiredInput<Option<WalkCommand>, "walk_command?">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub walk_joints_command: MainOutput<Option<BodyJointsCommand>>,
}

impl WalkingEngine {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, _context: CycleContext) -> Result<MainOutputs> {
        Ok(MainOutputs::default())
    }
}
