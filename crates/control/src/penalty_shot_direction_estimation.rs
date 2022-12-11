use color_eyre::Result;
use context_attribute::context;
use framework::MainOutput;
use types::{
    BallPosition, FieldDimensions, GameControllerState, PenaltyShotDirection, PrimaryState,
};

pub struct PenaltyShotDirectionEstimation {}

#[context]
pub struct CreationContext {
    pub field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    pub moving_distance_threshold:
        Parameter<f32, "control/penalty_shot_direction_estimation/moving_distance_threshold">,
}

#[context]
pub struct CycleContext {
    pub field_dimensions: Parameter<FieldDimensions, "field_dimensions">,
    pub moving_distance_threshold:
        Parameter<f32, "control/penalty_shot_direction_estimation/moving_distance_threshold">,

    pub ball_position: RequiredInput<Option<BallPosition>, "ball_position?">,
    pub game_controller_state: RequiredInput<Option<GameControllerState>, "game_controller_state?">,
    pub primary_state: RequiredInput<Option<PrimaryState>, "primary_state?">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub penalty_shot_direction: MainOutput<Option<PenaltyShotDirection>>,
}

impl PenaltyShotDirectionEstimation {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, _context: CycleContext) -> Result<MainOutputs> {
        Ok(MainOutputs::default())
    }
}
