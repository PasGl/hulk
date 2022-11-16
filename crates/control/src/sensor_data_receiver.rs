use context_attribute::context;
use framework::{MainOutput, Input, PerceptionInput};
use types::{Ball, Leds, PrimaryState, SensorData};

pub struct SensorDataReceiver {}

#[context]
pub struct NewContext {}

#[context]
pub struct CycleContext {}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub sensor_data: MainOutput<SensorData>,
}

impl SensorDataReceiver {
    pub fn new(_context: NewContext) -> anyhow::Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, _context: CycleContext) -> anyhow::Result<MainOutputs> {
        Ok(MainOutputs::default())
    }
}