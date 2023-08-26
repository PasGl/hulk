use types::{
    HeadMotion, MotionCommand, 
};

pub fn execute() -> Option<MotionCommand> {
    Some(MotionCommand::Stand {
        head: HeadMotion::Center,
        is_energy_saving: true,
    })
}
