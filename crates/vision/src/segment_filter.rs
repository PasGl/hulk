use context_attribute::context;
use framework::{MainOutput, Input};
use types::{FieldBorder, FilteredSegments, ImageSegments};

pub struct SegmentFilter {}

#[context]
pub struct NewContext {}

#[context]
pub struct CycleContext {
    pub field_border: Input<FieldBorder, "field_border?">,
    pub image_segments: Input<ImageSegments, "image_segments?">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub filtered_segments: MainOutput<FilteredSegments>,
}

impl SegmentFilter {
    pub fn new(_context: NewContext) -> anyhow::Result<Self> {
        Ok(Self {})
    }

    pub fn cycle(&mut self, _context: CycleContext) -> anyhow::Result<MainOutputs> {
        Ok(MainOutputs::default())
    }
}