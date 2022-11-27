use std::{
    mem::{size_of, ManuallyDrop},
    sync::Arc,
    time::SystemTime,
};

use color_eyre::Result;

use crate::YCbCr422;

use super::{CameraPosition, Joints, Leds, SensorData};

pub trait Interface {
    fn read_from_microphones(&self) -> Result<Samples>;

    fn get_now(&self) -> SystemTime;
    fn get_ids(&self) -> Ids;
    fn read_from_sensors(&self) -> Result<SensorData>;
    fn write_to_actuators(&self, positions: Joints, stiffnesses: Joints, leds: Leds) -> Result<()>;

    fn read_from_network(&self) -> Result<Message>;
    fn write_to_network(&self, message: Message) -> Result<()>;

    fn read_from_camera(&self, camera_position: CameraPosition) -> Result<Image>;
}

#[derive(Clone, Debug)]
pub struct Ids {
    pub body_id: String,
    pub head_id: String,
}

#[derive(Clone)]
pub struct Image {
    _buffer: Arc<Vec<YCbCr422>>,
}

impl Image {
    pub fn from_ycbcr_buffer(buffer: Vec<YCbCr422>) -> Self {
        Self {
            _buffer: Arc::new(buffer),
        }
    }

    pub fn from_raw_buffer(buffer: Vec<u8>) -> Self {
        let mut buffer = ManuallyDrop::new(buffer);

        let u8_pointer = buffer.as_mut_ptr();
        let u8_length = buffer.len();
        let u8_capacity = buffer.capacity();

        assert_eq!(u8_length % size_of::<YCbCr422>(), 0);
        assert_eq!(u8_capacity % size_of::<YCbCr422>(), 0);

        let ycbcr_pointer = u8_pointer as *mut YCbCr422;
        let ycbcr_length = u8_length / size_of::<YCbCr422>();
        let ycbcr_capacity = u8_capacity / size_of::<YCbCr422>();

        let buffer = unsafe { Vec::from_raw_parts(ycbcr_pointer, ycbcr_length, ycbcr_capacity) };

        Self {
            _buffer: Arc::new(buffer),
        }
    }
}

pub struct Message {}

#[derive(Clone)]
pub struct Samples {
    pub rate: u32,
    pub channels_of_samples: Arc<Vec<Vec<f32>>>,
}