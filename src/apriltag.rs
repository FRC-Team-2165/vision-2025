#![allow(dead_code)]
use apriltag_rs::{Image, Detector, CameraIntrinsics, TagFamily};

use vistream::{Locate, LocationData, FrameSource};
use vistream::frame::{Luma, Pixelate};
use vistream::error::{Result};


pub struct AprilTagLocator {
    detector: Detector,
}

impl AprilTagLocator {
    pub fn new(families: &[TagFamily]) -> AprilTagLocator {
        let mut detector = Detector::new();
        for family in families {
            detector.add(*family);
        }

        AprilTagLocator {
            detector,
        }
    }
}

impl<S: FrameSource<Luma>> Locate<Luma, S> for AprilTagLocator {
    fn locate(&mut self, source: &mut S) -> Result<Vec<LocationData>> {
        let Some(frame) = source.get_frame()? else {
            return Ok(Vec::new());
        };
        
        let image = Image::new(frame.width() as u32, frame.height() as u32, frame.bytes());
        let locations: Vec<_> = self.detector.detect(image).into_iter().map(|detection| {
            let pos = detection.center();
            let x = pos[0];
            let y = pos[1];

            let mut builder = LocationData::builder();
            builder.x(x).y(y).id(detection.id()).build().unwrap()
        }).collect();

        Ok(locations)
    }
}

pub struct AprilTag3dLocator {
    detector: Detector,
    intrinsics: CameraIntrinsics,
    tag_size: f64,
}

impl AprilTag3dLocator {
    pub fn new(families: &[TagFamily], intrinsics: CameraIntrinsics, tag_size: f64) -> AprilTag3dLocator {
        let mut detector = Detector::new();
        for family in families {
            detector.add(*family);
        }

        AprilTag3dLocator {
            detector,
            intrinsics,
            tag_size,
        }
    }
}

impl<S: FrameSource<Luma>> Locate<Luma, S> for AprilTag3dLocator {
    fn locate(&mut self, source: &mut S) -> Result<Vec<LocationData>> {
        let Some(frame) = source.get_frame()? else {
            return Ok(Vec::new());
        };

        let image = Image::new(frame.width() as u32, frame.height() as u32, frame.bytes());
        let locations: Vec<_> = self.detector.detect(image).into_iter().map(|detection| {
            let pose = detection.estimate_pose(&self.intrinsics, self.tag_size);
            let mut builder = LocationData::builder();
            builder.x(pose.pos.x);
            builder.y(-pose.pos.y);
            builder.z(pose.pos.z);

            builder.yaw(-pose.rot.pitch_deg());
            builder.pitch(-pose.rot.roll_deg());
            builder.roll(pose.rot.yaw_deg());
            
            builder.id(detection.id());

            builder.build().unwrap()
        }).collect();

        Ok(locations)
    }
}
