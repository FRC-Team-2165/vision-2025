use apriltag_rs::{CameraIntrinsics, TagFamily};
use vistream::{Camera, CameraConfig, FrameSource};
use vistream::stream::LocateStream;
use vistream::frame::{RGB, MJPG, Luma};
use vistream::transform::{JPGUnpacker, Rotate, Rotation, Convert};

use std::sync::{Arc};
use std::sync::atomic::{AtomicBool, Ordering};

mod apriltag;
use crate::apriltag::AprilTag3dLocator;

// port allocations
//
// 1180: upper stripcam Frame feed
// 1181: upper stripcam apriltag LocationData feed
// 1182: lower stripcam Frame feed
// 1183: lower stripcam apriltag LocationData feed
// 1184: picam Frame feed
// 1185: picam apriltag LocationData feed
// 1186: fisheye Frame feed

fn main() {

    // not actually unsafe. Just a vistream safety precaution
    unsafe{vistream::init().unwrap();}

    // Upper coral camera
    let upper_intrinsics = CameraIntrinsics {
        fx: 273.12925362,
        fy: 274.45475515,
        cx: 235.46841613,
        cy: 318.0861292,
    };
    
    // Lower coral camera
    // TODO: These need to be calibrated for the new camera
    // They're probably close, but not sufficiently correct.
    let lower_intrinsics = CameraIntrinsics {
        fx: 264.40423797,
        fy: 261.96072927,
        cx: 237.85975668,
        cy: 326.9399546,
    };

    // Algae pickup cam
    // picam
    let picam_intrinsics = CameraIntrinsics {
        fx: 944.01264457,
        fy: 943.41211612,
        cx: 234.68075873,
        cy: 325.99567316,
    };

    let tag_size = 0.1651;

    let mut cfg = CameraConfig::default();
   
    #[cfg(debug_assertions)]
    cfg.server_exe("/home/bisonbots/dev/vistream/target/debug/vistream-camera-server");
    #[cfg(not(debug_assertions))]
    cfg.server_exe("/home/bisonbots/dev/vistream/target/release/vistream-camera-server");
    
    cfg.conn_timeout(std::time::Duration::from_secs(1));
    cfg.width(640);
    cfg.height(480);

    let mut cameras_running = false;

    let upper = match Camera::<MJPG>::new("upper", cfg.clone()) {
        Ok(upper) => {
            let upper = JPGUnpacker::<Luma, _>::new(upper);
            let mut upper = Rotate::new(Rotation::Clockwise90, upper);
            match upper.start() {
                Ok(_) => {
                    let locator = AprilTag3dLocator::new(&[TagFamily::Tag36h11], upper_intrinsics, tag_size);
                    match LocateStream::launch("0.0.0.0:1181".parse().unwrap(), upper, locator) {
                        Ok(stream) => {
                            cameras_running = true;
                            Some(stream)
                        }
                        Err(e) => {
                            eprintln!("Something went wrong starting the upper locate stream: {}", e);
                            None
                        }
                    }
                }
                Err(e) => {
                    eprintln!("Something went wrong starting upper cam: {}", e);
                    None
                }
            }
        }
        Err(e) => {
            eprintln!("Could not start the upper stream: {}", e);
            None
        }
    };

    let lower = match Camera::<MJPG>::new("lower", cfg.clone()) {
        Ok(lower) => {
            let lower = JPGUnpacker::<Luma, _>::new(lower);
            let mut lower = Rotate::new(Rotation::Clockwise90, lower);
            match lower.start() {
                Ok(_) => {
                    let locator = AprilTag3dLocator::new(&[TagFamily::Tag36h11], lower_intrinsics, tag_size);
                    match LocateStream::launch("0.0.0.0:1183".parse().unwrap(), lower, locator) {
                        Ok(stream) => {
                            cameras_running = true;
                            Some(stream)
                        }
                        Err(e) => {
                            eprintln!("Something went wrong starting the lower locate stream: {}", e);
                            None
                        }
                    }
                }
                Err(e) => {
                    eprintln!("Something went wrong starting lower cam: {}", e);
                    None
                }
            }
        }
        Err(e) => {
            eprintln!("Could not start the lower stream: {}", e);
            None
        }
    };

    let picam = match Camera::<RGB>::new("pleco", cfg) {
        Ok(picam) => {
            let picam = Convert::new(picam);
            let mut picam = Rotate::new(Rotation::Counter90, picam);
            match picam.start() {
                Ok(_) => {
                    let locator = AprilTag3dLocator::new(&[TagFamily::Tag36h11], picam_intrinsics, tag_size);
                    match LocateStream::launch("0.0.0.0:1185".parse().unwrap(), picam, locator) {
                        Ok(stream) => {
                            cameras_running = true;
                            Some(stream)
                        }
                        Err(e) => {
                            eprintln!("Something went wrong starting the picam locate stream: {}", e);
                            None
                        }
                    }
                }
                Err(e) => {
                    eprintln!("Something went wrong starting picam cam: {}", e);
                    None
                }
            }
        }
        Err(e) => {
            eprintln!("Could not start the picam stream: {}", e);
            None
        }
    };

    // alias for fisheye lens: goldie

    if !cameras_running {
        eprintln!("None of the requested cameras are functioning!");
        std::process::exit(1);
    }

    let stop_flag = Arc::new(AtomicBool::new(false));

    let handler_stop_flag = stop_flag.clone();
    ctrlc::set_handler(move || {
        handler_stop_flag.store(true, Ordering::Release);
    }).expect("SIGTERM handler could not be set");
    
    println!("Press Ctrl-C to stop");
    while !stop_flag.load(Ordering::Acquire) {}

    if let Some(locator) = lower {
        let _ = locator.stop();
    }
    if let Some(locator) = upper {
        let _ = locator.stop();
    }
    if let Some(locator) = picam {
        let _ = locator.stop();
    }
}

