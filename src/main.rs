use apriltag_rs::{CameraIntrinsics, TagFamily};
use vistream::{Camera, CameraConfig, FrameSource};
use vistream::camera::FrameRateLimiter;
use vistream::stream::{LocateStream, FrameStream};
use vistream::frame::{RGB, MJPG, Luma};
use vistream::transform::{JPGUnpacker, Rotate, Rotation, Convert, JPGSource};
use vistream::transform::{Compressor, Subsamp};

use std::sync::{Arc};
use std::sync::atomic::{AtomicBool, Ordering};

mod apriltag;
use crate::apriltag::AprilTag3dLocator;

// port allocations
//
// 1180: upper stripcam Frame feed
// 1181: upper-global apriltag LocationData feed
// 1183: lower-global apriltag LocationData feed
// 1184: picam Frame feed
// 1185: picam apriltag LocationData feed
// 1186: fisheye Frame feed

fn main() {

    // not actually unsafe. Just a vistream safety precaution
    unsafe{vistream::init().unwrap();}

    // FIXME: Update calibration for new global-shutter cameras
    // Upper coral camera
    let upper_intrinsics = CameraIntrinsics {
        fx: 722.97330759,
        fy: 722.15492054,
        cx: 248.12094102,
        cy: 307.60200108,
    };
 //   [[722.97330759   0.         248.12094102]
 // [  0.         722.15492054 307.60200108]
 // [  0.           0.           1.        ]]


    // Lower coral camera
    // FIXME: These need to be calibrated for the new camera
    let lower_intrinsics = CameraIntrinsics {
        fx: 715.98368839,
        fy: 717.41934156,
        cx: 284.7698147,
        cy: 329.76605822,
    };
 //    [[715.98368839   0.         284.7698147 ]
 // [  0.         717.41934156 329.76605822]
 // [  0.           0.           1.        ]]


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

    let upper_frames = match Camera::<MJPG>::new("upper-global", cfg.clone()) {
        Ok(upper) => {
            let compressor = match Compressor::new() {
                Ok(mut comp) => {
                    let _ = comp.set_quality(8);
                    let _ = comp.set_subsamp(Subsamp::Sub1x4);
                    Some(comp)
                }
                Err(_) => {
                    None
                }
            };
            if let Some(compressor) = compressor {
                let upper = FrameRateLimiter::new_framerate(upper, 15.0);
                let upper = JPGUnpacker::<RGB, _>::new(upper);
                let upper = Rotate::new(Rotation::Clockwise90, upper);
                let mut upper = JPGSource::new_with_compressor(upper, compressor);
    
                match upper.start() {
                    Ok(_) => {
                        match FrameStream::launch("0.0.0.0:1180".parse().unwrap(), upper) {
                            Ok(stream) => {
                                cameras_running = true;
                                Some(stream)
                            }
                            Err(e) => {
                                eprintln!("Something went wrong starting upper frame stream: {}", e);
                                None
                            }
                        }
                    }
                    Err(_) => {
                        None
                    }
                }
    
            } else {
                None
            }
        }
        Err(_) => {
            None
        }
    };
    
    let picam_frames = match Camera::<RGB>::new("pleco", cfg.clone()) {
        Ok(picam) => {
            let compressor = match Compressor::new() {
                Ok(mut comp) => {
                    let _ = comp.set_quality(8);
                    let _ = comp.set_subsamp(Subsamp::Sub1x4);
                    Some(comp)
                }
                Err(_) => {
                    None
                }
            };
            if let Some(compressor) = compressor {
                let picam = FrameRateLimiter::new_framerate(picam, 15.0);
                let picam = Rotate::new(Rotation::Counter90, picam);
                let mut picam = JPGSource::new_with_compressor(picam, compressor);
                // FIXME This seems to not actually matter. That's a problem.
                match picam.start() {
                    Ok(_) => {
                        match FrameStream::launch("0.0.0.0:1184".parse().unwrap(), picam) {
                            Ok(stream) => {
                                cameras_running = true;
                                Some(stream)
                            }
                            Err(e) => {
                                eprintln!("Something went wrong starting picam frame stream: {}", e);
                                None
                            }
                        }
                    }
                    Err(_) => {
                        None
                    }
                }
    
            } else {
                None
            }
        }
        Err(_) => {
            None
        }
    };
    
    let goldie_frames = match Camera::<MJPG>::new("goldie", cfg.clone()) {
        Ok(goldie) => {
            let compressor = match Compressor::new() {
                Ok(mut comp) => {
                    let _ = comp.set_quality(8);
                    let _ = comp.set_subsamp(Subsamp::Sub1x4);
                    Some(comp)
                }
                Err(_) => {
                    None
                }
            };
            if let Some(compressor) = compressor {
                let goldie = FrameRateLimiter::new_framerate(goldie, 15.0);
                let goldie = JPGUnpacker::<RGB, _>::new(goldie);
                let goldie = Rotate::new(Rotation::Clockwise180, goldie);
                let mut goldie = JPGSource::new_with_compressor(goldie, compressor);
                match goldie.start() {
                    Ok(_) => {
                        match FrameStream::launch("0.0.0.0:1186".parse().unwrap(), goldie) {
                            Ok(stream) => {
                                cameras_running = true;
                                Some(stream)
                            }
                            Err(e) => {
                                eprintln!("Something went wrong starting goldie frame stream: {}", e);
                                None
                            }
                        }
                    }
                    Err(_) => {
                        None
                    }
                }

            } else {
                None
            }
        }
        Err(_) => {
            None
        }
    };


    let upper = match Camera::<MJPG>::new("upper-global", cfg.clone()) {
        Ok(upper) => {
            let upper = FrameRateLimiter::new(upper, std::time::Duration::from_millis(8));
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

    let lower = match Camera::<MJPG>::new("lower-global", cfg.clone()) {
        Ok(lower) => {
            let lower = FrameRateLimiter::new(lower, std::time::Duration::from_millis(8));
            let lower = JPGUnpacker::<Luma, _>::new(lower);
            let mut lower = Rotate::new(Rotation::Counter90, lower);
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
            let picam = FrameRateLimiter::new(picam, std::time::Duration::from_millis(8));
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
        let _ = dbg!(locator.stop());
    }
    if let Some(locator) = upper {
        let _ = dbg!(locator.stop());
    }
    if let Some(locator) = picam {
        let _ = dbg!(locator.stop());
    }
    
    if let Some(stream) = upper_frames {
        let _ = dbg!(stream.stop());
    }
    if let Some(stream) = picam_frames {
        let _ = dbg!(stream.stop());
    }
    if let Some(stream) = goldie_frames {
        let _ = dbg!(stream.stop());
    }


    std::thread::sleep(std::time::Duration::from_millis(50));

}
