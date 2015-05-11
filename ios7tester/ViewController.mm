//
//  ViewController.m
//  ios7tester
//
//  Created by Sam Fox on 10/4/13.
//  Copyright (c) 2013 Sam Fox. All rights reserved.
//



#import "ViewController.h"
#import "CVDelegate.h"
#import "templateMatcher.h"
#import "circleDetector.h"
#import "OpticalFlow.h"
#import "freak.h"

@interface ViewController ()

@end

@implementation ViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
   // self.freek = [[freak alloc] init];
	self.camera = [[CvVideoCamera alloc] initWithParentView:self.imgView];

    self.camera.delegate = nil;
    self.tableView.delegate = self;
    self.camera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    
    self.camera.defaultAVCaptureSessionPreset = AVCaptureSessionPresetMedium;
    self.camera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    
    self.camera.defaultFPS = 30;
    self.camera.grayscaleMode = NO;
    
    [self.camera start];
    AVCaptureDevice *device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    [device lockForConfiguration:nil];
    [device unlockForConfiguration];
    // Do any additional setup after loading the view, typically from a nib.
}

- (NSInteger) tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section{
    return 7;
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath{
    static NSString *identifier = @"CV_Algo";
    UITableViewCell *cell = [tableView dequeueReusableCellWithIdentifier:identifier];
    switch (indexPath.row) {
        case 0:
            cell.textLabel.text = @"Lucas-Kanade Sparse Optical Flow";
            break;
        case 1:
            cell.textLabel.text = @"template matching";
            break;
        case 2:
            cell.textLabel.text = @"Hough Circle Detector";
            break;
        case 3:
            cell.textLabel.text = @"FREAK: Fast Retina Keypoints";
            break;
        case 4:
            cell.textLabel.text = @"Farneback Dense Optical Flow";
            break;
        case 5:
            cell.textLabel.text = @"Camera Calibration";
            break;
        case 6:
            cell.textLabel.text = @"Structure from motion";
            break;
        default:
            break;
    }
    return cell;
}

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath{
    if (self.freek) {
        [self.freek.openGLView removeFromSuperview];
        
    }
    if (self.sfm) {
        [self.sfm.openGLView removeFromSuperview];
    }
    switch (indexPath.row) {
            case 0:{
                self.imgView.backgroundColor = [UIColor yellowColor];
                self.opticalFlow = [[OpticalFlow alloc] init];
                self.opticalFlow.type = LUCAS_KANADE_SPARSE;
                self.camera.delegate = self.opticalFlow;

                break;
            }
            case 1:{
                self.templateDel = [[templateMatcher alloc] init];
                self.camera.delegate = self.templateDel;
                self.imgView.backgroundColor = [UIColor purpleColor];

                break;
            }
            case 2:{
                self.circleD = [[circleDetector alloc] init];
                self.camera.delegate = self.circleD;
                self.imgView.backgroundColor = [UIColor blueColor];
                break;
            }
            case 3:{
                self.freek = [[freak alloc] init];
                self.camera.delegate = self.freek;
                self.imgView.backgroundColor = [UIColor greenColor];
                [self.imgView addSubview:self.freek.openGLView];
                [self.imgView bringSubviewToFront: self.freek.openGLView];
                self.imgView.userInteractionEnabled = TRUE;
                self.freek.openGLView.exclusiveTouch = TRUE;
                self.freek.openGLView.userInteractionEnabled = TRUE;
                break;
            }
            case 4:{
                self.opticalFlow = [[OpticalFlow alloc] init];
                self.opticalFlow.type = FARNEBECK_DENSE;
                self.camera.delegate = self.opticalFlow;
                self.imgView.backgroundColor = [UIColor redColor];
                break;
            }
            case 5:{
                self.calibrate = [[Calibrator alloc] init];
                self.camera.delegate = self.calibrate;
                self.imgView.backgroundColor = [UIColor redColor];
                break;
            }
            case 6:{
                self.sfm = [[StructureFromMotion alloc] init];
                self.camera.delegate = self.sfm;
                self.imgView.backgroundColor = [UIColor greenColor];
                [self.imgView addSubview:self.sfm.openGLView];
                [self.imgView bringSubviewToFront: self.sfm.openGLView];
                self.imgView.userInteractionEnabled = TRUE;
                self.sfm.openGLView.exclusiveTouch = TRUE;
                self.sfm.openGLView.userInteractionEnabled = TRUE;
            }
            default:
                break;
        }
}

- (IBAction) saveImage:(id)sender{
    self.sfm.shouldSave = YES;
}

-(IBAction)run:(id)sender{
    self.sfm.shouldRun = !self.sfm.shouldRun;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
