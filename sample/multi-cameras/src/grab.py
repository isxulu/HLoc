from pypylon import pylon
import threading
import cv2
import argparse


def arg_parse():
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument('--rate', default=20, type=int, help='Sample rate.')
    arg_parser.add_argument('--path1', type=str, default='./cam0/data/', help='Cam0 path.')
    arg_parser.add_argument('--path2', type=str, default='./cam1/data/', help='Cam1 path.')
    arg_parser.add_argument('--exposure_time', default=10000, type=int, help='Exposure time.')
    arg_parser.add_argument('--width', default=2448, type=int, help="Width of image.")
    arg_parser.add_argument('--height', default=2048, type=int, help="Height of image.")

    return arg_parser.parse_args()


def configure_camera(camera: pylon.InstantCamera, idx, exposure_time):
    max_width, max_height = 2472, 2064
    
    camera.Open()
    camera.ExposureTime.SetValue(exposure_time)
    camera.PixelFormat.SetValue("Mono8")
    camera.PtpEnable.SetValue(True)
    if idx == 0:
        camera.TriggerMode.SetValue('On')
        camera.LineSelector.SetValue('Line3')
        camera.LineMode.SetValue('Input')
        camera.TriggerSelector.SetValue('FrameStart')
        camera.TriggerSource.SetValue('Line3')
        camera.TriggerActivation.SetValue('RisingEdge')
    else:
        camera.BslPeriodicSignalSelector.SetValue('PeriodicSignal1')
        camera.BslPeriodicSignalPeriod.SetValue(1_000_000//args.rate)
        camera.BslPeriodicSignalDelay.SetValue(0)
        camera.TimerSelector.SetValue('Timer1')
        camera.TimerTriggerSource.SetValue('PeriodicSignal1')
        camera.TimerDuration.SetValue(100)
        camera.TriggerSelector.SetValue('FrameStart')
        camera.TriggerMode.SetValue('On')
        camera.TriggerSource.SetValue('PeriodicSignal1')
        camera.LineSelector.SetValue('Line3')
        camera.LineMode.SetValue('Output')
        camera.LineSource.SetValue('Timer1Active')

    camera.Height.SetValue(args.height)
    camera.Width.SetValue(args.width)
    camera.OffsetX.SetValue((max_width-args.width)//2)
    camera.OffsetY.SetValue((max_height-args.height)//2)


def save_image(filepath, image_data, width, height):
    image = image_data.reshape((height, width))
    cv2.imwrite(filepath, image)


def grab(cam, idx):
    path = args.path1 if idx == 0 else args.path2
    if path[-1] != '/':
        path = path + '/'
    while grab_tag:
        grab_result = cam.RetrieveResult(1000000, pylon.TimeoutHandling_ThrowException)
        save_image(path+f'{grab_result.TimeStamp}.png', grab_result.Array, args.width, args.height)


def main():
    global grab_tag

    try:
        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()
        
        assert len(devices) >= 2, "At least 2 cameras are required."

        cameras = []
        threads = []

        for i in range(2):
            cam = pylon.InstantCamera()
            cam.Attach(tlFactory.CreateDevice(devices[i]))
            configure_camera(cam, i, args.exposure_time)
            cameras.append(cam)

            cam.StartGrabbing()

            cam.RegisterConfiguration(pylon.SoftwareTriggerConfiguration(), pylon.RegistrationMode_ReplaceAll, pylon.Cleanup_Delete)

            thread = threading.Thread(target=grab, args=(cam, i))
            threads.append(thread)

        print('Start grabbing.')
        for thread in threads: 
            thread.start()

        while True:
            pass
    except KeyboardInterrupt:
        grab_tag = False
        for thread in threads:
            thread.join()
        for cam in cameras:
            cam.Close()
        print('\nEnd grabbing.')
    except Exception:
        print('Unkonwn error!')


if __name__ == "__main__":
    args = arg_parse()

    grab_tag = True

    main()

